from __future__ import annotations

import logging
import math
import os
import random
import sys
import time
from pathlib import Path
from threading import RLock
from typing import Any, Optional

from sbsvf_api.control_pb2 import CtrlCmd, CtrlMode
from sbsvf_api.object_pb2 import ObjectState, RoadObjectType
from sbsvf_api.scenario_pb2 import ScenarioPack
from google.protobuf.struct_pb2 import Struct

logger = logging.getLogger(__name__)


class PCLAAgentAV:
    """
    PCLA AV adapter (CARLA-backed):
    - init(): connect to CARLA, prepare runtime
    - reset(): load map, spawn ego, build route, init PCLA agent
    - step(): update CARLA world from obs, tick, run PCLA, return CtrlCmd
    """

    def __init__(self, output_dir: str, cfg: dict):
        self._output_dir = Path(output_dir)
        self.config = cfg or {}
        self._pcla_cfg = self.config.get("pcla", self.config)
        self._carla_cfg = self._pcla_cfg.get("carla", self.config.get("carla", {}))

        self._agent_name = self._pcla_cfg.get("agent") or self._pcla_cfg.get(
            "agent_name"
        )
        if not self._agent_name:
            self._agent_name = os.environ.get("PCLA_AGENT", "carl_plant")

        self._route_path_cfg = self._pcla_cfg.get("route_path")
        if not self._route_path_cfg:
            self._route_path_cfg = os.environ.get("PCLA_ROUTE")
        self._route_wp_distance = float(
            self._pcla_cfg.get("route_waypoint_distance", 2.0)
        )
        self._route_draw = bool(self._pcla_cfg.get("route_draw", False))
        self._route_cache_dir = Path(
            self._pcla_cfg.get("route_cache_dir", "pcla_routes")
        )

        pcla_root_cfg = self._pcla_cfg.get("pcla_root")
        if pcla_root_cfg:
            self._pcla_root = Path(pcla_root_cfg)
        else:
            self._pcla_root = Path(__file__).resolve().parent / "PCLA"

        self._host = os.environ.get("CARLA_HOST", self.config.get("host", "localhost"))

        self._port = int(os.environ.get("CARLA_PORT", self.config.get("port", 2000)))

        self._timeout = float(self._carla_cfg.get("timeout", 10.0))

        self._carla_root = self._carla_cfg.get("carla_root") or os.environ.get(
            "CARLA_ROOT"
        )
        self._carla_egg = self._carla_cfg.get("carla_egg")
        self._sync = bool(self._carla_cfg.get("sync", True))
        self._no_rendering = bool(self._carla_cfg.get("no_rendering", False))
        self._fixed_delta_seconds = self._carla_cfg.get("fixed_delta_seconds", 0.05)

        self._ego_role_name = self._carla_cfg.get("ego_role_name", "hero")
        self._ego_bp_id = self._carla_cfg.get("ego_bp_id", "vehicle.tesla.model3")
        self._yaw_sign = float(self._carla_cfg.get("yaw_sign", -1.0))
        self._yaw_offset_deg = float(self._carla_cfg.get("yaw_offset_deg", 0.0))
        self._yaw_unit = str(self._carla_cfg.get("yaw_unit", "rad")).lower()
        self._warned_yaw_unit_mismatch = False
        self._max_wait_sec = float(self._carla_cfg.get("max_wait_sec", 10.0))
        self._spawn_z_offset = float(self._carla_cfg.get("spawn_z_offset", 3.0))
        self._xodr_root = Path(self._carla_cfg.get("xodr_root", "/mnt/map/xodr"))
        self._carla_map_name = self._carla_cfg.get("carla_map_name", None)
        self._max_retry_times = int(self._carla_cfg.get("max_retry_times", 10))

        self._original_settings = None
        self._spawned_actor_ids = set()
        self._carla = None

        self._client = None
        self._world = None
        self._map = None
        self._vehicle = None
        self._pcla = None
        self._other_actors: list[Any] = []
        self._other_actor_types: list[RoadObjectType] = []

        self._sps: Optional[ScenarioPack] = None
        self._quit_flag = False
        self._last_error: Optional[str] = None
        self._state_lock = RLock()

    def _ensure_carla_imports(self) -> None:
        if self._carla is not None:
            return

        entries: list[str] = []
        if self._carla_root:
            root = Path(self._carla_root)
            entries.append(str(root / "PythonAPI"))
            entries.append(str(root / "PythonAPI" / "carla"))
            dist_dir = root / "PythonAPI" / "carla" / "dist"
            if self._carla_egg is None and dist_dir.exists():
                for ext in ("*.whl", "*.egg"):
                    matches = sorted(dist_dir.glob(ext))
                    if matches:
                        self._carla_egg = str(matches[0])
                        break

        if self._carla_egg:
            entries.append(str(self._carla_egg))

        for entry in entries:
            if entry and entry not in sys.path:
                sys.path.insert(0, entry)

        try:
            import carla  # type: ignore
        except Exception as e:
            raise RuntimeError("CARLA Python API not available") from e

        self._carla = carla

    def _ensure_pcla_imports(self) -> None:
        pcla_root = self._pcla_root
        if not pcla_root.exists():
            raise FileNotFoundError(f"PCLA root not found: {pcla_root}")
        if str(pcla_root) not in sys.path:
            sys.path.insert(0, str(pcla_root))

    def _connect(self) -> None:
        self._ensure_carla_imports()
        if self._client is not None:
            return
        logger.info("Connecting to CARLA server at %s:%s...", self._host, self._port)
        client = self._carla.Client(self._host, self._port)
        # Temporarily set a short timeout for the initial connection attempt
        client.set_timeout(5.0)
        world = client.get_world()
        if world is None:
            raise RuntimeError("Failed to connect to CARLA server")
        self._client = client
        self._client.set_timeout(self._timeout)
        logger.info("Connected to CARLA server at %s:%s", self._host, self._port)

    def _find_ego_vehicle_once(self):
        if self._world is None:
            return None
        actors = self._world.get_actors().filter("vehicle.*")
        for actor in actors:
            role = actor.attributes.get("role_name", "")
            if role == self._ego_role_name:
                return actor
        return None

    def _wait_for_ego(self, timeout_s: float):
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            logger.info(
                "Searching for ego vehicle with role_name=%r...", self._ego_role_name
            )
            actor = self._find_ego_vehicle_once()
            if actor is not None:
                return actor
            time.sleep(0.05)
        return None

    def _extract_xyz(self, pos):
        if pos is None:
            return 0.0, 0.0, 0.0
        world = getattr(pos, "world", None)
        if world is not None and hasattr(world, "x"):
            return float(world.x), float(world.y), float(world.z)
        return (
            float(getattr(pos, "x", 0.0)),
            float(getattr(pos, "y", 0.0)),
            float(getattr(pos, "z", 0.0)),
        )

    def _extract_yaw(self, pos) -> float:
        if pos is None:
            return 0.0
        if hasattr(pos, "yaw"):
            return float(pos.yaw)
        world = getattr(pos, "world", None)
        if world is not None and hasattr(world, "h"):
            return float(world.h)
        if hasattr(pos, "h"):
            return float(pos.h)
        return 0.0

    def _has_message_field(self, msg, field_name: str) -> bool:
        """Return True only when a protobuf message field is explicitly set."""
        has_field = getattr(msg, "HasField", None)
        if callable(has_field):
            try:
                return bool(has_field(field_name))
            except Exception:
                return False
        return getattr(msg, field_name, None) is not None

    def _to_carla_location(self, pos) -> Any:
        if self._carla is not None:
            if isinstance(pos, self._carla.Location):
                return pos
            if isinstance(pos, self._carla.Transform):
                return pos.location
        x, y, z = self._extract_xyz(pos)
        y = float(y) * self._yaw_sign
        return self._carla.Location(
            x=float(x),
            y=y,
            z=float(z),
        )

    def _input_yaw_to_degrees(self, yaw: float) -> float:
        yaw = float(yaw)
        if self._yaw_unit in ("deg", "degree", "degrees"):
            return yaw
        if self._yaw_unit in ("rad", "radian", "radians"):
            if abs(yaw) > (2.0 * math.pi + 0.5) and not self._warned_yaw_unit_mismatch:
                logger.warning(
                    "Input yaw looks like degrees (%.3f) while yaw_unit='rad'. "
                    "Set carla.yaw_unit='deg' if your observation yaw is in degrees.",
                    yaw,
                )
                self._warned_yaw_unit_mismatch = True
            return math.degrees(yaw)

        logger.warning(
            "Unknown carla.yaw_unit=%r, fallback to radians.", self._yaw_unit
        )
        self._yaw_unit = "rad"
        return math.degrees(yaw)

    def _input_yaw_rate_to_deg_per_sec(self, yaw_rate: float) -> float:
        yaw_rate = float(yaw_rate)
        if self._yaw_unit in ("deg", "degree", "degrees"):
            return yaw_rate
        return math.degrees(yaw_rate)

    def _to_carla_yaw(self, yaw: float) -> float:
        return self._yaw_sign * self._input_yaw_to_degrees(yaw) + self._yaw_offset_deg

    def init(self, sps: ScenarioPack) -> None:
        self._sps = sps
        for i in range(self._max_retry_times):
            logger.info("Initializing CARLA world (attempt %d)...", i + 1)
            try:
                self._connect()
                break
            except Exception:
                logger.exception("Failed to initialize CARLA world (attempt %d)", i + 1)
                if i == self._max_retry_times - 1:
                    raise RuntimeError(
                        "Exceeded maximum retry attempts for CARLA connection"
                    )
                time.sleep(1.0)
        logger.info("CarlaAgentAV initialized.")

    def reset(
        self,
        output_dir: Path,
        sps: ScenarioPack,
        init_obs: Optional[list[ObjectState]] = None,
    ) -> CtrlCmd:
        with self._state_lock:
            self._output_dir = Path(output_dir)
            self._sps = sps
            self._quit_flag = False
            self._last_error = None

            self._connect()

            # Critical ordering for CARLA stability:
            # fully cleanup old agents/actors before loading/changing world.
            if self._pcla is not None:
                try:
                    self._pcla.cleanup()
                except Exception:
                    logger.exception("Failed to cleanup previous PCLA instance")
                finally:
                    self._pcla = None
                    self._vehicle = None

            try:
                self._destroy_spawned_actors()
            except Exception:
                logger.exception("Failed to destroy spawned actors before reset")

            self._ensure_world(sps)
            self._apply_world_settings()
            self._set_data_provider()

            if self._vehicle is None:
                self._vehicle = self._spawn_ego(init_obs, self._sps)
            print(
                f"Ego vehicle spawned at {self._vehicle.get_transform()}*******************************************"
            )
            route_path = self._resolve_route_path(sps, init_obs)
            self._pcla = self._build_pcla(route_path)

            return self.step(
                obs=init_obs if init_obs is not None else [],
                time_stamp_ns=0,
            )

    def step(self, obs: list[ObjectState], time_stamp_ns: int) -> CtrlCmd:
        with self._state_lock:
            if self._pcla is None:
                return self._zero_control()

            try:
                self._update_and_tick(obs)
                action = self._pcla.get_action()
            except Exception as exc:
                self._quit_flag = True
                self._last_error = str(exc)
                logger.exception("PCLA step failed: %s", exc)
                return self._zero_control()

            if action is None:
                return self._zero_control()

            yaw_sign = self._yaw_sign if abs(self._yaw_sign) > 1e-6 else 1.0
            steer_sv = float(action.steer) / yaw_sign

            payload_struct = Struct()
            payload_struct.update(
                {
                    "throttle": float(action.throttle),
                    "brake": float(action.brake),
                    "steer": steer_sv,
                }
            )
            print(
                f"Action from PCLA: throttle={action.throttle:.3f}, brake={action.brake:.3f}, steer={steer_sv:.3f}==============================================="
            )
            return CtrlCmd(mode=CtrlMode.THROTTLE_STEER_BREAK, payload=payload_struct)

    def stop(self) -> None:
        with self._state_lock:
            if self._pcla is not None:
                try:
                    self._pcla.cleanup()
                except Exception:
                    logger.exception("Failed to cleanup PCLA instance")
                self._pcla = None

            try:
                self._destroy_spawned_actors()
            except Exception:
                logger.exception("Failed to destroy spawned actors")

            if self._world is not None and self._original_settings is not None:
                try:
                    self._world.apply_settings(self._original_settings)
                    logger.info("Restored original CARLA world settings.")
                except Exception as e:
                    logger.warning(f"Failed to restore CARLA world settings: {e}")

            self._vehicle = None
            self._world = None
            self._map = None
            self._quit_flag = True

    def should_quit(self) -> bool:
        if self._quit_flag:
            logger.info("PCLAAgentAV.should_quit: quit_flag set (%s)", self._last_error)
            return True
        return False

    def _zero_control(self) -> CtrlCmd:
        return CtrlCmd(mode=CtrlMode.NONE)

    def _set_data_provider(self) -> None:
        self._ensure_pcla_imports()
        from leaderboard_codes.carla_data_provider import CarlaDataProvider

        if self._client is not None:
            CarlaDataProvider.set_client(self._client)
        if self._world is not None:
            CarlaDataProvider.set_world(self._world)

    def _build_pcla(self, route_path: Path):
        self._ensure_pcla_imports()

        import PCLA as pcla_mod

        os.chdir(str(self._pcla_root))

        return pcla_mod.PCLA(
            self._agent_name, self._vehicle, str(route_path), self._client
        )

    def _resolve_route_path(
        self, sps: Optional[ScenarioPack], init_obs: Optional[list[ObjectState]]
    ) -> Path:
        if self._route_path_cfg:
            route_path = Path(self._route_path_cfg)
            if not route_path.is_absolute():
                route_path = (self._pcla_root / route_path).resolve()
            return route_path

        route_dir = self._route_cache_dir
        if not route_dir.is_absolute():
            route_dir = (self._output_dir / route_dir).resolve()
        route_dir.mkdir(parents=True, exist_ok=True)

        route_name = "route.xml"
        if sps is not None:
            scenario_name = getattr(sps, "name", None)
            if scenario_name:
                route_name = f"{scenario_name}.route.xml"
        route_path = route_dir / route_name

        requested_start_pos = self._get_spawn_position(init_obs, sps)

        start_pos = requested_start_pos
        if start_pos is None:
            raise RuntimeError("Cannot resolve route start position for PCLA")

        start_loc = self._to_carla_location(start_pos)
        start_loc.z += self._spawn_z_offset

        goal_pos = self._get_goal_position(sps)
        goal_candidates = []
        if goal_pos is not None:
            goal_loc = self._to_carla_location(goal_pos)
            goal_loc = self._carla.Location(
                x=float(goal_loc.x),
                y=float(goal_loc.y),
                z=float(goal_loc.z),
            )
            goal_loc.z += self._spawn_z_offset
            goal_candidates.append(goal_loc)
        elif self._map is not None:
            spawn_points = list(self._map.get_spawn_points())
            random.shuffle(spawn_points)
            for sp in spawn_points:
                loc = sp.location
                candidate_loc = self._carla.Location(
                    x=float(loc.x),
                    y=float(loc.y),
                    z=float(loc.z) + self._spawn_z_offset,
                )
                goal_candidates.append(candidate_loc)

        if not goal_candidates:
            raise RuntimeError("Cannot resolve route goal position for PCLA")

        self._ensure_pcla_imports()
        import PCLA as pcla_mod

        max_candidates = 1 if goal_pos is not None else min(len(goal_candidates), 30)
        chosen_goal = None
        chosen_waypoints = None
        last_error: Optional[Exception] = None

        for goal_loc in goal_candidates[:max_candidates]:
            try:
                waypoints = pcla_mod.location_to_waypoint(
                    self._client,
                    start_loc,
                    goal_loc,
                    distance=self._route_wp_distance,
                    draw=self._route_draw,
                )
            except Exception as exc:
                last_error = exc
                continue

            if len(waypoints) >= 2:
                chosen_goal = goal_loc
                chosen_waypoints = waypoints
                break

        if chosen_waypoints is None:
            map_name = self._map.name if self._map is not None else "unknown"
            raise RuntimeError(
                "Failed to find a reachable route candidate "
                f"(map={map_name}, start={start_loc}, goals_tested={max_candidates})"
            ) from last_error

        # location_to_waypoint already returns a dense drivable path.
        # Keep only the endpoints in XML and let setup_route interpolate once.
        route_waypoints = [chosen_waypoints[0], chosen_waypoints[-1]]

        logger.info(
            "Resolved route waypoints: raw=%d stored=%d start=%s goal=%s",
            len(chosen_waypoints),
            len(route_waypoints),
            start_loc,
            chosen_goal,
        )
        pcla_mod.route_maker(route_waypoints, savePath=str(route_path))
        return route_path

    def _spawn_ego(self, init_obs: Optional[list[ObjectState]], sps: ScenarioPack):
        if self._world is None:
            raise RuntimeError("CARLA world not available")

        bp_lib = self._world.get_blueprint_library()
        try:
            ego_bp = bp_lib.find(self._ego_bp_id)
        except Exception:
            candidates = bp_lib.filter("vehicle.*")
            if not candidates:
                raise RuntimeError("No vehicle blueprints available in CARLA")
            ego_bp = candidates[0]

        if ego_bp.has_attribute("role_name"):
            ego_bp.set_attribute("role_name", self._ego_role_name)

        pos = self._get_spawn_position(init_obs, sps)
        if pos is None:
            raise RuntimeError("No spawn position available for ego vehicle")
        carla_pos = self._to_carla_location(pos)
        carla_pos.z += self._spawn_z_offset
        carla_rot = self._carla.Rotation(
            pitch=0.0,
            yaw=self._to_carla_yaw(self._extract_yaw(pos)),
            roll=0.0,
        )
        transform = self._carla.Transform(carla_pos, carla_rot)
        ego = self._world.try_spawn_actor(ego_bp, transform)
        if ego is None:
            logger.warning("Initial spawn failed, trying spawn points...")
            spawn_points = self._map.get_spawn_points() if self._map is not None else []
            spawn_points = sorted(
                spawn_points, key=lambda sp: sp.location.distance(carla_pos)
            )
            for sp in spawn_points:
                ego = self._world.try_spawn_actor(ego_bp, sp)
                if ego is not None:
                    break

        if ego is None:
            raise RuntimeError("Failed to spawn ego vehicle")

        if self._sync:
            self._world.tick()
        else:
            self._world.wait_for_tick()

        return ego

    def _ensure_world(self, sps: Optional[ScenarioPack]) -> None:
        if self._client is None:
            self._connect()
        carla_map_name = self._carla_map_name
        opendrive_path = None

        if sps is not None:
            maps = getattr(sps, "maps", None)
            if maps:
                try:
                    carla_map_name = maps.get("carla_map_name", carla_map_name)
                except Exception:
                    pass
                try:
                    opendrive_path = maps.get("xodr_path", None)
                except Exception:
                    opendrive_path = None

            if opendrive_path is None:
                map_name = getattr(sps, "map_name", None)
                if map_name:
                    opendrive_path = self._xodr_root / f"{map_name}.xodr"

        if hasattr(carla_map_name, "path"):
            carla_map_name = carla_map_name.path
        if hasattr(opendrive_path, "path"):
            opendrive_path = opendrive_path.path

        world = None
        if carla_map_name:
            world = self._client.load_world(str(carla_map_name), reset_settings=False)
        elif opendrive_path and hasattr(self._client, "generate_opendrive_world"):
            opendrive_path = Path(opendrive_path)
            if not opendrive_path.exists():
                logger.warning(
                    "OpenDRIVE path not found (%s); using current world.",
                    opendrive_path,
                )
            else:
                with open(opendrive_path, "r", encoding="utf-8") as f:
                    opendrive_str = f.read()
                world = self._client.generate_opendrive_world(
                    opendrive_str,
                    self._carla.OpendriveGenerationParameters(
                        vertex_distance=0.2,
                        max_road_length=3000.0,
                        wall_height=0,
                        additional_width=0.6,
                        smooth_junctions=True,
                        enable_mesh_visibility=True,
                    ),
                )

        if world is None:
            world = self._client.get_world()

        self._world = world
        self._map = world.get_map() if world else None
        if self._original_settings is None:
            self._original_settings = world.get_settings()

    def _get_goal_position(self, sps: Optional[ScenarioPack]):
        if sps is None:
            return None
        ego = getattr(sps, "ego", None)
        if ego is None:
            return None
        for attr in ("goal_config", "goal"):
            cfg = getattr(ego, attr, None)
            if cfg is None:
                continue
            if not self._has_message_field(cfg, "position"):
                continue
            pos = getattr(cfg, "position", None)
            if pos is not None:
                return pos
        if self._has_message_field(ego, "goal_position"):
            return getattr(ego, "goal_position", None)
        return None

    def _get_spawn_position(
        self, init_obs: Optional[list[ObjectState]], sps: Optional[ScenarioPack]
    ):
        if init_obs:
            try:
                return init_obs[0].kinematic
            except Exception:
                pass
        if sps is None:
            return None
        ego = getattr(sps, "ego", None)
        if ego is None:
            return None
        for attr in ("spawn_config", "spawn"):
            cfg = getattr(ego, attr, None)
            if cfg is None:
                continue
            if not self._has_message_field(cfg, "position"):
                continue
            pos = getattr(cfg, "position", None)
            if pos is not None:
                return pos
        return None

    def _apply_world_settings(self) -> None:
        if self._world is None:
            return
        settings = self._world.get_settings()
        settings.synchronous_mode = self._sync
        logger.info("Synchronous mode = %s", settings.synchronous_mode)
        settings.no_rendering_mode = self._no_rendering
        logger.info("No rendering mode = %s", settings.no_rendering_mode)
        if self._fixed_delta_seconds is not None:
            logger.info("Setting fixed_delta_seconds = %s", self._fixed_delta_seconds)
            settings.fixed_delta_seconds = float(self._fixed_delta_seconds)
        self._world.apply_settings(settings)

    def _update_and_tick(self, obs: list[ObjectState]) -> None:
        if self._world is None:
            return

        self._ensure_pcla_imports()
        from leaderboard_codes.carla_data_provider import CarlaDataProvider

        def pick_blueprint(obj_type: RoadObjectType):
            if self._world is None:
                return None
            bp_lib = self._world.get_blueprint_library()
            if obj_type == RoadObjectType.PEDESTRIAN:
                return bp_lib.find("walker.pedestrian.0001")
            elif obj_type == RoadObjectType.BUS:
                return bp_lib.find("vehicle.mitsubishi.fusorosa")
            elif obj_type == RoadObjectType.TRUCK:
                return bp_lib.find("vehicle.carlamotors.carlacola")
            elif obj_type == RoadObjectType.TRAILER:
                return bp_lib.find("vehicle.carlamotors.firetruck")
            elif obj_type == RoadObjectType.VAN:
                return bp_lib.find("vehicle.mercedes.sprinter")
            elif obj_type == RoadObjectType.MOTORCYCLE:
                return bp_lib.find("vehicle.vespa.zx125")
            elif obj_type == RoadObjectType.BICYCLE:
                return bp_lib.find("vehicle.bh.crossbike")
            else:
                candidates = bp_lib.filter("vehicle.*")

            if not candidates and obj_type != RoadObjectType.PEDESTRIAN:
                candidates = bp_lib.filter("vehicle.*")
            if not candidates:
                return None
            return candidates[0]

        def make_transform(kin, z_offset: float = 0.0):
            loc = self._to_carla_location(kin)
            if z_offset:
                loc.z += z_offset
            rot = self._carla.Rotation(
                pitch=0.0,
                yaw=self._to_carla_yaw(float(kin.yaw)),
                roll=0.0,
            )
            return self._carla.Transform(loc, rot)

        def apply_kinematic(actor, kin) -> None:
            if actor is None:
                return
            try:
                actor.set_transform(make_transform(kin))
            except Exception:
                logger.exception("Failed to set actor transform")

            speed = float(kin.speed)
            yaw_carla_deg = self._to_carla_yaw(float(kin.yaw))
            yaw_carla_rad = math.radians(yaw_carla_deg)
            vx = speed * math.cos(yaw_carla_rad)
            vy = speed * math.sin(yaw_carla_rad)
            vel = self._carla.Vector3D(vx, vy, 0.0)
            try:
                actor.set_target_velocity(vel)
            except Exception:
                try:
                    actor.set_velocity(vel)
                except Exception:
                    pass

            if abs(float(kin.yaw_rate)) > 1e-6:
                ang_z = self._input_yaw_rate_to_deg_per_sec(float(kin.yaw_rate))
                ang_z *= self._yaw_sign
                ang = self._carla.Vector3D(0.0, 0.0, ang_z)
                try:
                    actor.set_target_angular_velocity(ang)
                except Exception:
                    try:
                        actor.set_angular_velocity(ang)
                    except Exception:
                        pass

        if not obs:
            if self._sync:
                self._world.tick()
            else:
                self._world.wait_for_tick()
            CarlaDataProvider.on_carla_tick()
            return

        if self._vehicle is None:
            self._vehicle = self._spawn_ego(obs, self._sps)

        ego_state = obs[0].kinematic
        apply_kinematic(self._vehicle, ego_state)

        desired_count = max(len(obs) - 1, 0)
        while len(self._other_actors) < desired_count:
            self._other_actors.append(None)
            self._other_actor_types.append(RoadObjectType.UNKNOWN)
        while len(self._other_actors) > desired_count:
            actor = self._other_actors.pop()
            self._other_actor_types.pop()
            if actor is not None:
                try:
                    actor.destroy()
                except Exception:
                    logger.exception("Failed to destroy extra actor")

        for idx, obj in enumerate(obs[1:]):
            actor = self._other_actors[idx]
            obj_type = obj.type
            if (
                actor is None
                or (hasattr(actor, "is_alive") and not actor.is_alive)
                or self._other_actor_types[idx] != obj_type
            ):
                if actor is not None:
                    try:
                        actor.destroy()
                    except Exception:
                        logger.exception("Failed to destroy actor %s", idx)
                bp = pick_blueprint(obj_type)
                if bp is None:
                    logger.warning("No blueprint for object type %s", obj_type)
                    self._other_actors[idx] = None
                    self._other_actor_types[idx] = obj_type
                    continue
                if bp.has_attribute("role_name"):
                    bp.set_attribute("role_name", f"agent_{idx}")
                transform = make_transform(obj.kinematic, z_offset=self._spawn_z_offset)
                actor = self._world.try_spawn_actor(bp, transform)
                if actor is None:
                    logger.warning("Failed to spawn actor for index %s", idx)
                self._other_actors[idx] = actor
                self._other_actor_types[idx] = obj_type

            apply_kinematic(self._other_actors[idx], obj.kinematic)

        if self._sync:
            self._world.tick()
        else:
            self._world.wait_for_tick()
        CarlaDataProvider.on_carla_tick()

    def _destroy_spawned_actors(self) -> None:
        if self._world is None:
            self._vehicle = None
            self._other_actors.clear()
            self._other_actor_types.clear()
            return

        if self._vehicle is not None:
            try:
                self._vehicle.destroy()
            except Exception:
                logger.exception("Failed to destroy ego vehicle")
            self._vehicle = None

        for actor in list(self._other_actors):
            try:
                if actor is not None:
                    actor.destroy()
            except Exception:
                actor_id = getattr(actor, "id", "unknown")
                logger.exception("Failed to destroy actor %s", actor_id)

        self._other_actors.clear()
        self._other_actor_types.clear()
