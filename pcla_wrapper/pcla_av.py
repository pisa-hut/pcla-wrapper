from __future__ import annotations

import contextlib
import json
import logging
import math
import os
import re
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from threading import RLock
from types import SimpleNamespace
from typing import Any

from pisa_api.av import (
    AvError,
    AvPreconditionFailed,
    AvTimeout,
    AvUnavailable,
    ControlCommand,
    ControlMode,
    InitRequest,
    InitResponse,
    InvalidAvRequest,
    ObjectStateData,
    ObservationData,
    ObservedAgentData,
    ResetRequest,
    ResetResponse,
    RoadObjectType,
    ScenarioPackData,
    ShapeType,
    ShouldQuitResponse,
    StepRequest,
    StepResponse,
)

from .lifecycle import clear_dynamic_actors, destroy_actor, force_async_world_for_cleanup
from .profiles import validate_image_profile

logger = logging.getLogger(__name__)
PCLA_CWD_LOCK = RLock()
SHAPE_TOLERANCE = 1e-6

BLUEPRINT_CANDIDATES = {
    RoadObjectType.PEDESTRIAN: ("walker.pedestrian.0001", "walker.pedestrian.*", "walker.*"),
    RoadObjectType.BUS: ("vehicle.mitsubishi.fusorosa", "vehicle.*bus*", "vehicle.*"),
    RoadObjectType.TRUCK: ("vehicle.carlamotors.carlacola", "vehicle.*truck*", "vehicle.*"),
    RoadObjectType.SEMITRAILER: ("vehicle.carlamotors.carlacola", "vehicle.*truck*", "vehicle.*"),
    RoadObjectType.TRAILER: ("vehicle.carlamotors.firetruck", "vehicle.*truck*", "vehicle.*"),
    RoadObjectType.VAN: ("vehicle.mercedes.sprinter", "vehicle.*van*", "vehicle.*"),
    RoadObjectType.MOTORCYCLE: ("vehicle.vespa.zx125", "vehicle.*motorcycle*", "vehicle.*"),
    RoadObjectType.BICYCLE: ("vehicle.bh.crossbike", "vehicle.*bike*", "vehicle.*"),
    RoadObjectType.TRAIN: ("vehicle.*",),
    RoadObjectType.TRAM: ("vehicle.*",),
    RoadObjectType.WHEEL_CHAIR: ("walker.pedestrian.*", "walker.*"),
    RoadObjectType.ANIMAL: ("walker.pedestrian.*", "walker.*"),
    RoadObjectType.CAR: ("vehicle.*",),
    RoadObjectType.UNKNOWN: ("vehicle.*",),
}


class PclaAV:
    """PISA lifecycle adapter for PCLA agents running in a shadow CARLA world."""

    def __init__(self) -> None:
        self._lock = RLock()
        self._carla = None
        self._pcla_module = None
        self._data_provider = None
        self._client = None
        self._world = None
        self._map = None
        self._server_process = None
        self._server_version = None
        self._server_owned = False
        self._pcla = None
        self._vehicle = None
        self._other_actors_by_key: dict[Any, Any] = {}
        self._other_actor_types_by_key: dict[Any, RoadObjectType] = {}
        self._stateless_other_actors: list[Any] = []
        self._using_tracking_ids = False
        self._agent_shapes_by_tracking_id: dict[int, Any] = {}
        self._ego_shape = None
        self._blueprint_dimensions: dict[str, tuple[float, float, float] | None] = {}
        self._geometry_warnings: set[tuple[Any, ...]] = set()
        self._spawned_actor_ids: set[int] = set()
        self._loaded_map_name = None
        self._loaded_opendrive_path = None
        self._finalized = True
        self._initialized = False
        self._quit_flag = False
        self._quit_msg = ""
        self._last_error = ""
        self._last_timestamp_ns = 0
        self._expected_timestamp_ns = 0
        self._dt_ns = 0
        self._step_count = 0
        self._route_start_location = None
        self._route_goal_location = None
        self.config: dict[str, Any] = {}
        self._output_base = Path()
        self._output_dir = Path()
        self._pcla_runtime_dir = None

    def init(self, request: InitRequest) -> InitResponse:
        with self._lock:
            if not self._finalized:
                self._finalize()
            self._output_base = Path(request.output_dir)
            self._output_dir = self._output_base
            self.config = self._normalize_config(request.config or {})
            self._fixed_delta_seconds = self._positive_float("dt", request.dt)
            self._dt_ns = round(self._fixed_delta_seconds * 1_000_000_000)
            if self._dt_ns <= 0:
                raise InvalidAvRequest("Init dt must resolve to at least one nanosecond")
            self._parse_config()
            self._validate_agent_name()
            validate_image_profile(self._agent_name, self._pretrained_root)
            logger.info(
                "CARLA mode=%s endpoint=%s:%s",
                "owned" if self._launch_carla_server else "external",
                self._host,
                self._port,
            )
            if self._launch_carla_server and self._server_process is None:
                self._launch_server()
            try:
                connected = self._ensure_connected()
            except (AvTimeout, AvUnavailable):
                self._terminate_server_process()
                raise
            if not connected:
                self._terminate_server_process()
                raise AvTimeout(self._connect_timeout_message(None))
            self._prepare_reused_server_state()
            self._quit_flag = False
            self._quit_msg = ""
            self._last_error = ""
            self._initialized = True
            return InitResponse(
                name=self._agent_name,
                metadata={"effective_config": self._effective_config()},
            )

    def reset(self, request: ResetRequest) -> ResetResponse:
        with self._lock:
            if not self._initialized:
                raise AvPreconditionFailed("PCLA wrapper must be initialized before reset")
            if not self._finalized:
                self._finalize()
            self._finalized = False
            self._quit_flag = False
            self._quit_msg = ""
            self._last_error = ""
            self._step_count = 0
            self._expected_timestamp_ns = 0
            self._last_timestamp_ns = 0
            self._agent_shapes_by_tracking_id.clear()
            self._ego_shape = None
            self._geometry_warnings.clear()
            try:
                self._pcla_runtime_dir = None
                self._output_dir = self._resolve_reset_output_dir(request.output_dir)
                self._pcla_runtime_dir = self._resolve_pcla_runtime_dir()
                scenario = request.scenario_pack
                observation = request.initial_observation
                self._validate_reset_request(scenario, observation)
                self._ensure_world(scenario.map_name)
                self._cleanup_wrapper_actors()
                self._blueprint_dimensions.clear()
                observation = self._prepare_observation(observation, 0)
                self._vehicle = self._spawn_ego(observation, scenario)
                self._apply_world_settings()
                self._set_data_provider()
                route_path = self._resolve_route_path(scenario, observation)
                self._pcla = self._build_pcla(route_path)
                self._reset_pcla_game_time()
                self._prepare_pcla_sensors()
                return ResetResponse(
                    ctrl_cmd=self.step(
                        StepRequest(observation=observation, timestamp_ns=0)
                    ).ctrl_cmd
                )
            except Exception:
                logger.exception("PCLA reset failed; cleaning partial state")
                self._finalize()
                raise

    def step(self, request: StepRequest) -> StepResponse:
        with self._lock:
            if self._pcla is None or self._vehicle is None:
                raise AvPreconditionFailed("PCLA scenario is not ready; call reset first")
            self._raise_if_owned_server_exited()
            timestamp_ns = self._validate_step_timestamp(request.timestamp_ns)
            observation = self._prepare_observation(request.observation, timestamp_ns)
            self._last_timestamp_ns = timestamp_ns
            try:
                snapshot = self._update_and_tick(observation)
                snapshot = self._normalized_snapshot(snapshot, timestamp_ns)
                if self._data_provider is not None:
                    self._data_provider.on_carla_tick()
                action = self._get_action(snapshot)
            except AvError:
                raise
            except Exception as exc:
                self._raise_if_owned_server_exited()
                self._set_fatal_error(f"PCLA step failed: {exc}")
                logger.exception("PCLA step failed")
                raise AvUnavailable(str(exc)) from exc

            if action is None:
                message = "PCLA returned no action for the current CARLA frame"
                self._set_fatal_error(message)
                if self._action_none_timeout > 0:
                    raise AvTimeout(message)
                raise AvPreconditionFailed(message)

            payload = self._normalize_control(action)

            if hasattr(self._pcla, "done") and self._pcla.done():
                self._quit_flag = True
                self._quit_msg = "PCLA agent reached the route endpoint."
            self._step_count += 1
            self._expected_timestamp_ns += self._dt_ns
            self._log_driving_state(observation.ego.kinematic, action)
            return StepResponse(
                ctrl_cmd=ControlCommand(
                    mode=ControlMode.THROTTLE_STEER_BREAK,
                    payload=payload,
                )
            )

    def stop(self) -> None:
        with self._lock:
            self._finalize()
            self._client = None
            self._server_version = None
            self._world = None
            self._map = None
            self._loaded_map_name = None
            self._loaded_opendrive_path = None
            self._pcla_module = None
            self._data_provider = None
            self._initialized = False
            self._quit_flag = True
            self._quit_msg = "PCLA service stopped."

    def should_quit(self) -> ShouldQuitResponse:
        process = self._server_process
        if self._server_owned and process is not None:
            return_code = process.poll()
            if return_code is not None:
                self._set_fatal_error(self._owned_server_exit_message(return_code))
        return ShouldQuitResponse(should_quit=self._quit_flag, msg=self._quit_msg)

    def _validate_step_timestamp(self, raw_timestamp_ns: Any) -> int:
        try:
            timestamp_ns = int(raw_timestamp_ns)
        except (TypeError, ValueError) as exc:
            raise InvalidAvRequest("Step timestamp_ns must be an integer") from exc
        expected = getattr(self, "_expected_timestamp_ns", timestamp_ns)
        if timestamp_ns != expected:
            raise InvalidAvRequest(f"Step timestamp_ns must be {expected}, got {timestamp_ns}")
        return timestamp_ns

    @staticmethod
    def _finite(name: str, value: Any) -> float:
        try:
            number = float(value)
        except (TypeError, ValueError) as exc:
            raise InvalidAvRequest(f"{name} must be numeric") from exc
        if not math.isfinite(number):
            raise InvalidAvRequest(f"{name} must be finite")
        return number

    def _validate_state(self, state: ObjectStateData, timestamp_ns: int, label: str) -> None:
        kin = state.kinematic
        state_time = int(getattr(kin, "time_ns", timestamp_ns))
        if state_time != timestamp_ns:
            raise InvalidAvRequest(
                f"{label}.kinematic.time_ns must equal Step timestamp_ns {timestamp_ns}"
            )
        for field in (
            "x",
            "y",
            "z",
            "yaw",
            "speed",
            "acceleration",
            "yaw_rate",
            "yaw_acceleration",
        ):
            self._finite(f"{label}.kinematic.{field}", getattr(kin, field, 0.0))

        shape = getattr(state, "shape", None)
        if shape is None:
            return
        if shape.type != ShapeType.BOUNDING_BOX:
            raise InvalidAvRequest(
                f"{label}.shape type {shape.type!r} is unsupported; PCLA supports BOUNDING_BOX"
            )
        dimensions = shape.dimensions
        for field in ("x", "y", "z"):
            value = self._finite(
                f"{label}.shape.dimensions.{field}", getattr(dimensions, field, None)
            )
            if value <= 0:
                raise InvalidAvRequest(f"{label}.shape.dimensions.{field} must be positive")
        center = shape.center
        for field in ("x", "y", "z", "roll", "pitch", "yaw"):
            self._finite(f"{label}.shape.center.{field}", getattr(center, field, None))

    @staticmethod
    def _shape_values(shape: Any) -> tuple[float, ...]:
        dimensions = shape.dimensions
        center = shape.center
        return (
            float(dimensions.x),
            float(dimensions.y),
            float(dimensions.z),
            float(center.x),
            float(center.y),
            float(center.z),
            float(center.roll),
            float(center.pitch),
            float(center.yaw),
        )

    @classmethod
    def _shapes_equivalent(cls, left: Any, right: Any) -> bool:
        return (
            left.type == right.type
            and getattr(left, "reference_point", "") == getattr(right, "reference_point", "")
            and all(
                math.isclose(a, b, rel_tol=SHAPE_TOLERANCE, abs_tol=SHAPE_TOLERANCE)
                for a, b in zip(cls._shape_values(left), cls._shape_values(right))
            )
        )

    @staticmethod
    def _state_with_shape(state: ObjectStateData, shape: Any) -> ObjectStateData:
        return ObjectStateData(type=state.type, kinematic=state.kinematic, shape=shape)

    def _prepare_observation(
        self, observation: ObservationData, timestamp_ns: int
    ) -> ObservationData:
        ego = observation.ego
        ego_shape = getattr(ego, "shape", None)
        self._validate_state(ego, timestamp_ns, "observation.ego")
        if ego_shape is None and self._ego_shape is not None:
            ego = self._state_with_shape(ego, self._ego_shape)
        elif (
            ego_shape is not None
            and self._ego_shape is not None
            and not self._shapes_equivalent(ego_shape, self._ego_shape)
        ):
            raise InvalidAvRequest("observation.ego changed shape within the episode")
        elif ego_shape is not None:
            self._ego_shape = ego_shape
        self._validate_state(ego, timestamp_ns, "observation.ego")
        prepared_agents = []
        for index, agent in enumerate(observation.agents):
            state = agent.state
            tracking_id = agent.tracking_id
            shape = getattr(state, "shape", None)
            label = f"observation.agents[{index}].state"
            self._validate_state(state, timestamp_ns, label)
            if tracking_id is not None:
                cached = self._agent_shapes_by_tracking_id.get(tracking_id)
                if shape is None and cached is not None:
                    shape = cached
                    state = self._state_with_shape(state, shape)
                elif (
                    shape is not None
                    and cached is not None
                    and not self._shapes_equivalent(shape, cached)
                ):
                    raise InvalidAvRequest(
                        f"observation.agents[{index}] changed shape for tracking ID {tracking_id}"
                    )
                elif shape is not None:
                    self._agent_shapes_by_tracking_id[tracking_id] = shape
            self._validate_state(state, timestamp_ns, label)
            prepared_agents.append(
                ObservedAgentData(
                    state=state,
                    tracking_id=tracking_id,
                    entity_name=agent.entity_name,
                )
            )
        return ObservationData(ego=ego, agents=prepared_agents)

    def _normalized_snapshot(self, snapshot: Any, timestamp_ns: int) -> Any:
        elapsed_seconds = timestamp_ns / 1_000_000_000
        delta_seconds = 0.0 if self._step_count == 0 else self._fixed_delta_seconds
        timestamp = SimpleNamespace(
            frame=self._step_count + 1,
            elapsed_seconds=elapsed_seconds,
            delta_seconds=delta_seconds,
            platform_timestamp=elapsed_seconds,
        )
        return SimpleNamespace(
            frame=self._step_count + 1,
            timestamp=timestamp,
            native_snapshot=snapshot,
        )

    def _reset_pcla_game_time(self) -> None:
        game_time = getattr(self._pcla_module, "GameTime", None)
        if game_time is None:
            return
        game_time._current_game_time = 0.0
        game_time._carla_time = 0.0
        game_time._last_frame = 0
        game_time._platform_timestamp = 0
        game_time._init = False

    def _normalize_control(self, action: Any) -> dict[str, float]:
        values = {}
        for field in ("throttle", "brake", "steer"):
            if not hasattr(action, field):
                raise AvUnavailable(
                    f"PCLA action is missing canonical THROTTLE_STEER_BREAK field {field!r}"
                )
            try:
                value = float(getattr(action, field))
            except (TypeError, ValueError) as exc:
                raise AvUnavailable(f"PCLA action {field} must be numeric") from exc
            if not math.isfinite(value):
                raise AvUnavailable(f"PCLA action {field} must be finite")
            values[field] = value
        throttle = values["throttle"]
        brake = values["brake"]
        steer = values["steer"] / self._steer_sign
        for field, value, lower, upper in (
            ("throttle", throttle, 0.0, 1.0),
            ("brake", brake, 0.0, 1.0),
            ("steer", steer, -1.0, 1.0),
        ):
            if not lower <= value <= upper:
                raise AvUnavailable(
                    f"PCLA action {field}={value} is outside canonical range [{lower}, {upper}]"
                )
        if brake > 0.0:
            throttle = 0.0
        return {"throttle": throttle, "brake": brake, "steer": steer}

    def _normalize_config(self, raw: dict[str, Any]) -> dict[str, Any]:
        config = dict(raw)
        nested_pcla = config.pop("pcla", None)
        nested_carla = config.pop("carla", None)
        pcla_aliases = {
            "agent": "pcla_agent",
            "agent_name": "pcla_agent",
            "route_path": "route_xml_path",
        }
        carla_aliases = {
            "host": "carla_host",
            "port": "carla_port",
            "timeout": "carla_timeout",
        }
        for section_name, section, aliases in (
            ("pcla", nested_pcla, pcla_aliases),
            ("carla", nested_carla, carla_aliases),
        ):
            if section is None:
                continue
            if not isinstance(section, dict):
                raise InvalidAvRequest(f"{section_name} config must be a mapping")
            for old_key, value in section.items():
                key = aliases.get(old_key, old_key)
                if key in config and config[key] != value:
                    raise InvalidAvRequest(f"Conflicting flat and nested config values for {key!r}")
                config[key] = value
        return config

    def _resolve_reset_output_dir(self, requested: Any) -> Path:
        requested_path = Path(requested)
        if requested_path.is_absolute():
            output_dir = requested_path
        else:
            base = self._output_base.resolve()
            output_dir = (base / requested_path).resolve()
            try:
                output_dir.relative_to(base)
            except ValueError as exc:
                raise InvalidAvRequest(
                    f"Reset output_dir escapes Init output base: {requested_path}"
                ) from exc
        try:
            output_dir.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            raise InvalidAvRequest(
                f"Failed to create reset output directory: {output_dir}"
            ) from exc
        return output_dir

    def _resolve_pcla_runtime_dir(self) -> Path:
        configured = self.config.get("pcla_runtime_dir")
        if configured in (None, ""):
            runtime_dir = self._output_dir / "pcla_runtime"
        else:
            try:
                configured_path = Path(configured)
            except TypeError as exc:
                raise InvalidAvRequest(
                    f"pcla_runtime_dir must be a path, got {configured!r}"
                ) from exc
            if configured_path.is_absolute():
                runtime_dir = configured_path
            else:
                output_dir = self._output_dir.resolve()
                runtime_dir = (output_dir / configured_path).resolve()
                try:
                    runtime_dir.relative_to(output_dir)
                except ValueError as exc:
                    raise InvalidAvRequest(
                        f"pcla_runtime_dir escapes Reset output directory: {configured_path}"
                    ) from exc
        try:
            runtime_dir.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            raise InvalidAvRequest(
                f"Failed to create PCLA runtime directory: {runtime_dir}"
            ) from exc
        return runtime_dir.resolve()

    @contextlib.contextmanager
    def _in_pcla_runtime_dir(self):
        runtime_dir = self._pcla_runtime_dir
        if runtime_dir is None:
            yield
            return
        with PCLA_CWD_LOCK:
            previous_dir = Path.cwd()
            try:
                os.chdir(runtime_dir)
            except OSError as exc:
                raise AvUnavailable(
                    f"Failed to enter PCLA runtime directory: {runtime_dir}"
                ) from exc
            try:
                yield
            finally:
                os.chdir(previous_dir)

    def _parse_config(self) -> None:
        default_root = Path(__file__).resolve().parents[1] / "PCLA"
        self._pcla_root = self._resolve_pcla_root(Path(self.config.get("pcla_root", default_root)))
        self._pretrained_root = Path(
            os.environ.get(
                "PCLA_PRETRAINED_ROOT",
                self.config.get("pcla_pretrained_root", "/mnt/weights"),
            )
        ).resolve()
        self._agent_name = str(
            os.environ.get("PCLA_AGENT", self.config.get("pcla_agent", "carl_plant_3"))
        )
        route_override = os.environ.get("PCLA_ROUTE")
        self._route_path_cfg = route_override or self.config.get("route_xml_path")
        self._route_wp_distance = self._config_float("route_waypoint_distance", 2.0)
        self._route_draw = self._config_bool("route_draw", False)
        self._launch_carla_server = self._config_bool("launch_carla_server", True)
        self._connect_timeout = self._config_float(
            "carla_connect_timeout_seconds",
            self._config_float("max_retry_times", 15.0) * 2.0,
        )
        self._retry_interval = self._config_float("retry_interval_seconds", 2.0)
        if self._connect_timeout <= 0:
            raise InvalidAvRequest("carla_connect_timeout_seconds must be positive")
        if self._retry_interval <= 0:
            raise InvalidAvRequest("retry_interval_seconds must be positive")
        self._host = os.environ.get("CARLA_HOST", str(self.config.get("carla_host", "localhost")))
        try:
            self._port = int(os.environ.get("CARLA_PORT", self.config.get("carla_port", 2000)))
            self._traffic_manager_port = int(os.environ.get("CARLA_TM_PORT", 8000))
        except (TypeError, ValueError) as exc:
            raise InvalidAvRequest("CARLA_PORT and CARLA_TM_PORT must be integers") from exc
        self._carla_timeout = self._env_float(
            "CARLA_TIMEOUT", self._config_float("carla_timeout", 10.0)
        )
        self._carla_home = Path(
            os.environ.get(
                "CARLA_HOME",
                self.config.get("carla_home", self._output_dir / "carla_home"),
            )
        ).resolve()
        self._sync = self._config_bool("sync", True)
        self._no_rendering = self._config_bool("no_rendering", True)
        self._xodr_root = Path(self.config.get("xodr_root", "/mnt/map/xodr"))
        self._reuse_generated_world = self._config_bool("reuse_generated_world", True)
        self._manage_traffic_manager_sync = self._config_bool("manage_traffic_manager_sync", False)
        self._ego_role_name = str(self.config.get("ego_role_name", "hero"))
        self._ego_bp_id = str(self.config.get("ego_bp_id", "vehicle.tesla.model3"))
        self._spawn_z_offset = self._config_float("spawn_z_offset", 3.0)
        self._coordinate_y_sign = self._config_sign("coordinate_y_sign", -1.0)
        self._yaw_sign = self._config_sign("yaw_sign", -1.0)
        if self._yaw_sign != self._coordinate_y_sign:
            raise InvalidAvRequest(
                "yaw_sign must equal coordinate_y_sign for a consistent handedness conversion"
            )
        self._steer_sign = self._config_sign("steer_sign", -1.0)
        self._yaw_offset_deg = self._config_float("yaw_offset_deg", 0.0)
        self._action_none_timeout = self._config_float("action_none_timeout_seconds", 0.0)
        if self._action_none_timeout < 0:
            raise InvalidAvRequest("action_none_timeout_seconds must be non-negative")
        self._sensor_warmup_ticks = self._config_int("sensor_warmup_ticks", 1)
        if self._sensor_warmup_ticks < 0:
            raise InvalidAvRequest("sensor_warmup_ticks must be non-negative")
        self._debug_log_interval_steps = self._config_int("debug_log_interval_steps", 20)
        if self._debug_log_interval_steps < 0:
            raise InvalidAvRequest("debug_log_interval_steps must be non-negative")

    def _effective_config(self) -> dict[str, Any]:
        return {
            "pcla_agent": self._agent_name,
            "route_waypoint_distance": self._route_wp_distance,
            "route_draw": self._route_draw,
            "launch_carla_server": self._launch_carla_server,
            "reuse_generated_world": self._reuse_generated_world,
            "manage_traffic_manager_sync": self._manage_traffic_manager_sync,
            "ego_role_name": self._ego_role_name,
            "ego_bp_id": self._ego_bp_id,
            "spawn_z_offset": self._spawn_z_offset,
            "coordinate_y_sign": self._coordinate_y_sign,
            "yaw_sign": self._yaw_sign,
            "steer_sign": self._steer_sign,
            "yaw_offset_deg": self._yaw_offset_deg,
            "action_none_timeout_seconds": self._action_none_timeout,
            "sensor_warmup_ticks": self._sensor_warmup_ticks,
            "debug_log_interval_steps": self._debug_log_interval_steps,
        }

    @staticmethod
    def _resolve_pcla_root(
        configured_root: Path,
        legacy_root: Path = Path("/app/PCLA-wrapper/PCLA"),
        image_root: Path = Path("/app/PCLA"),
    ) -> Path:
        configured_root = configured_root.resolve()
        legacy_root = legacy_root.resolve()
        image_root = image_root.resolve()
        if configured_root == legacy_root and not configured_root.exists() and image_root.is_dir():
            logger.warning(
                "Configured pcla_root %s uses the retired image path; using %s",
                configured_root,
                image_root,
            )
            return image_root
        return configured_root

    def _config_float(self, name: str, default: float) -> float:
        raw = self.config.get(name, default)
        try:
            return float(raw)
        except (TypeError, ValueError) as exc:
            raise InvalidAvRequest(f"{name} must be a float, got {raw!r}") from exc

    def _config_bool(self, name: str, default: bool) -> bool:
        raw = self.config.get(name, default)
        if isinstance(raw, bool):
            return raw
        if isinstance(raw, str):
            normalized = raw.strip().lower()
            if normalized in {"true", "1", "yes", "on"}:
                return True
            if normalized in {"false", "0", "no", "off"}:
                return False
        if isinstance(raw, int) and raw in (0, 1):
            return bool(raw)
        raise InvalidAvRequest(f"{name} must be a boolean, got {raw!r}")

    def _config_int(self, name: str, default: int) -> int:
        raw = self.config.get(name, default)
        try:
            value = int(raw)
        except (TypeError, ValueError) as exc:
            raise InvalidAvRequest(f"{name} must be an integer, got {raw!r}") from exc
        if isinstance(raw, float) and not raw.is_integer():
            raise InvalidAvRequest(f"{name} must be an integer, got {raw!r}")
        return value

    @staticmethod
    def _positive_float(name: str, raw: Any) -> float:
        try:
            value = float(raw)
        except (TypeError, ValueError) as exc:
            raise InvalidAvRequest(f"{name} must be a float, got {raw!r}") from exc
        if value <= 0:
            raise InvalidAvRequest(f"{name} must be positive")
        return value

    def _config_sign(self, name: str, default: float) -> float:
        value = self._config_float(name, default)
        if abs(value) < 1e-6:
            raise InvalidAvRequest(f"{name} must be non-zero")
        return 1.0 if value > 0 else -1.0

    @staticmethod
    def _env_float(name: str, default: float) -> float:
        try:
            return float(os.environ.get(name, default))
        except (TypeError, ValueError) as exc:
            raise InvalidAvRequest(f"{name} must be a float") from exc

    def _validate_agent_name(self) -> None:
        agents_path = self._pcla_root / "agents.json"
        if not agents_path.is_file():
            raise InvalidAvRequest(f"PCLA agents registry not found: {agents_path}")
        try:
            agents = json.loads(agents_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise InvalidAvRequest(f"Failed to read PCLA agents registry: {agents_path}") from exc
        parts = self._agent_name.split("_")
        if len(parts) not in (2, 3) or not all(parts[:2]):
            raise InvalidAvRequest(
                "PCLA agent must use <family>_<variant>[_seed], for example carl_plant_3"
            )
        family, variant = parts[:2]
        if len(parts) == 3 and (not parts[2] or not parts[2].isdigit()):
            raise InvalidAvRequest("PCLA agent seed suffix must be an integer")
        if family not in agents or variant not in agents[family]:
            formats = [
                f"{family_name}_{variant_name}[_seed]"
                for family_name, variants in agents.items()
                for variant_name in variants
            ]
            raise InvalidAvRequest(
                f"Unknown PCLA agent {self._agent_name!r}. Accepted formats: "
                + ", ".join(sorted(formats))
            )

    def _ensure_carla_imports(self) -> None:
        if self._carla is not None:
            return
        carla_root = os.environ.get("CARLA_ROOT") or self.config.get("carla_root")
        carla_api = self.config.get("carla_egg")
        entries: list[Path] = []
        if carla_root:
            root = Path(carla_root)
            entries.extend((root / "PythonAPI", root / "PythonAPI" / "carla"))
            if not carla_api:
                dist = root / "PythonAPI" / "carla" / "dist"
                matches = sorted((*dist.glob("*.whl"), *dist.glob("*.egg")))
                carla_api = matches[0] if matches else None
        if carla_api:
            entries.append(Path(carla_api))
        for entry in entries:
            if str(entry) not in sys.path:
                sys.path.insert(0, str(entry))
        try:
            import carla
        except Exception as exc:
            raise AvUnavailable("CARLA Python API is not available") from exc
        self._carla = carla

    def _ensure_pcla_imports(self) -> None:
        if self._pcla_module is not None:
            return
        if not self._pcla_root.is_dir():
            raise AvUnavailable(f"PCLA root not found: {self._pcla_root}")
        if str(self._pcla_root) not in sys.path:
            sys.path.insert(0, str(self._pcla_root))
        try:
            from leaderboard_codes.carla_data_provider import CarlaDataProvider

            import PCLA
        except Exception as exc:
            raise AvUnavailable(f"Failed to import PCLA from {self._pcla_root}") from exc
        self._pcla_module = PCLA
        self._data_provider = CarlaDataProvider

    def _launch_server(self) -> None:
        log_dir = self._output_dir / "carla_server"
        cache_dir = self._carla_home / "carlaCache"
        xdg_cache_dir = self._carla_home / ".cache"
        try:
            log_dir.mkdir(parents=True, exist_ok=True)
            cache_dir.mkdir(parents=True, exist_ok=True)
            xdg_cache_dir.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            raise AvUnavailable(
                f"Failed to create CARLA log/cache directories under "
                f"{log_dir} and {self._carla_home}"
            ) from exc
        command = str(self.config.get("carla_server_script", "/app/carla_server.sh"))
        server_env = os.environ.copy()
        server_env["HOME"] = str(self._carla_home)
        server_env["XDG_CACHE_HOME"] = str(xdg_cache_dir)
        server_env["CARLA_HOME"] = str(self._carla_home)
        try:
            with contextlib.ExitStack() as stack:
                stdout = stack.enter_context((log_dir / "stdout.log").open("w", encoding="utf-8"))
                stderr = stack.enter_context((log_dir / "stderr.log").open("w", encoding="utf-8"))
                self._server_process = subprocess.Popen(
                    [command],
                    stdout=stdout,
                    stderr=stderr,
                    env=server_env,
                    start_new_session=True,
                )
        except OSError as exc:
            raise AvUnavailable(f"Failed to launch CARLA server with {command}") from exc
        self._server_owned = True
        logger.info("Launched owned CARLA server process %s", self._server_process.pid)

    def _connect_once(self) -> None:
        self._ensure_carla_imports()
        client = self._carla.Client(self._host, self._port)
        try:
            client.set_timeout(min(2.0, self._carla_timeout))
            version = client.get_server_version()
            world = client.get_world()
            if world is None:
                raise RuntimeError("CARLA returned no world")
        finally:
            client.set_timeout(self._carla_timeout)
        self._client = client
        self._server_version = version

    def _ensure_connected(self) -> bool:
        if self._server_version is not None and self._client is not None:
            return True
        self._raise_if_owned_server_exited(cause=None)
        deadline = time.monotonic() + getattr(self, "_connect_timeout", 30.0)
        last_error: Exception | None = None
        while True:
            self._raise_if_owned_server_exited(cause=last_error)
            try:
                self._connect_once()
                return True
            except (AvTimeout, AvUnavailable):
                raise
            except Exception as exc:
                last_error = exc
                self._raise_if_owned_server_exited(cause=exc)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    message = self._connect_timeout_message(exc)
                    logger.exception(message)
                    raise AvTimeout(message) from exc
                logger.warning("CARLA connection failed; retrying", exc_info=True)
                time.sleep(min(self._retry_interval, remaining))

    def _carla_endpoint(self) -> str:
        return f"{getattr(self, '_host', 'localhost')}:{getattr(self, '_port', 2000)}"

    def _connection_error_details(self, exc: Exception | None) -> str:
        if exc is None:
            return "none"
        return f"{type(exc).__name__}: {exc}"

    def _server_log_hint(self) -> str:
        log_dir = getattr(self, "_output_base", Path("/mnt/output")) / "carla_server"
        return f"inspect {log_dir / 'stderr.log'} and {log_dir / 'stdout.log'}"

    def _connect_timeout_message(self, last_error: Exception | None) -> str:
        process = self._server_process
        connect_timeout = getattr(self, "_connect_timeout", 30.0)
        if self._server_owned and process is not None:
            return_code = process.poll()
            if return_code is not None:
                return (
                    f"Timed out after {connect_timeout:g}s connecting to CARLA at "
                    f"{self._carla_endpoint()}; owned CARLA server exited with return code "
                    f"{return_code}. Last connection error: "
                    f"{self._connection_error_details(last_error)}. {self._server_log_hint()}"
                )
            pid = getattr(process, "pid", None)
            process_status = (
                f"owned CARLA server process is still running with pid {pid}"
                if pid is not None
                else "owned CARLA server process is still running"
            )
        elif getattr(self, "_launch_carla_server", False):
            process_status = "owned CARLA server process status is unavailable"
        else:
            process_status = "external CARLA server health cannot be inferred"
        return (
            f"Timed out after {connect_timeout:g}s connecting to CARLA at "
            f"{self._carla_endpoint()}; {process_status} but RPC did not respond. "
            f"Last connection error: {self._connection_error_details(last_error)}"
        )

    def _prepare_reused_server_state(self) -> None:
        if self._client is None:
            return
        previous_world = self._world
        try:
            self._world = self._client.get_world()
            self._map = self._world.get_map()
        except Exception as exc:
            raise AvUnavailable("Failed to inspect existing CARLA world") from exc
        if self._world is not previous_world:
            self._loaded_map_name = None
            self._loaded_opendrive_path = None
        clear_dynamic_actors(
            self._world,
            client=self._client,
            traffic_manager_port=self._traffic_manager_port,
            manage_traffic_manager=self._manage_traffic_manager_sync,
            log=logger,
        )

    def _validate_reset_request(
        self,
        scenario: ScenarioPackData | None,
        observation: ObservationData,
    ) -> None:
        if scenario is None:
            raise InvalidAvRequest("ScenarioPack is required")
        if not getattr(scenario, "map_name", ""):
            raise InvalidAvRequest("ScenarioPack map_name is required")
        self._extract_xyz(observation.ego.kinematic)
        if self._get_goal_position(scenario) is None:
            raise InvalidAvRequest("ScenarioPack ego goal position is required")

    def _ensure_world(self, map_name: str) -> None:
        if self._client is None:
            raise AvUnavailable("CARLA client is not available")
        path = (self._xodr_root / f"{map_name}.xodr").resolve()
        if (
            self._reuse_generated_world
            and self._world is not None
            and self._loaded_map_name == map_name
            and self._loaded_opendrive_path == path
        ):
            self._map = self._world.get_map()
            return
        if not path.is_file():
            raise InvalidAvRequest(f"OpenDRIVE map not found: {path}")
        try:
            opendrive = path.read_text(encoding="utf-8")
        except OSError as exc:
            raise InvalidAvRequest(f"Failed to read OpenDRIVE map: {path}") from exc
        self._client.set_timeout(300.0)
        try:
            try:
                world = self._client.generate_opendrive_world(
                    opendrive,
                    self._carla.OpendriveGenerationParameters(
                        vertex_distance=2.0,
                        max_road_length=3000.0,
                        wall_height=0.0,
                        additional_width=0.6,
                        smooth_junctions=True,
                        enable_mesh_visibility=True,
                    ),
                )
            except Exception as exc:
                raise AvPreconditionFailed(
                    f"Failed to generate CARLA world from OpenDRIVE map: {path}"
                ) from exc
        finally:
            self._client.set_timeout(self._carla_timeout)
        if world is None:
            raise AvUnavailable("CARLA returned no generated world")
        self._world = world
        try:
            self._map = world.get_map()
        except Exception as exc:
            raise AvPreconditionFailed("Generated CARLA world has no readable map") from exc
        self._loaded_map_name = map_name
        self._loaded_opendrive_path = path
        self._blueprint_dimensions.clear()

    def _apply_world_settings(self) -> None:
        settings = self._world.get_settings()
        settings.synchronous_mode = self._sync
        settings.no_rendering_mode = self._no_rendering
        settings.fixed_delta_seconds = self._fixed_delta_seconds
        self._world.apply_settings(settings)

    def _set_data_provider(self) -> None:
        self._ensure_pcla_imports()
        self._data_provider.set_client(self._client)
        self._data_provider.set_world(self._world)

    def _extract_xyz(self, pos: Any) -> tuple[float, float, float]:
        if pos is None:
            raise InvalidAvRequest("Position is required")
        world = getattr(pos, "world", None)
        source = world if world is not None else pos
        missing = [name for name in ("x", "y", "z") if not hasattr(source, name)]
        if missing:
            raise InvalidAvRequest(f"Position is missing coordinate field(s): {', '.join(missing)}")
        try:
            return float(source.x), float(source.y), float(source.z)
        except (TypeError, ValueError) as exc:
            raise InvalidAvRequest("Position coordinates must be numeric") from exc

    def _extract_yaw(self, pos: Any) -> float:
        source = getattr(pos, "world", None) or pos
        raw = getattr(source, "h", getattr(pos, "yaw", getattr(pos, "h", 0.0)))
        try:
            return float(raw)
        except (TypeError, ValueError) as exc:
            raise InvalidAvRequest("Position yaw must be numeric radians") from exc

    def _to_carla_location(self, pos: Any):
        x, y, z = self._extract_xyz(pos)
        return self._carla.Location(x=x, y=y * self._coordinate_y_sign, z=z)

    def _to_carla_yaw(self, yaw_rad: float) -> float:
        return self._yaw_sign * math.degrees(float(yaw_rad)) + self._yaw_offset_deg

    @staticmethod
    def _format_xyz(position: Any) -> str:
        return f"({float(position.x):.3f}, {float(position.y):.3f}, {float(position.z):.3f})"

    def _find_blueprint(self, library: Any, candidates: tuple[str, ...]):
        for pattern in candidates:
            try:
                if "*" not in pattern:
                    return library.find(pattern)
                matches = library.filter(pattern)
            except Exception:
                continue
            if matches:
                return matches[0]
        return None

    def _candidate_blueprints(self, obj_type: RoadObjectType) -> list[Any]:
        library = self._world.get_blueprint_library()
        patterns = BLUEPRINT_CANDIDATES.get(obj_type, BLUEPRINT_CANDIDATES[RoadObjectType.UNKNOWN])
        by_id = {}
        for pattern in patterns:
            try:
                matches = library.filter(pattern) if "*" in pattern else [library.find(pattern)]
            except Exception:
                continue
            for blueprint in matches:
                by_id[str(blueprint.id)] = blueprint
        return [by_id[key] for key in sorted(by_id)]

    def _measure_blueprint(self, blueprint: Any) -> tuple[float, float, float] | None:
        blueprint_id = str(blueprint.id)
        if blueprint_id in self._blueprint_dimensions:
            return self._blueprint_dimensions[blueprint_id]
        probe_transform = self._carla.Transform(
            self._carla.Location(x=0.0, y=0.0, z=10_000.0),
            self._carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0),
        )
        actor = self._world.try_spawn_actor(blueprint, probe_transform)
        dimensions = None
        if actor is not None:
            try:
                extent = actor.bounding_box.extent
                measured = (float(extent.x) * 2, float(extent.y) * 2, float(extent.z) * 2)
                if all(math.isfinite(value) and value > 0 for value in measured):
                    dimensions = measured
            except Exception:
                logger.debug("Could not measure blueprint %s", blueprint_id, exc_info=True)
            finally:
                destroy_actor(actor, log=logger, label="geometry probe actor")
        self._blueprint_dimensions[blueprint_id] = dimensions
        return dimensions

    @staticmethod
    def _dimension_score(
        requested: tuple[float, float, float], candidate: tuple[float, float, float]
    ) -> float:
        return math.sqrt(
            sum(((actual - target) / target) ** 2 for target, actual in zip(requested, candidate))
        )

    def _pick_blueprint_for_state(
        self, state: ObjectStateData, *, preferred_id: str | None = None
    ) -> Any:
        shape = getattr(state, "shape", None)
        library = self._world.get_blueprint_library()
        if shape is None:
            patterns = ((preferred_id,) if preferred_id else ()) + BLUEPRINT_CANDIDATES.get(
                state.type, BLUEPRINT_CANDIDATES[RoadObjectType.UNKNOWN]
            )
            return self._find_blueprint(library, patterns)

        requested = tuple(float(getattr(shape.dimensions, axis)) for axis in ("x", "y", "z"))
        candidates = self._candidate_blueprints(state.type)
        if preferred_id:
            try:
                preferred = library.find(preferred_id)
            except Exception:
                preferred = None
            if preferred is not None and all(item.id != preferred.id for item in candidates):
                candidates.append(preferred)
        ranked = []
        for blueprint in candidates:
            dimensions = self._measure_blueprint(blueprint)
            if dimensions is not None:
                ranked.append(
                    (
                        self._dimension_score(requested, dimensions),
                        str(blueprint.id),
                        blueprint,
                        dimensions,
                    )
                )
        if not ranked:
            fallback = self._find_blueprint(
                library,
                BLUEPRINT_CANDIDATES.get(state.type, BLUEPRINT_CANDIDATES[RoadObjectType.UNKNOWN]),
            )
            if fallback is None:
                return None
            warning_key = (state.type, requested, str(fallback.id), None)
            if warning_key not in self._geometry_warnings:
                self._geometry_warnings.add(warning_key)
                logger.warning(
                    "Unable to measure CARLA geometry; using blueprint=%s requested_dimensions=%s",
                    fallback.id,
                    requested,
                )
            return fallback
        score, blueprint_id, blueprint, dimensions = min(
            ranked, key=lambda item: (item[0], item[1])
        )
        warning_key = (state.type, requested, blueprint_id, dimensions)
        if warning_key not in self._geometry_warnings:
            self._geometry_warnings.add(warning_key)
            logger.warning(
                "PISA shape uses nearest CARLA geometry blueprint=%s requested_dimensions=%s "
                "native_dimensions=%s relative_error=%.6f",
                blueprint_id,
                requested,
                dimensions,
                score,
            )
        return blueprint

    def _spawn_ego(
        self,
        observation: ObservationData,
        scenario: ScenarioPackData,
    ):
        blueprint = self._pick_blueprint_for_state(observation.ego, preferred_id=self._ego_bp_id)
        if blueprint is None:
            raise AvPreconditionFailed("No CARLA vehicle blueprint is available for ego")
        if blueprint.has_attribute("role_name"):
            blueprint.set_attribute("role_name", self._ego_role_name)
        pos = self._get_spawn_position(observation, scenario)
        location = self._to_carla_location(pos)
        location.z += self._spawn_z_offset
        transform = self._carla.Transform(
            location,
            self._carla.Rotation(
                pitch=0.0, yaw=self._to_carla_yaw(self._extract_yaw(pos)), roll=0.0
            ),
        )
        ego = self._spawn_actor_allowing_observation_overlap(blueprint, transform)
        if ego is None:
            raise AvPreconditionFailed("Failed to spawn ego vehicle")
        self._spawned_actor_ids.add(ego.id)
        self._apply_state(ego, observation.ego)
        return ego

    def _get_spawn_position(
        self,
        observation: ObservationData,
        scenario: ScenarioPackData,
    ):
        return observation.ego.kinematic

    @staticmethod
    def _get_goal_position(scenario: ScenarioPackData):
        ego = getattr(scenario, "ego", None)
        goal = getattr(ego, "goal_config", None)
        return getattr(goal, "position", None)

    def _resolve_route_path(
        self,
        scenario: ScenarioPackData,
        observation: ObservationData,
    ) -> Path:
        raw_start = self._get_spawn_position(observation, scenario)
        raw_goal = self._get_goal_position(scenario)
        start_xyz = self._extract_xyz(raw_start)
        goal_xyz = self._extract_xyz(raw_goal)
        start = self._to_carla_location(raw_start)
        goal = self._to_carla_location(raw_goal)
        logger.info(
            "Reset route endpoints scenario=%r PISA start=(%.3f, %.3f, %.3f) "
            "goal=(%.3f, %.3f, %.3f) CARLA start=%s goal=%s",
            scenario.name,
            *start_xyz,
            *goal_xyz,
            self._format_xyz(start),
            self._format_xyz(goal),
        )

        if self._route_path_cfg:
            path = Path(self._route_path_cfg)
            if not path.is_absolute():
                path = (self._pcla_root / path).resolve()
            if not path.is_file() or not os.access(path, os.R_OK):
                raise InvalidAvRequest(f"Configured route XML is not readable: {path}")
            logger.info("Using configured route XML: %s", path)
            return path

        self._ensure_pcla_imports()
        route_dir = self._output_dir / "pcla_routes"
        route_dir.mkdir(parents=True, exist_ok=True)
        safe_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", scenario.name or "scenario")
        safe_name = safe_name.strip("._") or "scenario"
        route_path = route_dir / f"{safe_name}.route.xml"

        start_wp = self._map.get_waypoint(start, project_to_road=True)
        goal_wp = self._map.get_waypoint(goal, project_to_road=True)
        if start_wp is None or goal_wp is None:
            raise AvPreconditionFailed(
                "Failed to project route endpoints onto the CARLA map: "
                f"start={self._format_xyz(start)}, goal={self._format_xyz(goal)}"
            )
        logger.info(
            "Projected route endpoints scenario=%r start=%s goal=%s",
            scenario.name,
            self._format_xyz(start_wp.transform.location),
            self._format_xyz(goal_wp.transform.location),
        )
        self._route_start_location = start_wp.transform.location
        self._route_goal_location = goal_wp.transform.location
        try:
            waypoints = self._pcla_module.location_to_waypoint(
                self._client,
                start_wp.transform.location,
                goal_wp.transform.location,
                distance=self._route_wp_distance,
                draw=self._route_draw,
            )
        except Exception as exc:
            raise AvPreconditionFailed(
                "PCLA route planner failed: "
                f"start={self._format_xyz(start_wp.transform.location)}, "
                f"goal={self._format_xyz(goal_wp.transform.location)}"
            ) from exc
        if len(waypoints) < 2:
            raise AvPreconditionFailed("PCLA route planner returned fewer than two waypoints")
        endpoints = [waypoints[0], waypoints[-1]]
        temp_path = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w",
                dir=route_dir,
                prefix=f".{safe_name}.",
                suffix=".xml",
                delete=False,
            ) as handle:
                temp_path = Path(handle.name)
            self._pcla_module.route_maker(endpoints, savePath=str(temp_path))
            if not temp_path.is_file() or temp_path.stat().st_size == 0:
                raise AvPreconditionFailed("PCLA route writer produced an empty route")
            os.replace(temp_path, route_path)
        except AvError:
            raise
        except Exception as exc:
            raise AvPreconditionFailed("Failed to write PCLA route XML") from exc
        finally:
            if temp_path is not None:
                with contextlib.suppress(FileNotFoundError):
                    temp_path.unlink()
        return route_path

    @staticmethod
    def _normalize_degrees(angle: float) -> float:
        return (angle + 180.0) % 360.0 - 180.0

    def _log_driving_state(self, kinematic: Any, action: Any) -> None:
        interval = getattr(self, "_debug_log_interval_steps", 20)
        if interval == 0 or (self._step_count > 3 and self._step_count % interval != 0):
            return
        try:
            transform = self._vehicle.get_transform()
            velocity = self._vehicle.get_velocity()
        except Exception:
            logger.debug(
                "Unable to collect shadow CARLA actor state for diagnostics", exc_info=True
            )
            return
        actor_speed = math.sqrt(
            float(velocity.x) ** 2 + float(velocity.y) ** 2 + float(velocity.z) ** 2
        )
        route_heading = None
        heading_error = None
        if self._route_start_location is not None and self._route_goal_location is not None:
            route_heading = math.degrees(
                math.atan2(
                    self._route_goal_location.y - self._route_start_location.y,
                    self._route_goal_location.x - self._route_start_location.x,
                )
            )
            heading_error = self._normalize_degrees(route_heading - transform.rotation.yaw)
        logger.debug(
            "Driving state step=%d timestamp_ns=%d "
            "PISA pose=(%.3f, %.3f, %.3f) yaw_rad=%.6f speed=%.3f "
            "CARLA pose=%s yaw_deg=%.3f speed=%.3f "
            "route_heading_deg=%s heading_error_deg=%s "
            "control_raw=(throttle=%.3f brake=%.3f steer=%.3f) output_steer=%.3f",
            self._step_count,
            self._last_timestamp_ns,
            *self._extract_xyz(kinematic),
            float(kinematic.yaw),
            float(kinematic.speed),
            self._format_xyz(transform.location),
            float(transform.rotation.yaw),
            actor_speed,
            "n/a" if route_heading is None else f"{route_heading:.3f}",
            "n/a" if heading_error is None else f"{heading_error:.3f}",
            float(action.throttle),
            float(action.brake),
            float(action.steer),
            float(action.steer) / self._steer_sign,
        )

    def _build_pcla(self, route_path: Path):
        self._ensure_pcla_imports()
        try:
            with self._in_pcla_runtime_dir():
                return self._pcla_module.PCLA(
                    self._agent_name,
                    self._vehicle,
                    str(route_path),
                    self._client,
                    destroy_vehicle=False,
                )
        except TimeoutError as exc:
            raise AvTimeout(f"Timed out loading PCLA agent {self._agent_name!r}") from exc
        except (FileNotFoundError, ImportError, ModuleNotFoundError) as exc:
            raise AvUnavailable(
                f"PCLA agent {self._agent_name!r} dependencies or weights are unavailable: {exc}"
            ) from exc

    def _prepare_pcla_sensors(self) -> None:
        sensors = getattr(self._pcla, "_sensors", ())
        has_camera = any(
            str(getattr(sensor, "type_id", "")).startswith("sensor.camera.") for sensor in sensors
        )
        if has_camera:
            settings = self._world.get_settings()
            if settings.no_rendering_mode:
                logger.warning("PCLA agent uses camera sensors; disabling CARLA no-rendering mode")
                settings.no_rendering_mode = False
                self._world.apply_settings(settings)

        for _ in range(self._sensor_warmup_ticks):
            self._raise_if_owned_server_exited()
            if self._sync:
                self._world.tick()
            else:
                self._world.wait_for_tick()
            self._raise_if_owned_server_exited()

    def _get_action(self, snapshot: Any):
        deadline = time.monotonic() + self._action_none_timeout
        while True:
            with self._in_pcla_runtime_dir():
                try:
                    action = self._pcla.get_action(snapshot=snapshot)
                except TypeError as exc:
                    if "snapshot" not in str(exc):
                        raise
                    action = self._pcla.get_action()
            if action is not None or self._action_none_timeout <= 0:
                return action
            if time.monotonic() >= deadline:
                return None
            time.sleep(min(0.01, max(0.0, deadline - time.monotonic())))

    @staticmethod
    def _role_name_for_tracking_id(tracking_id: int) -> str:
        return f"agent_{tracking_id}"[:255]

    def _spawn_actor_allowing_observation_overlap(self, blueprint: Any, transform: Any):
        actor = self._world.try_spawn_actor(blueprint, transform)
        if actor is not None:
            return actor
        base = transform.location
        for offset in (max(self._spawn_z_offset, 5.0), 10.0, 20.0, 50.0):
            elevated = self._carla.Transform(
                self._carla.Location(base.x, base.y, base.z + offset),
                transform.rotation,
            )
            actor = self._world.try_spawn_actor(blueprint, elevated)
            if actor is not None:
                return actor
        return None

    def _make_observation_actor_kinematic(self, actor: Any) -> None:
        with contextlib.suppress(Exception):
            actor.set_simulate_physics(False)
        with contextlib.suppress(Exception):
            actor.set_enable_gravity(False)

    @staticmethod
    def _rotation_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float):
        roll, pitch, yaw = map(math.radians, (roll_deg, pitch_deg, yaw_deg))
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        return (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        )

    @staticmethod
    def _matrix_multiply(left, right):
        return tuple(
            tuple(sum(left[row][k] * right[k][column] for k in range(3)) for column in range(3))
            for row in range(3)
        )

    @staticmethod
    def _matrix_vector(matrix, vector):
        return tuple(sum(matrix[row][k] * vector[k] for k in range(3)) for row in range(3))

    @staticmethod
    def _matrix_transpose(matrix):
        return tuple(tuple(matrix[column][row] for column in range(3)) for row in range(3))

    def _relative_rotation_to_carla(self, center: Any):
        pisa_rotation = self._rotation_matrix(
            math.degrees(float(center.roll)),
            math.degrees(float(center.pitch)),
            math.degrees(float(center.yaw)),
        )
        basis = (
            (1.0, 0.0, 0.0),
            (0.0, self._coordinate_y_sign, 0.0),
            (0.0, 0.0, 1.0),
        )
        return self._matrix_multiply(self._matrix_multiply(basis, pisa_rotation), basis)

    @staticmethod
    def _matrix_to_rotation(matrix):
        pitch = math.asin(max(-1.0, min(1.0, -matrix[2][0])))
        if abs(math.cos(pitch)) > 1e-8:
            roll = math.atan2(matrix[2][1], matrix[2][2])
            yaw = math.atan2(matrix[1][0], matrix[0][0])
        else:
            roll = 0.0
            yaw = math.atan2(-matrix[0][1], matrix[1][1])
        return tuple(map(math.degrees, (roll, pitch, yaw)))

    def _object_transform(self, state: ObjectStateData, actor: Any = None, z_offset: float = 0.0):
        kin = state.kinematic
        kin_loc = self._to_carla_location(kin)
        kin_loc.z += z_offset
        kin_yaw = self._to_carla_yaw(float(kin.yaw))
        fallback = self._carla.Transform(
            kin_loc, self._carla.Rotation(pitch=0.0, yaw=kin_yaw, roll=0.0)
        )
        shape = getattr(state, "shape", None)
        bounding_box = getattr(actor, "bounding_box", None)
        if shape is None or shape.type != ShapeType.BOUNDING_BOX or bounding_box is None:
            return fallback

        center = shape.center
        kin_rotation = self._rotation_matrix(0.0, 0.0, kin_yaw)
        center_rotation = self._relative_rotation_to_carla(center)
        box_world_rotation = self._matrix_multiply(kin_rotation, center_rotation)
        center_offset = (
            float(center.x),
            float(center.y) * self._coordinate_y_sign,
            float(center.z),
        )
        rotated_center = self._matrix_vector(kin_rotation, center_offset)
        box_world_location = tuple(
            value + offset
            for value, offset in zip(
                (float(kin_loc.x), float(kin_loc.y), float(kin_loc.z)), rotated_center
            )
        )
        local_location = getattr(bounding_box, "location", None)
        local_rotation = getattr(bounding_box, "rotation", None)
        actor_box_rotation = self._rotation_matrix(
            float(getattr(local_rotation, "roll", 0.0)),
            float(getattr(local_rotation, "pitch", 0.0)),
            float(getattr(local_rotation, "yaw", 0.0)),
        )
        actor_rotation = self._matrix_multiply(
            box_world_rotation, self._matrix_transpose(actor_box_rotation)
        )
        local_offset = (
            float(getattr(local_location, "x", 0.0)),
            float(getattr(local_location, "y", 0.0)),
            float(getattr(local_location, "z", 0.0)),
        )
        actor_box_offset = self._matrix_vector(actor_rotation, local_offset)
        actor_location = self._carla.Location(
            *(box_world_location[i] - actor_box_offset[i] for i in range(3))
        )
        roll, pitch, yaw = self._matrix_to_rotation(actor_rotation)
        return self._carla.Transform(
            actor_location, self._carla.Rotation(pitch=pitch, yaw=yaw, roll=roll)
        )

    def _apply_state(self, actor: Any, state: ObjectStateData, *, kinematic: bool = False) -> None:
        if kinematic:
            self._make_observation_actor_kinematic(actor)
        actor.set_transform(self._object_transform(state, actor))
        kin = state.kinematic
        yaw = self._to_carla_yaw(float(kin.yaw))
        yaw_rad = math.radians(yaw)
        velocity = self._carla.Vector3D(
            float(kin.speed) * math.cos(yaw_rad),
            float(kin.speed) * math.sin(yaw_rad),
            0.0,
        )
        try:
            actor.set_target_velocity(velocity)
        except Exception:
            with contextlib.suppress(Exception):
                actor.set_velocity(velocity)
        angular = self._carla.Vector3D(0.0, 0.0, math.degrees(float(kin.yaw_rate)) * self._yaw_sign)
        try:
            actor.set_target_angular_velocity(angular)
        except Exception:
            with contextlib.suppress(Exception):
                actor.set_angular_velocity(angular)

    def _update_and_tick(self, observation: ObservationData):
        self._apply_state(self._vehicle, observation.ego)
        agents = list(observation.agents)
        tracking_ids = [agent.tracking_id for agent in agents]
        use_tracking_ids = bool(agents) and all(
            tracking_id is not None for tracking_id in tracking_ids
        )
        if use_tracking_ids and len(set(tracking_ids)) != len(tracking_ids):
            raise InvalidAvRequest("Observation contains duplicate agent tracking IDs")

        if not use_tracking_ids:
            self._destroy_other_actors()
            for observed_agent in agents:
                obj = observed_agent.state
                blueprint = self._pick_blueprint_for_state(obj)
                if blueprint is None:
                    raise AvPreconditionFailed(f"No CARLA blueprint for object type {obj.type}")
                if blueprint.has_attribute("role_name"):
                    blueprint.set_attribute("role_name", "agent")
                transform = self._object_transform(obj, z_offset=self._spawn_z_offset)
                actor = self._spawn_actor_allowing_observation_overlap(blueprint, transform)
                if actor is None:
                    raise AvPreconditionFailed("Failed to spawn stateless observation actor")
                self._spawned_actor_ids.add(actor.id)
                self._stateless_other_actors.append(actor)
                self._apply_state(actor, obj, kinematic=True)
        else:
            if not self._using_tracking_ids:
                self._destroy_other_actors()
            self._using_tracking_ids = True
            observed_keys = set(tracking_ids)
            for observed_agent in agents:
                key = observed_agent.tracking_id
                obj = observed_agent.state
                actor = self._other_actors_by_key.get(key)
                if (
                    actor is None
                    or not getattr(actor, "is_alive", True)
                    or self._other_actor_types_by_key.get(key) != obj.type
                ):
                    if actor is not None:
                        actor_id = getattr(actor, "id", None)
                        if destroy_actor(actor, log=logger):
                            self._spawned_actor_ids.discard(actor_id)
                    blueprint = self._pick_blueprint_for_state(obj)
                    if blueprint is None:
                        raise AvPreconditionFailed(f"No CARLA blueprint for object type {obj.type}")
                    if blueprint.has_attribute("role_name"):
                        blueprint.set_attribute("role_name", self._role_name_for_tracking_id(key))
                    transform = self._object_transform(obj, z_offset=self._spawn_z_offset)
                    actor = self._spawn_actor_allowing_observation_overlap(blueprint, transform)
                    if actor is None:
                        raise AvPreconditionFailed(f"Failed to spawn actor for tracking ID {key}")
                    self._spawned_actor_ids.add(actor.id)
                    self._other_actors_by_key[key] = actor
                    self._other_actor_types_by_key[key] = obj.type
                self._apply_state(actor, obj, kinematic=True)

            for key in set(self._other_actors_by_key) - observed_keys:
                actor = self._other_actors_by_key.pop(key)
                self._other_actor_types_by_key.pop(key, None)
                actor_id = getattr(actor, "id", None)
                if destroy_actor(actor, log=logger, label="stale actor"):
                    self._spawned_actor_ids.discard(actor_id)

        if self._sync:
            self._world.tick()
        else:
            self._world.wait_for_tick()
        return self._world.get_snapshot()

    def _destroy_other_actors(self) -> None:
        actors = [*self._other_actors_by_key.values(), *self._stateless_other_actors]
        for actor in actors:
            actor_id = getattr(actor, "id", None)
            if destroy_actor(actor, log=logger):
                self._spawned_actor_ids.discard(actor_id)
        self._other_actors_by_key.clear()
        self._other_actor_types_by_key.clear()
        self._stateless_other_actors.clear()
        self._using_tracking_ids = False

    def _cleanup_wrapper_actors(self) -> None:
        if self._world is None:
            self._vehicle = None
            self._other_actors_by_key.clear()
            self._other_actor_types_by_key.clear()
            self._stateless_other_actors.clear()
            self._using_tracking_ids = False
            self._agent_shapes_by_tracking_id.clear()
            self._ego_shape = None
            self._spawned_actor_ids.clear()
            return
        force_async_world_for_cleanup(
            self._world,
            client=self._client,
            traffic_manager_port=self._traffic_manager_port,
            manage_traffic_manager=self._manage_traffic_manager_sync,
            log=logger,
        )
        actors = [
            self._vehicle,
            *self._other_actors_by_key.values(),
            *self._stateless_other_actors,
        ]
        destroyed = set()
        for actor in actors:
            actor_id = getattr(actor, "id", None)
            if actor_id in destroyed:
                continue
            if destroy_actor(actor, log=logger):
                destroyed.add(actor_id)
        self._vehicle = None
        self._other_actors_by_key.clear()
        self._other_actor_types_by_key.clear()
        self._stateless_other_actors.clear()
        self._using_tracking_ids = False
        self._agent_shapes_by_tracking_id.clear()
        self._ego_shape = None
        self._spawned_actor_ids.clear()

    def _finalize(self) -> None:
        if self._pcla is not None:
            try:
                with self._in_pcla_runtime_dir():
                    self._pcla.cleanup()
            except Exception:
                logger.exception("Failed to cleanup PCLA")
            self._pcla = None
        self._cleanup_wrapper_actors()
        self._finalized = True
        self._quit_flag = True
        if not self._quit_msg:
            self._quit_msg = "PCLA scenario finalized."

    def _terminate_server_process(self) -> None:
        process = self._server_process
        if process is None:
            self._server_owned = False
            return
        try:
            if process.poll() is None:
                self._signal_server_process_group(process, signal.SIGTERM)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    self._signal_server_process_group(process, signal.SIGKILL)
                    process.wait(timeout=10)
        except Exception:
            logger.exception("Failed to terminate owned CARLA server")
        finally:
            self._server_process = None
            self._server_owned = False

    @staticmethod
    def _signal_server_process_group(process: Any, sig: signal.Signals) -> None:
        try:
            os.killpg(process.pid, sig)
        except (AttributeError, ProcessLookupError, OSError):
            if sig == signal.SIGTERM:
                process.terminate()
            else:
                process.kill()

    def _raise_if_owned_server_exited(self, *, cause: Exception | None = None) -> None:
        process = self._server_process
        if not self._server_owned or process is None:
            return
        return_code = process.poll()
        if return_code is None:
            return
        message = self._owned_server_exit_message(return_code)
        if cause is not None:
            message = f"{message}; last connection error: {self._connection_error_details(cause)}"
        self._set_fatal_error(message)
        if cause is None:
            raise AvUnavailable(message)
        raise AvUnavailable(message) from cause

    def _owned_server_exit_message(self, return_code: int) -> str:
        return (
            f"Owned CARLA server exited with return code {return_code} before responding at "
            f"{self._carla_endpoint()}; {self._server_log_hint()}"
        )

    def _set_fatal_error(self, message: str) -> None:
        self._last_error = message
        self._quit_flag = True
        self._quit_msg = message
