import ast
import importlib.util
import inspect
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest
from pisa_api.av import (
    AvPreconditionFailed,
    AvTimeout,
    AvUnavailable,
    ControlMode,
    InvalidAvRequest,
    RoadObjectType,
    ShouldQuitResponse,
    StepResponse,
)

from pcla_wrapper.lifecycle import clear_dynamic_actors
from pcla_wrapper.pcla_av import PclaAV


class FakeLocation:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class FakeRotation:
    def __init__(self, pitch=0.0, yaw=0.0, roll=0.0):
        self.pitch = pitch
        self.yaw = yaw
        self.roll = roll


class FakeTransform:
    def __init__(self, location, rotation=None):
        self.location = location
        self.rotation = rotation or FakeRotation()


class FakeVector:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class FakeBlueprint:
    def __init__(self, blueprint_id="vehicle.test"):
        self.id = blueprint_id
        self.attributes = {}

    def has_attribute(self, name):
        return name == "role_name"

    def set_attribute(self, name, value):
        self.attributes[name] = value


class FakeBlueprintLibrary:
    def __init__(self, exact=None, patterns=None):
        self.exact = dict(exact or {})
        self.patterns = dict(patterns or {})
        self.find_calls = []
        self.filter_calls = []

    def find(self, name):
        self.find_calls.append(name)
        if name not in self.exact:
            raise KeyError(name)
        return self.exact[name]

    def filter(self, pattern):
        self.filter_calls.append(pattern)
        return list(self.patterns.get(pattern, []))


class FakeActor:
    def __init__(self, actor_id, type_id="vehicle.test"):
        self.id = actor_id
        self.type_id = type_id
        self.is_alive = True
        self.destroy_calls = 0
        self.transforms = []
        self.physics_calls = []
        self.gravity_calls = []

    def destroy(self):
        self.destroy_calls += 1
        self.is_alive = False
        return True

    def set_transform(self, transform):
        self.transforms.append(transform)

    def set_target_velocity(self, velocity):
        self.velocity = velocity

    def set_target_angular_velocity(self, velocity):
        self.angular_velocity = velocity

    def set_simulate_physics(self, enabled):
        self.physics_calls.append(enabled)

    def set_enable_gravity(self, enabled):
        self.gravity_calls.append(enabled)


class FakeMap:
    name = "OpenDriveMap"

    def get_waypoint(self, location, project_to_road=True):
        return SimpleNamespace(transform=FakeTransform(location))


class FakeWorld:
    def __init__(self, blueprints=None, actors=None):
        self.blueprints = blueprints or FakeBlueprintLibrary()
        self.actors = list(actors or [])
        self.settings = SimpleNamespace(
            synchronous_mode=False,
            no_rendering_mode=False,
            fixed_delta_seconds=None,
        )
        self.tick_calls = 0
        self.events = []
        self.spawn_results = []
        self.snapshot = SimpleNamespace(timestamp=SimpleNamespace(frame=1))
        self.map = FakeMap()

    def get_settings(self):
        return self.settings

    def apply_settings(self, settings):
        self.settings = settings

    def get_blueprint_library(self):
        return self.blueprints

    def get_map(self):
        return self.map

    def get_actors(self):
        return list(self.actors)

    def get_actor(self, actor_id):
        for actor in self.actors:
            if actor.id == actor_id:
                return actor
        return None

    def try_spawn_actor(self, blueprint, transform):
        if self.spawn_results:
            result = self.spawn_results.pop(0)
            if result is None:
                return None
            actor = result
        else:
            actor = FakeActor(100 + len(self.actors))
        self.actors.append(actor)
        actor.spawn_transform = transform
        return actor

    def tick(self):
        self.tick_calls += 1
        self.events.append("tick")
        return self.tick_calls

    def wait_for_tick(self):
        return self.tick()

    def get_snapshot(self):
        self.events.append("snapshot")
        return self.snapshot


class FakeClient:
    def __init__(self, world=None):
        self.world = world or FakeWorld()
        self.timeouts = []
        self.generate_calls = 0
        self.generate_error = None

    def set_timeout(self, value):
        self.timeouts.append(value)

    def get_server_version(self):
        return "0.9.16"

    def get_world(self):
        return self.world

    def generate_opendrive_world(self, opendrive, parameters):
        self.generate_calls += 1
        if self.generate_error:
            raise self.generate_error
        return self.world


class FakeProcess:
    def __init__(self, return_code=None):
        self.return_code = return_code
        self.terminate_calls = 0
        self.kill_calls = 0
        self.wait_calls = 0

    def poll(self):
        return self.return_code

    def terminate(self):
        self.terminate_calls += 1

    def kill(self):
        self.kill_calls += 1

    def wait(self, timeout=None):
        self.wait_calls += 1
        self.return_code = 0
        return 0


def fake_carla():
    return SimpleNamespace(
        Location=FakeLocation,
        Rotation=FakeRotation,
        Transform=FakeTransform,
        Vector3D=FakeVector,
        OpendriveGenerationParameters=lambda **kwargs: SimpleNamespace(**kwargs),
    )


def kinematic(x=0.0, y=0.0, z=0.0, yaw=0.0, speed=0.0, yaw_rate=0.0):
    return SimpleNamespace(
        x=x,
        y=y,
        z=z,
        yaw=yaw,
        speed=speed,
        yaw_rate=yaw_rate,
    )


def object_state(x=0.0, object_type=RoadObjectType.CAR, **extra):
    values = {"type": object_type, "kinematic": kinematic(x=x)}
    values.update(extra)
    return SimpleNamespace(**values)


def configured_adapter(world=None):
    adapter = PclaAV()
    adapter._carla = fake_carla()
    adapter._world = world or FakeWorld(
        FakeBlueprintLibrary(patterns={"vehicle.*": [FakeBlueprint()]})
    )
    adapter._map = adapter._world.get_map()
    adapter._client = FakeClient(adapter._world)
    adapter._sync = True
    adapter._spawn_z_offset = 0.0
    adapter._coordinate_y_sign = 1.0
    adapter._yaw_sign = 1.0
    adapter._steer_sign = 1.0
    adapter._yaw_offset_deg = 0.0
    adapter._object_identity_mode = "index"
    adapter._traffic_manager_port = 8000
    adapter._manage_traffic_manager_sync = False
    adapter._action_none_timeout = 0.0
    adapter._sensor_warmup_ticks = 0
    adapter._pcla_runtime_dir = None
    adapter._vehicle = FakeActor(1)
    adapter._spawned_actor_ids = {1}
    return adapter


def write_agents(root):
    root.mkdir(parents=True)
    (root / "agents.json").write_text(
        '{"carl": {"plant": {"agent": "agent.py", "config": "weights"}}}',
        encoding="utf-8",
    )


def test_server_uses_generic_pisa_service():
    source = Path("pcla_wrapper/server.py").read_text(encoding="utf-8")
    assert 'serve_av_system(PclaAV(), name="PCLA")' in source
    assert "grpc.server" not in source
    assert "sbsvf_api" not in source


def test_owned_carla_launcher_preserves_rendering_and_drops_root():
    source = Path("carla_server.sh").read_text(encoding="utf-8")
    dockerfile = Path("Dockerfile").read_text(encoding="utf-8")
    assert "-RenderOffScreen" in source
    assert "CARLA_NULLRHI:-0" in source
    assert 'else\n    args+=("-quality-level=' in source
    assert 'cd "${CARLA_ROOT}"' in source
    assert "carla-rpc-timeout" in source
    assert "carla-tm-port" in source
    assert "CARLA_RUN_UID:-$(id -u carla)" in source
    assert "CARLA_RUN_GID:-$(id -g carla)" in source
    assert "setpriv" in source
    assert "/root/.Xauthority" in source
    assert "XDG_RUNTIME_DIR" in source
    assert "FROM ubuntu:24.04" in dockerfile
    assert "usermod --login carla" in dockerfile
    assert "NVIDIA_DRIVER_CAPABILITIES=all" in dockerfile
    assert "docker/nvidia_icd.json" in dockerfile
    assert "libegl1" in dockerfile
    assert "xdg-user-dirs" in dockerfile
    assert Path("docker/nvidia_icd.json").is_file()


def test_pretrained_weights_are_external_and_reproducible():
    dockerfile = Path("Dockerfile").read_text(encoding="utf-8")
    dockerignore = Path(".dockerignore").read_text(encoding="utf-8")
    entrypoint = Path("entrypoint.sh").read_text(encoding="utf-8")
    gitmodules = Path(".gitmodules").read_text(encoding="utf-8")
    downloader = Path("scripts/download_pcla_pretrained.sh").read_text(encoding="utf-8")

    assert "PCLA-wrapper/PCLA/pcla_agents/*_pretrained/" in dockerignore
    assert (
        "PCLA-wrapper/PCLA/pcla_agents/plant*/carla_garage/speed_limits/"
        "OpenDriveMap_speed_limits.npy"
    ) in dockerignore
    assert 'ln -s "/opt/pcla-pretrained/${name}"' in dockerfile
    assert "ENV PCLA_PRETRAINED_ROOT=/opt/pcla-pretrained" in dockerfile
    assert "ENV CUBLAS_WORKSPACE_CONFIG=:4096:8" in dockerfile
    assert 'map_name == "OpenDriveMap"' in dockerfile
    assert "MapImage.draw_map_image" in dockerfile
    assert 'export PCLA_PRETRAINED_ROOT="${PCLA_PRETRAINED_ROOT:-' in entrypoint
    assert 'export CUBLAS_WORKSPACE_CONFIG="${CUBLAS_WORKSPACE_CONFIG:-:4096:8}"' in entrypoint
    assert "https://github.com/sysnycu/PCLA.git" in gitmodules
    assert "branch = pisa-integration" in gitmodules
    assert "curl -fL --retry 5" in downloader
    assert "sha256sum --check -" in downloader
    assert "scripts/validate_pcla_pretrained.py" in downloader


def test_duplicate_map_assets_are_deduplicated_in_the_image():
    dockerfile = Path("Dockerfile").read_text(encoding="utf-8")
    dockerignore = Path(".dockerignore").read_text(encoding="utf-8")

    duplicate_map_paths = (
        "PCLA-wrapper/PCLA/pcla_agents/simlingo/birds_eye_view/maps_2ppm_cv/",
        "PCLA-wrapper/PCLA/pcla_agents/simlingo/birds_eye_view/maps_4ppm_cv/",
        "PCLA-wrapper/PCLA/pcla_agents/simlingo/birds_eye_view/maps_8ppm_cv/",
        "PCLA-wrapper/PCLA/pcla_agents/simlingo/birds_eye_view/maps_high_res/",
        "PCLA-wrapper/PCLA/pcla_agents/transfuserv5/birds_eye_view/maps_2ppm_cv/",
        "PCLA-wrapper/PCLA/pcla_agents/transfuserv5/birds_eye_view/maps_4ppm_cv/",
        "PCLA-wrapper/PCLA/pcla_agents/transfuserv5/birds_eye_view/maps_8ppm_cv/",
        "PCLA-wrapper/PCLA/pcla_agents/transfuserv5/birds_eye_view/maps_high_res/",
        "PCLA-wrapper/PCLA/pcla_agents/carl/birds_eye_view/maps_2ppm_cv/",
        ("PCLA-wrapper/PCLA/pcla_agents/transfuserv6/lead/expert/hdmap/maps_2ppm_cv/"),
    )
    for path in duplicate_map_paths:
        assert path in dockerignore

    duplicate_speed_limit_paths = (
        ("PCLA-wrapper/PCLA/pcla_agents/plant2/carla_garage/speed_limits/*_speed_limits.npy"),
        "PCLA-wrapper/PCLA/pcla_agents/simlingo/speed_limits/*_speed_limits.npy",
        ("PCLA-wrapper/PCLA/pcla_agents/transfuserv5/speed_limits/*_speed_limits.npy"),
    )
    for path in duplicate_speed_limit_paths:
        assert path in dockerignore

    assert "canonical_maps=/app/PCLA-wrapper/PCLA/pcla_agents/plant2/" in dockerfile
    assert "canonical_speed_limits=/app/PCLA-wrapper/PCLA/pcla_agents/plant/" in dockerfile


def test_plant_route_planner_uses_world_coordinates():
    source = Path("PCLA-wrapper/PCLA/pcla_agents/plant/PlanT_agent.py").read_text(encoding="utf-8")
    assert "set_route(self._global_plan_world_coord, False)" in source
    assert "set_route(self._global_plan, True)" not in source
    assert "downsample_route(global_plan_world_coord, 50)" in source
    assert "downsample_route(global_plan_world_coord, 200)" not in source
    assert "PlanT state step=%d" in source


def test_plant_privileged_route_index_tracks_actual_prefix():
    source = Path(
        "PCLA-wrapper/PCLA/pcla_agents/plant/carla_garage/privileged_route_planner.py"
    ).read_text(encoding="utf-8")
    assert "self.route_index = 0" in source
    assert "self.route_index += self.points_per_meter" in source


def test_plant_planners_compute_dynamic_opendrive_speed_limits():
    for agent in ("plant", "plant2"):
        source = Path(
            f"PCLA-wrapper/PCLA/pcla_agents/{agent}/carla_garage/privileged_route_planner.py"
        ).read_text(encoding="utf-8")
        assert 'map_name == "OpenDriveMap"' in source
        assert "carla_map.to_opendrive()" in source
        assert '"mph": 1.609344' in source
        assert "previous_speed_limit = 50.0" in source
        assert "previous_speed_limit / 3.6" in source
        assert "if speed_limit <= category" in source


def test_plant2_generates_bev_masks_for_dynamic_opendrive_maps():
    manager = Path(
        "PCLA-wrapper/PCLA/pcla_agents/plant2/carla_garage/birds_eye_view/chauffeurnet.py"
    ).read_text(encoding="utf-8")
    generator = Path(
        "PCLA-wrapper/PCLA/pcla_agents/plant2/carla_garage/birds_eye_view/birdview_map_opencv.py"
    ).read_text(encoding="utf-8")

    assert 'self._town != "OpenDriveMap"' in manager
    assert "MapImage.draw_map_image(" in manager
    assert "precision=0.5" in manager
    assert "TrafficLightHandler.reset(self._world)" in manager
    assert "dtype=np.uint8" in generator
    assert "from .traffic_light import TrafficLightHandler" in generator


def test_neat_agents_use_world_route_in_native_coordinate_frame():
    planner = Path("PCLA-wrapper/PCLA/pcla_agents/neat/planner.py").read_text(encoding="utf-8")
    assert "def set_route_world(self, global_plan):" in planner
    assert "np.array([-location.y, location.x])" in planner

    for agent_path in (
        "PCLA-wrapper/PCLA/pcla_agents/neat/neat_agent.py",
        "PCLA-wrapper/PCLA/pcla_agents/neat/aim_mt_2d_agent.py",
        "PCLA-wrapper/PCLA/pcla_agents/neat/aim_mt_bev_agent.py",
    ):
        source = Path(agent_path).read_text(encoding="utf-8")
        assert "set_route_world(self._global_plan_world_coord)" in source
        assert "set_route(self._global_plan, True)" not in source
        assert "return np.array([-location.y, location.x])" in source


def test_constructor_is_lazy():
    before = set(sys.modules)
    adapter = PclaAV()
    added = set(sys.modules) - before
    assert adapter._carla is None
    assert adapter._pcla is None
    assert "carla" not in added
    assert "PCLA" not in added


def test_flat_and_nested_config_are_compatible_and_conflicts_fail():
    adapter = PclaAV()
    assert adapter._normalize_config({"pcla": {"agent": "carl_plant_3"}}) == {
        "pcla_agent": "carl_plant_3"
    }
    assert adapter._normalize_config({"carla": {"sync": False}})["sync"] is False
    assert adapter._normalize_config({"carla": {"host": "carla", "port": 2001}}) == {
        "carla_host": "carla",
        "carla_port": 2001,
    }
    with pytest.raises(InvalidAvRequest, match="Conflicting"):
        adapter._normalize_config({"pcla_agent": "carl_plant_3", "pcla": {"agent": "carl_carl_1"}})


def test_config_validation_for_agent_identity_sign_and_timeout(tmp_path):
    root = tmp_path / "PCLA"
    write_agents(root)
    adapter = PclaAV()
    adapter.config = {
        "pcla_root": str(root),
        "pcla_agent": "missing_agent",
        "object_identity_mode": "bad",
    }
    with pytest.raises(InvalidAvRequest, match="object_identity_mode"):
        adapter._parse_config()

    adapter.config = {"pcla_root": str(root), "coordinate_y_sign": 0}
    with pytest.raises(InvalidAvRequest, match="non-zero"):
        adapter._parse_config()

    adapter.config = {"pcla_root": str(root), "retry_interval_seconds": 0}
    with pytest.raises(InvalidAvRequest, match="retry_interval_seconds"):
        adapter._parse_config()

    adapter.config = {"pcla_root": str(root), "sensor_warmup_ticks": -1}
    with pytest.raises(InvalidAvRequest, match="sensor_warmup_ticks"):
        adapter._parse_config()

    adapter.config = {"pcla_root": str(root), "sensor_warmup_ticks": 1.5}
    with pytest.raises(InvalidAvRequest, match="integer"):
        adapter._parse_config()

    adapter.config = {"pcla_root": str(root), "pcla_agent": "missing_agent"}
    adapter._parse_config()
    with pytest.raises(InvalidAvRequest, match="Accepted formats"):
        adapter._validate_agent_name()


def test_init_parses_request_without_loading_models(tmp_path):
    root = tmp_path / "PCLA"
    write_agents(root)
    adapter = PclaAV()
    adapter._ensure_connected = lambda: True
    adapter._prepare_reused_server_state = lambda: None
    request = SimpleNamespace(
        output_dir=tmp_path / "output",
        dt=0.05,
        config={
            "pcla_root": str(root),
            "pcla_agent": "carl_plant_3",
            "launch_carla_server": False,
        },
    )
    adapter.init(request)
    assert adapter._initialized is True
    assert adapter._fixed_delta_seconds == pytest.approx(0.05)
    assert adapter._pcla is None
    assert adapter._pcla_module is None


def test_string_false_disables_owned_carla_launch(tmp_path):
    root = tmp_path / "PCLA"
    write_agents(root)
    adapter = PclaAV()
    adapter.config = {
        "pcla_root": str(root),
        "launch_carla_server": "false",
        "sync": "true",
        "no_rendering": "false",
    }

    adapter._parse_config()

    assert adapter._launch_carla_server is False
    assert adapter._sync is True
    assert adapter._no_rendering is False


def test_invalid_boolean_config_is_rejected(tmp_path):
    root = tmp_path / "PCLA"
    write_agents(root)
    adapter = PclaAV()
    adapter.config = {
        "pcla_root": str(root),
        "launch_carla_server": "external",
    }

    with pytest.raises(InvalidAvRequest, match="launch_carla_server must be a boolean"):
        adapter._parse_config()


def test_environment_overrides_agent_route_and_carla(monkeypatch, tmp_path):
    route = tmp_path / "route.xml"
    monkeypatch.setenv("PCLA_AGENT", "carl_plant_9")
    monkeypatch.setenv("PCLA_ROUTE", str(route))
    monkeypatch.setenv("CARLA_HOST", "external")
    monkeypatch.setenv("CARLA_PORT", "2100")
    monkeypatch.setenv("CARLA_TIMEOUT", "33")
    adapter = PclaAV()
    adapter.config = {
        "pcla_agent": "carl_plant_1",
        "route_xml_path": "ignored.xml",
        "carla_host": "ignored",
        "carla_port": 2000,
    }
    adapter._parse_config()
    assert adapter._agent_name == "carl_plant_9"
    assert adapter._route_path_cfg == str(route)
    assert adapter._host == "external"
    assert adapter._port == 2100
    assert adapter._carla_timeout == 33.0


def test_reset_output_dir_is_relative_to_init_output_base(tmp_path):
    adapter = PclaAV()
    adapter._output_base = tmp_path / "output"

    resolved = adapter._resolve_reset_output_dir(Path("concrete"))

    assert resolved == (tmp_path / "output" / "concrete").resolve()
    assert resolved.is_dir()


def test_absolute_reset_output_dir_is_preserved(tmp_path):
    adapter = PclaAV()
    adapter._output_base = tmp_path / "base"
    absolute = tmp_path / "absolute-case"

    resolved = adapter._resolve_reset_output_dir(absolute)

    assert resolved == absolute
    assert resolved.is_dir()


def test_reset_output_dir_cannot_escape_init_output_base(tmp_path):
    adapter = PclaAV()
    adapter._output_base = tmp_path / "output"

    with pytest.raises(InvalidAvRequest, match="escapes Init output base"):
        adapter._resolve_reset_output_dir(Path("../outside"))


def test_pcla_runtime_dir_defaults_to_reset_output_and_blocks_escape(tmp_path):
    adapter = PclaAV()
    adapter._output_dir = tmp_path / "case"
    adapter.config = {}

    resolved = adapter._resolve_pcla_runtime_dir()

    assert resolved == (tmp_path / "case" / "pcla_runtime").resolve()
    assert resolved.is_dir()

    adapter.config = {"pcla_runtime_dir": "../outside"}
    with pytest.raises(InvalidAvRequest, match="escapes Reset output"):
        adapter._resolve_pcla_runtime_dir()


def test_absolute_pcla_runtime_dir_is_preserved(tmp_path):
    adapter = PclaAV()
    adapter._output_dir = tmp_path / "case"
    runtime_dir = tmp_path / "shared-runtime"
    adapter.config = {"pcla_runtime_dir": str(runtime_dir)}

    assert adapter._resolve_pcla_runtime_dir() == runtime_dir.resolve()
    assert runtime_dir.is_dir()


def test_owned_carla_gets_writable_home_and_cache(monkeypatch, tmp_path):
    adapter = PclaAV()
    adapter._output_dir = tmp_path / "output"
    adapter._carla_home = tmp_path / "carla-home"
    adapter.config = {"carla_server_script": "/fake/carla_server.sh"}
    captured = {}

    class Process:
        pid = 123

    def popen(args, **kwargs):
        captured["args"] = args
        captured.update(kwargs)
        return Process()

    monkeypatch.setattr("pcla_wrapper.pcla_av.subprocess.Popen", popen)

    adapter._launch_server()

    assert captured["args"] == ["/fake/carla_server.sh"]
    assert captured["env"]["HOME"] == str(adapter._carla_home)
    assert captured["env"]["CARLA_HOME"] == str(adapter._carla_home)
    assert captured["env"]["XDG_CACHE_HOME"] == str(adapter._carla_home / ".cache")
    assert captured["start_new_session"] is True
    assert (adapter._carla_home / "carlaCache").is_dir()
    assert (adapter._carla_home / ".cache").is_dir()


def test_connection_restores_timeout_on_success_and_failure(monkeypatch):
    adapter = PclaAV()
    adapter._carla = SimpleNamespace(Client=lambda host, port: client)
    adapter._host = "localhost"
    adapter._port = 2000
    adapter._carla_timeout = 12.0
    client = FakeClient()
    adapter._connect_once()
    assert client.timeouts == [2.0, 12.0]

    class FailingClient(FakeClient):
        def get_server_version(self):
            raise RuntimeError("not ready")

    failing = FailingClient()
    adapter._client = None
    adapter._server_version = None
    adapter._carla = SimpleNamespace(Client=lambda host, port: failing)
    with pytest.raises(RuntimeError, match="not ready"):
        adapter._connect_once()
    assert failing.timeouts == [2.0, 12.0]


def test_connection_retry_uses_total_timeout(monkeypatch):
    adapter = PclaAV()
    attempts = []
    adapter._server_version = None
    adapter._client = None
    adapter._connect_timeout = 0.2
    adapter._retry_interval = 0.01

    def fail():
        attempts.append(1)
        raise RuntimeError("offline")

    monkeypatch.setattr(adapter, "_connect_once", fail)
    assert adapter._ensure_connected() is False
    assert len(attempts) > 1


@pytest.mark.parametrize(
    "scenario,observation,message",
    [
        (None, [object_state()], "ScenarioPack"),
        (SimpleNamespace(map_name="", ego=object()), [object_state()], "map_name"),
        (SimpleNamespace(map_name="Town", ego=object()), [], "Initial observation"),
        (
            SimpleNamespace(
                map_name="Town",
                ego=SimpleNamespace(goal_config=SimpleNamespace(position=None)),
            ),
            [object_state()],
            "goal position",
        ),
    ],
)
def test_reset_request_validation(scenario, observation, message):
    with pytest.raises(InvalidAvRequest, match=message):
        PclaAV()._validate_reset_request(scenario, observation)


def test_same_map_opendrive_world_is_reused(tmp_path):
    adapter = configured_adapter()
    path = (tmp_path / "Town.xodr").resolve()
    adapter._xodr_root = tmp_path
    adapter._reuse_generated_world = True
    adapter._loaded_map_name = "Town"
    adapter._loaded_opendrive_path = path
    adapter._ensure_world("Town")
    assert adapter._client.generate_calls == 0


def test_opendrive_timeout_is_restored_on_failure(tmp_path):
    adapter = configured_adapter()
    adapter._xodr_root = tmp_path
    adapter._reuse_generated_world = False
    adapter._carla_timeout = 17.0
    (tmp_path / "Town.xodr").write_text("<OpenDRIVE/>", encoding="utf-8")
    adapter._client.generate_error = RuntimeError("generation failed")
    with pytest.raises(AvPreconditionFailed, match="generate"):
        adapter._ensure_world("Town")
    assert adapter._client.timeouts[-2:] == [300.0, 17.0]


def test_missing_opendrive_is_invalid_request(tmp_path):
    adapter = configured_adapter()
    adapter._xodr_root = tmp_path
    adapter._reuse_generated_world = False
    with pytest.raises(InvalidAvRequest, match="not found"):
        adapter._ensure_world("Town")


def test_route_path_validation_and_output_isolation(tmp_path):
    adapter = configured_adapter()
    adapter._pcla_root = tmp_path
    adapter._route_path_cfg = "missing.xml"
    scenario = SimpleNamespace(
        name="x",
        ego=SimpleNamespace(goal_config=SimpleNamespace(position=kinematic(x=10))),
    )
    with pytest.raises(InvalidAvRequest, match="not readable"):
        adapter._resolve_route_path(scenario, [object_state()])

    route = tmp_path / "route.xml"
    route.write_text("<route/>", encoding="utf-8")
    adapter._route_path_cfg = str(route)
    assert adapter._resolve_route_path(scenario, [object_state()]) == route


def test_generated_route_logs_raw_converted_and_projected_endpoints(tmp_path, caplog):
    adapter = configured_adapter()
    adapter._route_path_cfg = None
    adapter._output_dir = tmp_path / "case-a"
    adapter._route_wp_distance = 2.0
    adapter._route_draw = False
    adapter._coordinate_y_sign = -1.0
    adapter._pcla_module = SimpleNamespace(
        location_to_waypoint=lambda *args, **kwargs: [
            SimpleNamespace(transform=FakeTransform(FakeLocation(0, 0, 0))),
            SimpleNamespace(transform=FakeTransform(FakeLocation(1, 0, 0))),
        ],
        route_maker=lambda waypoints, savePath: Path(savePath).write_text(
            "<route/>", encoding="utf-8"
        ),
    )
    scenario = SimpleNamespace(
        name="../../unsafe",
        ego=SimpleNamespace(
            goal_config=SimpleNamespace(position=kinematic(x=10, y=6, z=1)),
        ),
    )
    with caplog.at_level("INFO", logger="pcla_wrapper.pcla_av"):
        route = adapter._resolve_route_path(
            scenario,
            [SimpleNamespace(kinematic=kinematic(x=2, y=4, z=1))],
        )
    assert route.parent == tmp_path / "case-a" / "pcla_routes"
    assert ".." not in route.name
    assert route.read_text(encoding="utf-8") == "<route/>"
    assert "PISA start=(2.000, 4.000, 1.000) goal=(10.000, 6.000, 1.000)" in caplog.text
    assert "CARLA start=(2.000, -4.000, 1.000) goal=(10.000, -6.000, 1.000)" in caplog.text
    assert "Projected route endpoints" in caplog.text

    adapter._pcla_module.location_to_waypoint = lambda *args, **kwargs: []
    with pytest.raises(AvPreconditionFailed, match="fewer than two"):
        adapter._resolve_route_path(scenario, [object_state()])


def test_pcla_constructor_uses_writable_runtime_and_restores_cwd(tmp_path):
    adapter = configured_adapter()
    calls = []
    original_cwd = Path.cwd()
    adapter._pcla_runtime_dir = tmp_path / "runtime"
    adapter._pcla_runtime_dir.mkdir()

    class FakePcla:
        def __init__(self, *args, **kwargs):
            Path("plant_viz/run").mkdir(parents=True)
            calls.append((args, kwargs, Path.cwd()))

    adapter._pcla_module = SimpleNamespace(PCLA=FakePcla)
    adapter._agent_name = "carl_plant_3"
    route = Path("/tmp/route.xml")
    adapter._build_pcla(route)
    args, kwargs, constructor_cwd = calls[0]
    assert args == ("carl_plant_3", adapter._vehicle, str(route), adapter._client)
    assert kwargs == {"destroy_vehicle": False}
    assert constructor_cwd == adapter._pcla_runtime_dir
    assert (adapter._pcla_runtime_dir / "plant_viz" / "run").is_dir()
    assert Path.cwd() == original_cwd


def test_pcla_runtime_cwd_restores_after_failure(tmp_path):
    adapter = configured_adapter()
    original_cwd = Path.cwd()
    adapter._pcla_runtime_dir = tmp_path / "runtime"
    adapter._pcla_runtime_dir.mkdir()
    adapter._agent_name = "carl_plant_3"

    def fail(*args, **kwargs):
        assert Path.cwd() == adapter._pcla_runtime_dir
        raise RuntimeError("setup failed")

    adapter._pcla_module = SimpleNamespace(PCLA=fail)
    with pytest.raises(RuntimeError, match="setup failed"):
        adapter._build_pcla(Path("route.xml"))
    assert Path.cwd() == original_cwd


def test_pcla_constructor_maps_timeout_and_missing_weights():
    adapter = configured_adapter()
    adapter._agent_name = "carl_plant_3"
    adapter._pcla_module = SimpleNamespace(
        PCLA=lambda *args, **kwargs: (_ for _ in ()).throw(TimeoutError())
    )
    with pytest.raises(AvTimeout, match="Timed out loading"):
        adapter._build_pcla(Path("route.xml"))
    adapter._pcla_module = SimpleNamespace(
        PCLA=lambda *args, **kwargs: (_ for _ in ()).throw(FileNotFoundError("checkpoint.pt"))
    )
    with pytest.raises(AvUnavailable, match="checkpoint.pt"):
        adapter._build_pcla(Path("route.xml"))


def test_camera_sensor_enables_rendering_and_warms_up():
    adapter = configured_adapter()
    adapter._world.settings.no_rendering_mode = True
    adapter._sensor_warmup_ticks = 2
    adapter._pcla = SimpleNamespace(
        _sensors=[
            SimpleNamespace(type_id="sensor.camera.rgb"),
            SimpleNamespace(type_id="sensor.other.imu"),
        ]
    )

    adapter._prepare_pcla_sensors()

    assert adapter._world.settings.no_rendering_mode is False
    assert adapter._world.tick_calls == 2


def test_driving_state_log_includes_heading_and_converted_steer(caplog):
    adapter = configured_adapter()
    adapter._debug_log_interval_steps = 20
    adapter._step_count = 1
    adapter._last_timestamp_ns = 123
    adapter._steer_sign = -1.0
    adapter._route_start_location = FakeLocation(0, 0, 0)
    adapter._route_goal_location = FakeLocation(100, 0, 0)
    adapter._vehicle.transform = FakeTransform(FakeLocation(5, 0, 0), FakeRotation(yaw=10))
    adapter._vehicle.velocity = FakeVector(3, 4, 0)
    adapter._vehicle.get_transform = lambda: adapter._vehicle.transform
    adapter._vehicle.get_velocity = lambda: adapter._vehicle.velocity
    action = SimpleNamespace(throttle=0.5, brake=0.0, steer=0.25)

    with caplog.at_level("INFO", logger="pcla_wrapper.pcla_av"):
        adapter._log_driving_state(kinematic(x=5, yaw=0.1, speed=5), action)

    assert "heading_error_deg=-10.000" in caplog.text
    assert "speed=5.000" in caplog.text
    assert "output_steer=-0.250" in caplog.text


def test_sensor_warmup_reports_owned_server_exit():
    adapter = configured_adapter()
    adapter._sensor_warmup_ticks = 1
    adapter._pcla = SimpleNamespace(_sensors=[])
    adapter._server_owned = True
    adapter._server_process = FakeProcess(return_code=139)
    adapter._output_base = Path("/mnt/output")

    with pytest.raises(AvUnavailable, match="return code 139"):
        adapter._prepare_pcla_sensors()
    assert "stderr.log" in adapter.should_quit().msg


def test_coordinate_yaw_yaw_rate_and_steer_conversion():
    adapter = configured_adapter()
    adapter._coordinate_y_sign = -1.0
    adapter._yaw_sign = -1.0
    adapter._steer_sign = -1.0
    adapter._yaw_offset_deg = 10.0
    location = adapter._to_carla_location(kinematic(x=1, y=2, z=3))
    assert (location.x, location.y, location.z) == (1.0, -2.0, 3.0)
    assert adapter._to_carla_yaw(3.141592653589793 / 2) == pytest.approx(-80.0)
    actor = FakeActor(1)
    adapter._apply_kinematic(actor, kinematic(yaw_rate=1.0))
    assert actor.angular_velocity.z == pytest.approx(-57.2957795)

    adapter._pcla = SimpleNamespace(
        get_action=lambda snapshot=None: SimpleNamespace(throttle=0.2, brake=0.1, steer=0.4)
    )
    adapter._data_provider = None
    adapter._vehicle = actor
    response = adapter.step(SimpleNamespace(observation=[object_state()], timestamp_ns=5))
    assert response.ctrl_cmd.payload["steer"] == pytest.approx(-0.4)


def test_extract_xyz_rejects_missing_or_non_numeric_fields():
    adapter = PclaAV()
    with pytest.raises(InvalidAvRequest, match="missing"):
        adapter._extract_xyz(SimpleNamespace(x=1, y=2))
    with pytest.raises(InvalidAvRequest, match="numeric"):
        adapter._extract_xyz(SimpleNamespace(x="bad", y=2, z=3))


def test_actor_identity_modes_disappearance_type_change_and_kinematic():
    world = FakeWorld(
        FakeBlueprintLibrary(
            patterns={
                "vehicle.*": [FakeBlueprint()],
                "walker.pedestrian.*": [FakeBlueprint("walker.pedestrian.0001")],
            }
        )
    )
    adapter = configured_adapter(world)
    ego = object_state()
    first = object_state(1)

    adapter._object_identity_mode = "index"
    adapter._update_and_tick([ego, first])
    actor = adapter._other_actors_by_key[("index", 0)]
    adapter._update_and_tick([ego, first])
    assert adapter._other_actors_by_key[("index", 0)] is actor
    assert actor.physics_calls[-1] is False
    assert actor.gravity_calls[-1] is False
    assert adapter._vehicle.physics_calls == []

    changed = object_state(1, RoadObjectType.PEDESTRIAN)
    adapter._update_and_tick([ego, changed])
    replacement = adapter._other_actors_by_key[("index", 0)]
    assert replacement is not actor
    assert actor.destroy_calls == 1
    adapter._update_and_tick([ego])
    assert replacement.destroy_calls == 1


def test_stateless_and_provided_identity():
    adapter = configured_adapter()
    ego = object_state()
    item = object_state(1, id="stable")
    adapter._object_identity_mode = "stateless"
    adapter._update_and_tick([ego, item])
    first = adapter._other_actors_by_key[("frame", 0)]
    adapter._update_and_tick([ego, item])
    assert adapter._other_actors_by_key[("frame", 0)] is not first

    adapter._object_identity_mode = "provided"
    adapter._destroy_other_actors()
    adapter._update_and_tick([ego, item])
    provided = adapter._other_actors_by_key[("id", "stable")]
    adapter._update_and_tick([ego, item])
    assert adapter._other_actors_by_key[("id", "stable")] is provided
    with pytest.raises(InvalidAvRequest, match="requires one of"):
        adapter._update_and_tick([ego, object_state(2)])


def test_spawn_overlap_retries_and_teleports_to_observation():
    adapter = configured_adapter()
    actor = FakeActor(9)
    adapter._world.spawn_results = [None, actor]
    blueprint = FakeBlueprint()
    transform = FakeTransform(FakeLocation(1, 2, 3), FakeRotation())
    result = adapter._spawn_actor_allowing_observation_overlap(blueprint, transform)
    assert result is actor
    adapter._apply_kinematic(result, kinematic(x=1, y=2, z=3), kinematic=True)
    assert result.transforms[-1].location.z == 3.0


def test_step_ticks_once_before_provider_and_action_with_same_snapshot():
    adapter = configured_adapter()
    events = adapter._world.events

    class Provider:
        @staticmethod
        def on_carla_tick():
            events.append("provider")

    class Agent:
        def get_action(self, snapshot=None):
            events.append(("action", snapshot))
            return SimpleNamespace(throttle=0.1, brake=0.0, steer=0.2)

    adapter._data_provider = Provider
    adapter._pcla = Agent()
    response = adapter.step(SimpleNamespace(observation=[object_state()], timestamp_ns=42))
    assert isinstance(response, StepResponse)
    assert response.ctrl_cmd.mode == ControlMode.THROTTLE_STEER_BREAK
    assert adapter._world.tick_calls == 1
    assert events == [
        "tick",
        "snapshot",
        "provider",
        ("action", adapter._world.snapshot),
    ]
    assert adapter._last_timestamp_ns == 42


def test_action_and_cleanup_use_runtime_directory(tmp_path):
    adapter = configured_adapter()
    original_cwd = Path.cwd()
    adapter._pcla_runtime_dir = tmp_path / "runtime"
    adapter._pcla_runtime_dir.mkdir()
    calls = []

    class Agent:
        def get_action(self, snapshot=None):
            calls.append(("action", Path.cwd()))
            Path("action-output").mkdir()
            return SimpleNamespace(throttle=0.1, brake=0.0, steer=0.2)

        def cleanup(self):
            calls.append(("cleanup", Path.cwd()))
            Path("cleanup-output").mkdir()

    adapter._pcla = Agent()
    adapter._get_action(object())
    adapter._finalize()

    assert calls == [
        ("action", adapter._pcla_runtime_dir),
        ("cleanup", adapter._pcla_runtime_dir),
    ]
    assert (adapter._pcla_runtime_dir / "action-output").is_dir()
    assert (adapter._pcla_runtime_dir / "cleanup-output").is_dir()
    assert Path.cwd() == original_cwd


def test_none_action_and_pcla_exception_are_not_silent():
    adapter = configured_adapter()
    adapter._data_provider = None
    adapter._pcla = SimpleNamespace(get_action=lambda snapshot=None: None)
    with pytest.raises(AvPreconditionFailed, match="no action"):
        adapter.step(SimpleNamespace(observation=[object_state()], timestamp_ns=0))
    assert adapter.should_quit().should_quit is True

    def fail(snapshot=None):
        raise RuntimeError("model crashed")

    adapter._pcla = SimpleNamespace(get_action=fail)
    with pytest.raises(AvUnavailable, match="model crashed"):
        adapter.step(SimpleNamespace(observation=[object_state()], timestamp_ns=0))
    assert "model crashed" in adapter.should_quit().msg


def test_none_action_can_use_timeout_policy():
    adapter = configured_adapter()
    adapter._data_provider = None
    adapter._action_none_timeout = 0.001
    adapter._pcla = SimpleNamespace(get_action=lambda snapshot=None: None)
    with pytest.raises(AvTimeout, match="no action"):
        adapter.step(SimpleNamespace(observation=[object_state()], timestamp_ns=0))


def test_reset_partial_failure_finalizes():
    adapter = PclaAV()
    adapter._initialized = True
    adapter._finalized = True
    adapter._validate_reset_request = lambda scenario, observation: None
    adapter._ensure_world = lambda map_name: (_ for _ in ()).throw(RuntimeError("broken"))
    finalized = []
    adapter._finalize = lambda: finalized.append(True)
    request = SimpleNamespace(
        output_dir=Path("out"),
        scenario_pack=SimpleNamespace(map_name="Town"),
        initial_observation=[object_state()],
    )
    with pytest.raises(RuntimeError, match="broken"):
        adapter.reset(request)
    assert finalized == [True]


def test_should_quit_reports_owned_process_exit():
    adapter = PclaAV()
    adapter._server_owned = True
    adapter._server_process = FakeProcess(return_code=7)
    response = adapter.should_quit()
    assert isinstance(response, ShouldQuitResponse)
    assert response.should_quit is True
    assert "return code 7" in response.msg


def test_stop_is_idempotent_and_keeps_server_while_cleaning_agent_and_actors():
    adapter = configured_adapter()
    process = FakeProcess()
    pcla = SimpleNamespace(cleanup_calls=0)

    def cleanup():
        pcla.cleanup_calls += 1

    pcla.cleanup = cleanup
    adapter._pcla = pcla
    adapter._server_process = process
    adapter._server_owned = True
    vehicle = adapter._vehicle
    adapter.stop()
    adapter.stop()
    assert pcla.cleanup_calls == 1
    assert vehicle.destroy_calls == 1
    assert process.terminate_calls == 0
    assert adapter._server_process is process
    assert adapter._client is None
    assert adapter._world is None


def test_owned_server_termination_signals_the_process_group(monkeypatch):
    adapter = PclaAV()
    process = FakeProcess()
    process.pid = 123
    adapter._server_process = process
    adapter._server_owned = True
    signals = []
    monkeypatch.setattr(
        "pcla_wrapper.pcla_av.os.killpg",
        lambda process_group, sig: signals.append((process_group, sig)),
    )

    adapter._terminate_server_process()

    assert signals == [(123, 15)]
    assert process.wait_calls == 1
    assert adapter._server_process is None
    assert adapter._server_owned is False


def test_cleanup_helper_preserves_traffic_lights_and_static_props():
    vehicle = FakeActor(1, "vehicle.test")
    sensor = FakeActor(2, "sensor.camera.rgb")
    light = FakeActor(3, "traffic.traffic_light")
    prop = FakeActor(4, "static.prop.barrier")
    world = FakeWorld(actors=[vehicle, sensor, light, prop])
    world.settings.synchronous_mode = True
    world.settings.fixed_delta_seconds = 0.05
    clear_dynamic_actors(world)
    assert vehicle.destroy_calls == 1
    assert sensor.destroy_calls == 1
    assert light.destroy_calls == 0
    assert prop.destroy_calls == 0
    assert world.settings.synchronous_mode is False
    assert world.settings.fixed_delta_seconds is None


def load_upstream_pcla(monkeypatch):
    carla = ModuleType("carla")
    carla.Location = FakeLocation
    carla.Rotation = FakeRotation
    carla.Transform = FakeTransform
    monkeypatch.setitem(sys.modules, "carla", carla)

    functions = ModuleType("pcla_functions")
    functions.give_path = lambda *args: ("agent.py", "config")
    functions.setup_sensor_attributes = lambda blueprint, spec: blueprint
    functions.location_to_waypoint = lambda *args, **kwargs: []
    functions.route_maker = lambda *args, **kwargs: None
    monkeypatch.setitem(sys.modules, "pcla_functions", functions)

    leaderboard = ModuleType("leaderboard_codes")
    monkeypatch.setitem(sys.modules, "leaderboard_codes", leaderboard)
    modules = {
        "watchdog": {
            "Watchdog": lambda timeout: SimpleNamespace(start=lambda: None, stop=lambda: None)
        },
        "timer": {
            "GameTime": SimpleNamespace(
                restart=lambda: None,
                on_carla_tick=lambda timestamp: None,
            )
        },
        "route_indexer": {"RouteIndexer": object},
        "carla_data_provider": {
            "CarlaDataProvider": SimpleNamespace(
                register_actor=lambda actor: None,
                cleanup_calls=0,
                cleanup=lambda: (_ for _ in ()).throw(
                    AssertionError("destructive provider cleanup")
                ),
                _actor_velocity_map={},
                _actor_location_map={},
                _actor_transform_map={},
                _traffic_light_map={},
                _carla_actor_pool={},
                _vehicles_with_open_doors={},
                _map=object(),
                _world=object(),
                _all_actors=object(),
                _client=object(),
                _spawn_points=[],
                _ego_vehicle_route=[],
                _sync_flag=True,
                _spawn_index=4,
            )
        },
        "route_manipulation": {"interpolate_trajectory": lambda *args: ([], [])},
        "sensor_interface": {
            "CallBack": lambda *args: object(),
            "OpenDriveMapReader": object,
            "SpeedometerReader": object,
        },
    }
    for suffix, attrs in modules.items():
        module = ModuleType("leaderboard_codes." + suffix)
        for name, value in attrs.items():
            setattr(module, name, value)
        monkeypatch.setitem(sys.modules, "leaderboard_codes." + suffix, module)

    path = Path("PCLA-wrapper/PCLA/PCLA.py").resolve()
    spec = importlib.util.spec_from_file_location("pcla_upstream_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeSensor:
    def __init__(self):
        self.listen_calls = 0
        self.stop_calls = 0
        self.destroy_calls = 0

    def listen(self, callback):
        self.listen_calls += 1

    def is_listening(self):
        return self.listen_calls > self.stop_calls

    def stop(self):
        self.stop_calls += 1

    def destroy(self):
        self.destroy_calls += 1


def test_pcla_setup_partial_sensor_failure_cleans_owned_sensor(monkeypatch):
    module = load_upstream_pcla(monkeypatch)
    instance = module.PCLA.__new__(module.PCLA)
    first = FakeSensor()
    calls = []

    class SensorWorld:
        def get_blueprint_library(self):
            return FakeBlueprintLibrary(
                exact={
                    "sensor.camera.rgb": FakeBlueprint("sensor.camera.rgb"),
                    "sensor.lidar.ray_cast": FakeBlueprint("sensor.lidar.ray_cast"),
                }
            )

        def spawn_actor(self, blueprint, transform, vehicle):
            calls.append(blueprint.id)
            if len(calls) == 2:
                raise RuntimeError("spawn failed")
            return first

    specs = [
        {
            "type": "sensor.camera.rgb",
            "id": "camera",
            "x": 0,
            "y": 0,
            "z": 1,
            "pitch": 0,
            "roll": 0,
            "yaw": 0,
        },
        {
            "type": "sensor.lidar.ray_cast",
            "id": "lidar",
            "x": 0,
            "y": 0,
            "z": 1,
            "pitch": 0,
            "roll": 0,
            "yaw": 0,
        },
    ]
    instance.world = SensorWorld()
    instance.vehicle = FakeActor(1)
    instance.agent_instance = SimpleNamespace(
        sensors=lambda: specs,
        sensor_interface=object(),
    )
    instance._sensors = []
    with pytest.raises(RuntimeError, match="spawn failed"):
        instance.setup_sensors()
    assert first.stop_calls == 1
    assert first.destroy_calls == 1
    assert instance._sensors == []


def test_pcla_cleanup_only_owned_sensors_and_does_not_destroy_ego(monkeypatch):
    module = load_upstream_pcla(monkeypatch)
    instance = module.PCLA.__new__(module.PCLA)
    owned = FakeSensor()
    other = FakeSensor()
    vehicle = FakeActor(1)
    agent = SimpleNamespace(destroy_calls=0)

    def destroy_agent():
        agent.destroy_calls += 1

    agent.destroy = destroy_agent
    instance._watchdog = None
    instance.agent_instance = agent
    instance._sensors = [owned]
    instance._destroy_vehicle = False
    instance.vehicle = vehicle
    instance.world = SimpleNamespace(
        get_actors=lambda: (_ for _ in ()).throw(AssertionError("global actor scan"))
    )
    instance.current_dir = "x"
    instance.client = object()
    instance.agentPath = "x"
    instance.configPath = "x"
    instance.routePath = "x"
    module.CarlaDataProvider._carla_actor_pool[99] = other
    instance.cleanup()
    instance.cleanup()
    assert owned.destroy_calls == 1
    assert other.destroy_calls == 0
    assert vehicle.destroy_calls == 0
    assert agent.destroy_calls == 1
    assert module.CarlaDataProvider._carla_actor_pool == {}
    assert module.CarlaDataProvider._world is None


def test_pcla_resets_process_global_torch_state_between_agents(monkeypatch):
    calls = []
    cudnn = SimpleNamespace(benchmark=True, deterministic=True)
    fake_torch = SimpleNamespace(
        float32=object(),
        backends=SimpleNamespace(cudnn=cudnn),
        set_default_dtype=lambda dtype: calls.append(("dtype", dtype)),
        use_deterministic_algorithms=lambda enabled: calls.append(("deterministic", enabled)),
    )
    monkeypatch.setitem(sys.modules, "torch", fake_torch)
    module = load_upstream_pcla(monkeypatch)

    module._reset_torch_runtime_state()

    assert calls == [
        ("dtype", fake_torch.float32),
        ("deterministic", False),
    ]
    assert cudnn.benchmark is False
    assert cudnn.deterministic is False
    assert "CUBLAS_WORKSPACE_CONFIG" in module.os.environ


def test_pcla_done_delegates_to_agent(monkeypatch):
    module = load_upstream_pcla(monkeypatch)
    instance = module.PCLA.__new__(module.PCLA)
    instance.agent_instance = SimpleNamespace(done=lambda: True)
    assert instance.done() is True
    instance.agent_instance = SimpleNamespace()
    assert instance.done() is False


def test_no_production_sbsvf_or_debug_prints():
    production = [
        Path("pcla_wrapper/pcla_av.py"),
        Path("pcla_wrapper/server.py"),
        Path("PCLA-wrapper/PCLA_agent.py"),
        Path("PCLA-wrapper/server.py"),
    ]
    sources = [path.read_text(encoding="utf-8") for path in production]
    combined = "\n".join(sources)
    assert "sbsvf_api" not in combined
    for source in sources:
        calls = [
            node
            for node in ast.walk(ast.parse(source))
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "print"
        ]
        assert calls == []
    assert "grpc.server" not in combined
    assert inspect.signature(PclaAV).parameters == {}
