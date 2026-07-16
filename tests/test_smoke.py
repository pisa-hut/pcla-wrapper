import ast
import importlib.metadata
import importlib.util
import inspect
import json
import math
import os
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest
from google.protobuf.json_format import MessageToDict
from google.protobuf.struct_pb2 import Struct
from pisa_api.av import (
    AvPreconditionFailed,
    AvTimeout,
    AvUnavailable,
    ControlMode,
    GenericAvService,
    InitRequest,
    InitResponse,
    InvalidAvRequest,
    ObservationData,
    ObservedAgentData,
    RoadObjectType,
    ShapeCenterPoseData,
    ShapeData,
    ShapeDimensionData,
    ShapeType,
    ShouldQuitResponse,
    StepResponse,
    init_request_to_proto,
)
from pisa_api.empty_pb2 import Empty

from pcla_wrapper import version as version_module
from pcla_wrapper.lifecycle import clear_dynamic_actors
from pcla_wrapper.pcla_av import PclaAV
from pcla_wrapper.profiles import isolate_profile_import_paths, validate_image_profile
from pcla_wrapper.version import wrapper_version
from pcla_wrapper.weights import install_give_path_resolver, resolve_weight_path


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
    def __init__(self, blueprint_id="vehicle.test", dimensions=(4.0, 2.0, 1.5)):
        self.id = blueprint_id
        self.dimensions = dimensions
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
    def __init__(self, actor_id, type_id="vehicle.test", dimensions=(4.0, 2.0, 1.5)):
        self.id = actor_id
        self.type_id = type_id
        self.is_alive = True
        self.destroy_calls = 0
        self.transforms = []
        self.controls = []
        self.physics_calls = []
        self.gravity_calls = []
        self.bounding_box = SimpleNamespace(
            location=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            rotation=SimpleNamespace(roll=0.0, pitch=0.0, yaw=0.0),
            extent=SimpleNamespace(x=dimensions[0] / 2, y=dimensions[1] / 2, z=dimensions[2] / 2),
        )

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

    def apply_control(self, control):
        self.controls.append(control)

    def get_control(self):
        if self.controls:
            return self.controls[-1]
        return SimpleNamespace(throttle=0.0, brake=0.0, steer=0.0, gear=0)

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
            actor = FakeActor(
                100 + len(self.actors), blueprint.id, getattr(blueprint, "dimensions", (4, 2, 1.5))
            )
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


def kinematic(
    x=0.0,
    y=0.0,
    z=0.0,
    yaw=0.0,
    speed=0.0,
    yaw_rate=0.0,
    time_ns=0,
    acceleration=0.0,
    yaw_acceleration=0.0,
):
    return SimpleNamespace(
        x=x,
        y=y,
        z=z,
        yaw=yaw,
        speed=speed,
        yaw_rate=yaw_rate,
        time_ns=time_ns,
        acceleration=acceleration,
        yaw_acceleration=yaw_acceleration,
    )


def object_state(x=0.0, object_type=RoadObjectType.CAR, **extra):
    values = {"type": object_type, "kinematic": kinematic(x=x)}
    values.update(extra)
    return SimpleNamespace(**values)


def observation(ego=None, agents=()):
    return ObservationData(ego=ego or object_state(), agents=list(agents))


def observed_agent(state=None, *, tracking_id=None, entity_name=None):
    return ObservedAgentData(
        state=state or object_state(), tracking_id=tracking_id, entity_name=entity_name
    )


def box_shape(dimensions=(4.0, 2.0, 1.5), **center):
    return ShapeData(
        type=ShapeType.BOUNDING_BOX,
        dimensions=ShapeDimensionData(x=dimensions[0], y=dimensions[1], z=dimensions[2]),
        center=ShapeCenterPoseData(**center),
        reference_point="test_actor_origin",
    )


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
    adapter._traffic_manager_port = 8000
    adapter._manage_traffic_manager_sync = False
    adapter._action_none_timeout = 0.0
    adapter._sensor_warmup_ticks = 0
    adapter._fixed_delta_seconds = 0.05
    adapter._dt_ns = 50_000_000
    adapter._expected_timestamp_ns = 0
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
    assert "serve_av_system(PclaAV(), name=WRAPPER_NAME, version=wrapper_version())" in source
    assert "grpc.server" not in source
    assert "sbsvf_api" not in source


def test_wrapper_version_uses_distribution_metadata(monkeypatch):
    monkeypatch.setattr(version_module.metadata, "version", lambda name: "9.8.7")

    assert wrapper_version() == "9.8.7"


def test_wrapper_version_falls_back_to_checkout_pyproject(monkeypatch, tmp_path):
    package_dir = tmp_path / "pcla_wrapper"
    package_dir.mkdir()
    (tmp_path / "pyproject.toml").write_text(
        '[project]\nname = "pcla-wrapper"\nversion = "4.5.6"\n', encoding="utf-8"
    )
    monkeypatch.setattr(version_module, "__file__", str(package_dir / "version.py"))

    def missing(_name):
        raise importlib.metadata.PackageNotFoundError

    monkeypatch.setattr(version_module.metadata, "version", missing)

    assert wrapper_version() == "4.5.6"


def test_generated_service_ping_reports_wrapper_identity():
    service = GenericAvService(PclaAV(), name="pcla-wrapper", version="0.3.2")
    context = SimpleNamespace(peer=lambda: "test")

    response = service.Ping(Empty(), context)

    assert response.msg == "pcla-wrapper alive"
    assert response.name == "pcla-wrapper"
    assert response.version == "0.3.2"


def test_canonical_config_example_contains_current_keys_only():
    config = Path("pcla_config.yaml").read_text(encoding="utf-8")
    for key in (
        "pcla_agent",
        "pcla_pretrained_root",
        "carla_server_script",
        "carla_host",
        "carla_port",
        "carla_root",
        "carla_egg",
        "no_rendering",
    ):
        assert f"{key}:" in config
    assert "object_identity_mode:" not in config
    assert "config_example.yaml" not in config


def test_pcla_pseudosensor_emits_at_exact_frequency_boundary():
    path = Path("PCLA/leaderboard_codes/sensor_interface.py")
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    function = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "_measurement_due"
    )
    namespace = {}
    module = ast.fix_missing_locations(ast.Module(body=[function], type_ignores=[]))
    exec(compile(module, str(path), "exec"), namespace)
    measurement_due = namespace["_measurement_due"]

    assert measurement_due(0.05, 0.0, 20.0)
    assert measurement_due(0.10, 0.05, 20.0)
    assert not measurement_due(0.049, 0.0, 20.0)


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

    assert "PCLA/pcla_agents/*_pretrained/" in dockerignore
    assert (
        "PCLA/pcla_agents/plant*/carla_garage/speed_limits/OpenDriveMap_speed_limits.npy"
    ) in dockerignore
    assert 'ln -s "/mnt/weights"' in dockerfile
    assert "Mount the selected weight directory at /mnt/weights:ro." in dockerfile
    assert "pcla-" + "pretrained" not in dockerfile
    assert "ENV PCLA_PRETRAINED_ROOT=/mnt/weights" in dockerfile
    assert "ENV CUBLAS_WORKSPACE_CONFIG=:4096:8" in dockerfile
    assert 'map_name == "OpenDriveMap"' in dockerfile
    assert "MapImage.draw_map_image" in dockerfile
    assert 'export PCLA_PRETRAINED_ROOT="${PCLA_PRETRAINED_ROOT:-' in entrypoint
    assert 'export CUBLAS_WORKSPACE_CONFIG="${CUBLAS_WORKSPACE_CONFIG:-:4096:8}"' in entrypoint
    assert "https://github.com/sysnycu/PCLA.git" in gitmodules
    assert "branch = pisa-integration" in gitmodules
    assert "/app/scripts/smoke_common_agent.py" in dockerfile
    assert "/app/scripts/validate_common_runtime.py" in dockerfile
    assert "/opt/conda" not in dockerfile
    assert "/usr/local/cuda-11.8" not in dockerfile
    assert "FROM common-runtime AS common-slim" in dockerfile
    assert "PCLA_IMAGE_PROFILE=common" in dockerfile
    assert "ENV CARLA_NULLRHI=1" in dockerfile


def test_no_weight_bundling_image_build_path():
    profiles = json.loads(Path("pcla_wrapper/agent_profiles.json").read_text(encoding="utf-8"))

    assert not any(Path("docker").glob("*bundled*"))
    assert profiles["common"]["weight_directories"] == [
        "plant_pretrained",
        "plant2_pretrained",
        "carl_pretrained",
    ]
    assert "simlingo_simlingo" not in profiles["common"]["agents"]


def test_plant_profile_maps_each_seed_to_its_checkpoint():
    agents = json.loads(Path("pcla_wrapper/agent_profiles.json").read_text(encoding="utf-8"))[
        "common"
    ]["agents"]

    assert agents["carl_plant_0"] == ["plant_pretrained/last.ckpt"]
    for seed in range(1, 5):
        assert agents[f"carl_plant_{seed}"] == [f"plant_pretrained/last-v{seed}.ckpt"]


def test_weight_path_selects_nested_directory_from_profile_root(tmp_path):
    pretrained_root = tmp_path / "weights"
    pcla_root = tmp_path / "PCLA"
    checkpoint = pretrained_root / "plant2_pretrained" / "model.ckpt"
    checkpoint.parent.mkdir(parents=True)
    checkpoint.write_bytes(b"checkpoint")

    resolved = resolve_weight_path(
        str(pcla_root / "pcla_agents" / "plant2_pretrained" / "model.ckpt"),
        pcla_root,
        pretrained_root,
    )

    assert resolved == str(checkpoint)


def test_weight_path_selects_direct_directory_mount(tmp_path):
    pretrained_root = tmp_path / "selected_weights"
    pretrained_root.mkdir()
    checkpoint = pretrained_root / "model.ckpt"
    checkpoint.write_bytes(b"checkpoint")
    pcla_root = tmp_path / "PCLA"

    resolved = resolve_weight_path(
        str(pcla_root / "pcla_agents" / "plant2_pretrained" / "model.ckpt"),
        pcla_root,
        pretrained_root,
    )

    assert resolved == str(checkpoint)


def test_give_path_resolver_updates_plant2_checkpoint_without_writing_image(tmp_path):
    pcla_root = tmp_path / "PCLA"
    pretrained_root = tmp_path / "weights"
    checkpoint = pretrained_root / "plant2_pretrained" / "model.ckpt"
    checkpoint.parent.mkdir(parents=True)
    checkpoint.write_bytes(b"checkpoint")
    original_path = pcla_root / "pcla_agents" / "plant2_pretrained" / "model.ckpt"
    module = SimpleNamespace(give_path=lambda *_: (str(pcla_root / "agent.py"), str(original_path)))

    install_give_path_resolver(module, pretrained_root)
    agent_path, config_path = module.give_path("plant2_plant2_0", str(pcla_root), "")

    assert agent_path == str(pcla_root / "agent.py")
    assert config_path == str(checkpoint)
    assert os.environ["PLANT_CHECKPOINT"] == str(checkpoint)


def test_entrypoint_allows_runtime_validation_commands():
    entrypoint = Path("entrypoint.sh").read_text(encoding="utf-8")

    assert "pcla_wrapper.weights" not in entrypoint
    assert 'if (( $# > 0 )); then\n    exec "$@"\nfi' in entrypoint
    assert "exec /opt/pcla-venv/bin/python -m pcla_wrapper.server" in entrypoint


def test_repository_uses_unambiguous_wrapper_and_upstream_paths():
    gitmodules = Path(".gitmodules").read_text(encoding="utf-8")
    dockerfile = Path("Dockerfile").read_text(encoding="utf-8")

    assert Path("pcla_wrapper").is_dir()
    assert Path("PCLA").is_dir()
    assert not Path("PCLA-wrapper").exists()
    assert '[submodule "PCLA"]' in gitmodules
    assert "path = PCLA" in gitmodules
    assert "/app/PCLA/" in dockerfile
    assert "PCLA-wrapper" not in dockerfile


def test_duplicate_map_assets_are_deduplicated_in_the_image():
    dockerfile = Path("Dockerfile").read_text(encoding="utf-8")
    dockerignore = Path(".dockerignore").read_text(encoding="utf-8")

    duplicate_map_paths = (
        "PCLA/pcla_agents/simlingo/birds_eye_view/maps_2ppm_cv/",
        "PCLA/pcla_agents/simlingo/birds_eye_view/maps_4ppm_cv/",
        "PCLA/pcla_agents/simlingo/birds_eye_view/maps_8ppm_cv/",
        "PCLA/pcla_agents/simlingo/birds_eye_view/maps_high_res/",
        "PCLA/pcla_agents/transfuserv5/birds_eye_view/maps_2ppm_cv/",
        "PCLA/pcla_agents/transfuserv5/birds_eye_view/maps_4ppm_cv/",
        "PCLA/pcla_agents/transfuserv5/birds_eye_view/maps_8ppm_cv/",
        "PCLA/pcla_agents/transfuserv5/birds_eye_view/maps_high_res/",
        "PCLA/pcla_agents/carl/birds_eye_view/maps_2ppm_cv/",
        ("PCLA/pcla_agents/transfuserv6/lead/expert/hdmap/maps_2ppm_cv/"),
    )
    for path in duplicate_map_paths:
        assert path in dockerignore

    duplicate_speed_limit_paths = (
        ("PCLA/pcla_agents/plant2/carla_garage/speed_limits/*_speed_limits.npy"),
        "PCLA/pcla_agents/simlingo/speed_limits/*_speed_limits.npy",
        ("PCLA/pcla_agents/transfuserv5/speed_limits/*_speed_limits.npy"),
    )
    for path in duplicate_speed_limit_paths:
        assert path in dockerignore

    assert "/app/PCLA/pcla_agents/plant2/carla_garage/birds_eye_view/maps_2ppm_cv" in dockerfile
    assert "canonical_speed_limits=/app/PCLA/pcla_agents/plant/" in dockerfile


def test_common_profile_accepts_supported_agent_and_rejects_other_profiles(monkeypatch, tmp_path):
    pretrained_root = tmp_path / "weights"
    checkpoint = pretrained_root / "plant_pretrained" / "last-v3.ckpt"
    checkpoint.parent.mkdir(parents=True)
    checkpoint.write_bytes(b"checkpoint")
    monkeypatch.setenv("PCLA_IMAGE_PROFILE", "common")

    validate_image_profile("carl_plant_3", pretrained_root)

    with pytest.raises(InvalidAvRequest, match="not supported by image profile"):
        validate_image_profile("simlingo_simlingo", pretrained_root)


def test_common_profile_accepts_direct_selected_weight_dir(monkeypatch, tmp_path):
    checkpoint = tmp_path / "last-v3.ckpt"
    checkpoint.write_bytes(b"checkpoint")
    monkeypatch.setenv("PCLA_IMAGE_PROFILE", "common")

    validate_image_profile("carl_plant_3", tmp_path)


def test_common_profile_reports_missing_selected_agent_weights(monkeypatch, tmp_path):
    monkeypatch.setenv("PCLA_IMAGE_PROFILE", "common")

    with pytest.raises(InvalidAvRequest, match="weights are unavailable"):
        validate_image_profile("plant2_plant2_0", tmp_path)


def test_common_profile_removes_lmdrive_private_timm_path(monkeypatch, tmp_path):
    pcla_root = tmp_path / "PCLA"
    private_path = pcla_root / "pcla_agents" / "lmdrive" / "vision_encoder"
    private_path.mkdir(parents=True)
    fake_module_path = private_path / "timm" / "__init__.py"
    fake_module = ModuleType("pcla_test_private_timm")
    fake_module.__file__ = str(fake_module_path)
    monkeypatch.setitem(sys.modules, fake_module.__name__, fake_module)
    monkeypatch.setattr(sys, "path", [str(private_path), *sys.path])
    monkeypatch.setenv("PCLA_IMAGE_PROFILE", "common")

    isolate_profile_import_paths(pcla_root)

    assert str(private_path) not in sys.path
    assert fake_module.__name__ not in sys.modules


def test_plant_route_planner_uses_world_coordinates():
    source = Path("PCLA/pcla_agents/plant/PlanT_agent.py").read_text(encoding="utf-8")
    assert "set_route(self._global_plan_world_coord, False)" in source
    assert "set_route(self._global_plan, True)" not in source
    assert "downsample_route(global_plan_world_coord, 50)" in source
    assert "downsample_route(global_plan_world_coord, 200)" not in source
    assert "inital_frames_delay" not in source
    assert 'cfg.get("initial_control_delay_seconds", 0.0)' in source
    assert "self.step <= self.initial_control_delay_steps" in source

    eval_config = Path("PCLA/pcla_agents/plant/config/eval.yaml").read_text(encoding="utf-8")
    assert "initial_control_delay_seconds: 0.0" in eval_config


def test_plant_initial_control_delay_converts_seconds_to_ticks():
    path = Path("PCLA/pcla_agents/plant/PlanT_agent.py")
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    function = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "_initial_control_delay_steps"
    )
    namespace = {"math": math}
    module = ast.fix_missing_locations(ast.Module(body=[function], type_ignores=[]))
    exec(compile(module, str(path), "exec"), namespace)
    delay_steps = namespace["_initial_control_delay_steps"]

    assert delay_steps(0.0, 0.05) == (0.0, 0)
    assert delay_steps(2.0, 0.05) == (2.0, 40)
    assert delay_steps(0.01, 0.05) == (0.01, 1)
    for invalid_delay in (-0.1, float("inf"), True, "invalid"):
        with pytest.raises(ValueError, match="non-negative number"):
            delay_steps(invalid_delay, 0.05)


def test_carl_stop_sign_check_skips_expensive_waypoints_when_irrelevant():
    source = Path("PCLA/pcla_agents/carl/birds_eye_view/run_stop_sign.py").read_text(
        encoding="utf-8"
    )

    no_sign_fast_path = source.index("if not self.list_stop_signs:")
    actor_transform = source.index("actor_transform = vehicle.get_transform()", no_sign_fast_path)
    waypoint_generation = source.index("check_wps = self.get_waypoints(vehicle)", actor_transform)
    proximity_fast_path = source.index("if not self.target_stop_sign and all(", actor_transform)

    assert no_sign_fast_path < actor_transform < proximity_fast_path < waypoint_generation
    assert "self.stop_sign_locations.append(stop_location)" in source


def test_roach_bev_warps_a_local_crop_instead_of_the_full_map():
    source = Path("PCLA/pcla_agents/carl/birds_eye_view/chauffeurnet.py").read_text(
        encoding="utf-8"
    )

    assert "def warp_affine_local_crop(image, transform, output_width):" in source
    assert "inverse_transform = cv.invertAffineTransform(transform)" in source
    assert "cropped = np.ascontiguousarray(image[y0:y1, x0:x1])" in source
    assert "local_transform[:, 2] +=" in source
    assert "warp_affine_local_crop(self.hd_map_array, m_warp, self._width)" in source
    assert "cv.warpAffine(self.hd_map_array" not in source


def test_plant_privileged_route_index_tracks_actual_prefix():
    source = Path("PCLA/pcla_agents/plant/carla_garage/privileged_route_planner.py").read_text(
        encoding="utf-8"
    )
    assert "self.route_index = 0" in source
    assert "self.route_index += self.points_per_meter" in source


def test_plant_loads_hf_config_from_mounted_checkpoint_directory():
    agent = Path("PCLA/pcla_agents/plant/PlanT_agent.py").read_text(encoding="utf-8")
    model = Path("PCLA/pcla_agents/plant/model.py").read_text(encoding="utf-8")

    assert 'os.environ["PLANT_CHECKPOINT"] = LOAD_CKPT_PATH' in agent
    assert 'checkpoint_path = os.environ.get("PLANT_CHECKPOINT")' in model
    assert "Path(checkpoint_path).parent" in model
    assert "config = BertConfig.from_pretrained(" in model
    assert "local_files_only=bool(checkpoint_path)" in model


def test_plant_planners_compute_dynamic_opendrive_speed_limits():
    for agent in ("plant", "plant2"):
        source = Path(
            f"PCLA/pcla_agents/{agent}/carla_garage/privileged_route_planner.py"
        ).read_text(encoding="utf-8")
        assert 'map_name == "OpenDriveMap"' in source
        assert "carla_map.to_opendrive()" in source
        assert '"mph": 1.609344' in source
        assert "previous_speed_limit = 50.0" in source
        assert "previous_speed_limit / 3.6" in source
        assert "if speed_limit <= category" in source


def test_plant2_generates_bev_masks_for_dynamic_opendrive_maps():
    manager = Path("PCLA/pcla_agents/plant2/carla_garage/birds_eye_view/chauffeurnet.py").read_text(
        encoding="utf-8"
    )
    generator = Path(
        "PCLA/pcla_agents/plant2/carla_garage/birds_eye_view/birdview_map_opencv.py"
    ).read_text(encoding="utf-8")

    assert 'self._town != "OpenDriveMap"' in manager
    assert "MapImage.draw_map_image(" in manager
    assert "precision=0.5" in manager
    assert "TrafficLightHandler.reset(self._world)" in manager
    assert "dtype=np.uint8" in generator
    assert "from .traffic_light import TrafficLightHandler" in generator


def test_carl_generates_bev_masks_for_dynamic_opendrive_maps():
    manager = Path("PCLA/pcla_agents/carl/birds_eye_view/bev_observation.py").read_text(
        encoding="utf-8"
    )
    roach_manager = Path("PCLA/pcla_agents/carl/birds_eye_view/chauffeurnet.py").read_text(
        encoding="utf-8"
    )
    generator = Path("PCLA/pcla_agents/carl/birds_eye_view/birdview_map_opencv.py").read_text(
        encoding="utf-8"
    )
    agent = Path("PCLA/pcla_agents/carl/eval_agent.py").read_text(encoding="utf-8")
    red_light = Path("PCLA/pcla_agents/carl/reward/criteria/run_red_light.py").read_text(
        encoding="utf-8"
    )

    assert "map_name != 'OpenDriveMap'" in manager
    assert "MapImage.draw_map_image(" in manager
    assert "precision=0.5" in manager
    assert "map_name != 'OpenDriveMap'" in roach_manager
    assert "MapImage.draw_map_image(" in roach_manager
    assert "precision=0.5" in roach_manager
    assert "dtype=np.uint8" in generator
    assert "from .traffic_light import TrafficLightHandler" in generator
    assert "if not self.initialized:" in agent
    assert "assert TrafficLightHandler.num_tl > 0" not in red_light


def test_neat_agents_use_world_route_in_native_coordinate_frame():
    planner = Path("PCLA/pcla_agents/neat/planner.py").read_text(encoding="utf-8")
    assert "def set_route_world(self, global_plan):" in planner
    assert "np.array([-location.y, location.x])" in planner

    for agent_path in (
        "PCLA/pcla_agents/neat/neat_agent.py",
        "PCLA/pcla_agents/neat/aim_mt_2d_agent.py",
        "PCLA/pcla_agents/neat/aim_mt_bev_agent.py",
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


def test_config_validation_for_sign_and_timeout(tmp_path):
    root = tmp_path / "PCLA"
    write_agents(root)
    adapter = PclaAV()
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


def test_legacy_image_pcla_root_migrates_to_current_path(tmp_path, caplog):
    current_root = tmp_path / "PCLA"
    write_agents(current_root)
    legacy_root = tmp_path / "PCLA-wrapper" / "PCLA"
    resolved_root = PclaAV._resolve_pcla_root(
        legacy_root,
        legacy_root=legacy_root,
        image_root=current_root,
    )

    assert resolved_root == current_root
    assert "retired image path" in caplog.text


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
    response = adapter.init(request)
    assert isinstance(response, InitResponse)
    assert response.name == "carl_plant_3"
    assert response.metadata == {
        "effective_config": {
            "pcla_agent": "carl_plant_3",
            "route_waypoint_distance": 2.0,
            "route_draw": False,
            "launch_carla_server": False,
            "reuse_generated_world": True,
            "manage_traffic_manager_sync": False,
            "ego_role_name": "hero",
            "ego_bp_id": "vehicle.tesla.model3",
            "spawn_z_offset": 3.0,
            "coordinate_y_sign": -1.0,
            "yaw_sign": -1.0,
            "steer_sign": -1.0,
            "yaw_offset_deg": 0.0,
            "action_none_timeout_seconds": 0.0,
            "sensor_warmup_ticks": 1,
        }
    }
    struct = Struct()
    struct.update(response.metadata)
    assert MessageToDict(struct) == response.metadata
    assert adapter._initialized is True
    assert adapter._fixed_delta_seconds == pytest.approx(0.05)
    assert adapter._pcla is None
    assert adapter._pcla_module is None


def test_init_returns_canonical_selected_component_and_normalized_config(tmp_path):
    root = tmp_path / "PCLA"
    write_agents(root)
    adapter = PclaAV()
    adapter._ensure_connected = lambda: True
    adapter._prepare_reused_server_state = lambda: None

    response = adapter.init(
        SimpleNamespace(
            output_dir=tmp_path / "output",
            dt=0.05,
            config={
                "pcla_root": str(root),
                "pcla_agent": "carl_plant_9",
                "launch_carla_server": "false",
                "route_draw": "true",
                "coordinate_y_sign": -7,
                "yaw_sign": -2,
                "steer_sign": 3,
            },
        )
    )

    assert response.name == "carl_plant_9"
    effective = response.metadata["effective_config"]
    assert effective["launch_carla_server"] is False
    assert effective["route_draw"] is True
    assert effective["coordinate_y_sign"] == -1.0
    assert effective["yaw_sign"] == -1.0
    assert effective["steer_sign"] == 1.0


def test_generated_service_init_serializes_component_and_metadata(tmp_path):
    root = tmp_path / "PCLA"
    write_agents(root)
    adapter = PclaAV()
    adapter._ensure_connected = lambda: True
    adapter._prepare_reused_server_state = lambda: None
    service = GenericAvService(adapter, name="pcla-wrapper", version="0.3.2")
    context = SimpleNamespace(peer=lambda: "test")
    request = init_request_to_proto(
        InitRequest(
            output_dir=tmp_path / "output",
            dt=0.05,
            config={
                "pcla_root": str(root),
                "pcla_agent": "carl_plant_3",
                "launch_carla_server": False,
            },
        )
    )

    response = service.Init(request, context)

    assert response.name == "carl_plant_3"
    metadata = MessageToDict(response.metadata)
    assert metadata["effective_config"]["pcla_agent"] == "carl_plant_3"
    assert metadata["effective_config"]["launch_carla_server"] is False


def test_init_failure_does_not_produce_success_response(tmp_path):
    root = tmp_path / "PCLA"
    write_agents(root)
    adapter = PclaAV()
    adapter._ensure_connected = lambda: (_ for _ in ()).throw(AvTimeout("unavailable"))

    with pytest.raises(AvTimeout, match="unavailable"):
        adapter.init(
            SimpleNamespace(
                output_dir=tmp_path / "output",
                dt=0.05,
                config={
                    "pcla_root": str(root),
                    "pcla_agent": "carl_plant_3",
                    "launch_carla_server": False,
                },
            )
        )

    assert adapter._initialized is False


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
    with pytest.raises(AvTimeout, match="Last connection error: RuntimeError: offline"):
        adapter._ensure_connected()
    assert len(attempts) > 1


def test_connection_reports_owned_server_exit_during_retry(monkeypatch):
    adapter = PclaAV()
    adapter._server_version = None
    adapter._client = None
    adapter._server_owned = True
    adapter._server_process = FakeProcess(return_code=17)
    adapter._host = "127.0.0.1"
    adapter._port = 2000
    adapter._output_base = Path("/mnt/output")

    with pytest.raises(AvUnavailable) as exc_info:
        adapter._ensure_connected()

    message = str(exc_info.value)
    assert "return code 17" in message
    assert "127.0.0.1:2000" in message
    assert "stderr.log" in message
    assert "stdout.log" in message


def test_connection_timeout_reports_owned_server_still_running(monkeypatch):
    adapter = PclaAV()
    adapter._server_version = None
    adapter._client = None
    adapter._server_owned = True
    adapter._server_process = FakeProcess(return_code=None)
    adapter._host = "127.0.0.1"
    adapter._port = 2000
    adapter._connect_timeout = 0.001
    adapter._retry_interval = 0.001

    def fail():
        raise RuntimeError("rpc silent")

    monkeypatch.setattr(adapter, "_connect_once", fail)
    with pytest.raises(AvTimeout) as exc_info:
        adapter._ensure_connected()

    message = str(exc_info.value)
    assert "owned CARLA server process is still running" in message
    assert "127.0.0.1:2000" in message
    assert "RuntimeError: rpc silent" in message


def test_connection_timeout_reports_external_server_health_unavailable(monkeypatch):
    adapter = PclaAV()
    adapter._server_version = None
    adapter._client = None
    adapter._launch_carla_server = False
    adapter._server_owned = False
    adapter._server_process = None
    adapter._host = "carla.example"
    adapter._port = 2000
    adapter._connect_timeout = 0.001
    adapter._retry_interval = 0.001

    def fail():
        raise RuntimeError("connection refused")

    monkeypatch.setattr(adapter, "_connect_once", fail)
    with pytest.raises(AvTimeout) as exc_info:
        adapter._ensure_connected()

    message = str(exc_info.value)
    assert "external CARLA server health cannot be inferred" in message
    assert "carla.example:2000" in message
    assert "RuntimeError: connection refused" in message
    assert "stderr.log" not in message


@pytest.mark.parametrize(
    "scenario,observation,message",
    [
        (None, observation(), "ScenarioPack"),
        (SimpleNamespace(map_name="", ego=object()), observation(), "map_name"),
        (
            SimpleNamespace(
                map_name="Town",
                ego=SimpleNamespace(goal_config=SimpleNamespace(position=None)),
            ),
            observation(),
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
        adapter._resolve_route_path(scenario, observation())

    route = tmp_path / "route.xml"
    route.write_text("<route/>", encoding="utf-8")
    adapter._route_path_cfg = str(route)
    assert adapter._resolve_route_path(scenario, observation()) == route


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
    route = adapter._resolve_route_path(
        scenario,
        observation(object_state(kinematic=kinematic(x=2, y=4, z=1))),
    )
    assert route.parent == tmp_path / "case-a" / "pcla_routes"
    assert ".." not in route.name
    assert route.read_text(encoding="utf-8") == "<route/>"

    adapter._pcla_module.location_to_waypoint = lambda *args, **kwargs: []
    with pytest.raises(AvPreconditionFailed, match="fewer than two"):
        adapter._resolve_route_path(scenario, observation())


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
    adapter._apply_state(actor, object_state(kinematic=kinematic(yaw_rate=1.0)))
    assert actor.angular_velocity.z == pytest.approx(-57.2957795)

    adapter._pcla = SimpleNamespace(
        get_action=lambda snapshot=None: SimpleNamespace(throttle=0.2, brake=0.1, steer=0.4)
    )
    adapter._data_provider = None
    adapter._vehicle = actor
    response = adapter.step(SimpleNamespace(observation=observation(), timestamp_ns=0))
    assert response.ctrl_cmd.payload["steer"] == pytest.approx(-0.4)


def test_extract_xyz_rejects_missing_or_non_numeric_fields():
    adapter = PclaAV()
    with pytest.raises(InvalidAvRequest, match="missing"):
        adapter._extract_xyz(SimpleNamespace(x=1, y=2))
    with pytest.raises(InvalidAvRequest, match="numeric"):
        adapter._extract_xyz(SimpleNamespace(x="bad", y=2, z=3))


def test_tracking_ids_preserve_identity_across_reorder_and_remove_disappeared():
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
    first = observed_agent(object_state(1), tracking_id=10)
    second = observed_agent(object_state(2), tracking_id=20)

    adapter._update_and_tick(observation(ego, [first, second]))
    actor = adapter._other_actors_by_key[10]
    second_actor = adapter._other_actors_by_key[20]
    adapter._update_and_tick(observation(ego, [second, first]))
    assert adapter._other_actors_by_key[10] is actor
    assert adapter._other_actors_by_key[20] is second_actor
    assert actor.physics_calls[-1] is True
    assert actor.gravity_calls[-1] is True
    assert adapter._vehicle.physics_calls == [True, True]
    assert adapter._vehicle.gravity_calls == [True, True]

    changed = observed_agent(object_state(1, RoadObjectType.PEDESTRIAN), tracking_id=10)
    adapter._update_and_tick(observation(ego, [changed]))
    replacement = adapter._other_actors_by_key[10]
    assert replacement is not actor
    assert actor.destroy_calls == 1
    assert second_actor.destroy_calls == 1
    adapter._update_and_tick(observation(ego))
    assert replacement.destroy_calls == 1


def test_missing_or_mixed_tracking_ids_are_stateless_and_ignore_entity_name():
    adapter = configured_adapter()
    ego = object_state()
    unnamed = observed_agent(object_state(1))
    named = observed_agent(object_state(2), tracking_id=7, entity_name="NPC")
    adapter._update_and_tick(observation(ego, [unnamed, named]))
    first_actors = list(adapter._stateless_other_actors)
    adapter._update_and_tick(observation(ego, [named, unnamed]))
    assert adapter._other_actors_by_key == {}
    assert len(adapter._stateless_other_actors) == 2
    assert all(actor.destroy_calls == 1 for actor in first_actors)


def test_duplicate_tracking_ids_are_rejected():
    adapter = configured_adapter()
    agents = [
        observed_agent(object_state(1), tracking_id=7),
        observed_agent(object_state(2), tracking_id=7),
    ]
    with pytest.raises(InvalidAvRequest, match="duplicate"):
        adapter._update_and_tick(observation(agents=agents))


def test_agent_order_does_not_change_ego_pose():
    adapter = configured_adapter()
    agents = [
        observed_agent(object_state(1), tracking_id=1),
        observed_agent(object_state(2), tracking_id=2),
    ]
    ego = object_state(50)
    adapter._update_and_tick(observation(ego, agents))
    adapter._update_and_tick(observation(ego, reversed(agents)))
    assert [transform.location.x for transform in adapter._vehicle.transforms] == [50.0, 50.0]


def test_shadow_ego_keeps_physics_enabled_for_velocity_observation():
    adapter = configured_adapter()
    ego = object_state(37)

    adapter._update_and_tick(observation(ego))

    assert adapter._world.tick_calls == 1
    assert [transform.location.x for transform in adapter._vehicle.transforms] == [37.0]
    assert adapter._vehicle.physics_calls == [True]
    assert adapter._vehicle.gravity_calls == [True]


def test_step_applies_output_control_to_shadow_ego_for_next_observation():
    adapter = configured_adapter()
    action = SimpleNamespace(throttle=0.7, brake=0.2, steer=0.3, gear=1)
    adapter._pcla = SimpleNamespace(get_action=lambda snapshot=None: action)
    adapter._data_provider = None

    response = adapter.step(SimpleNamespace(observation=observation(), timestamp_ns=0))

    assert response.ctrl_cmd.payload == {
        "throttle": 0.0,
        "brake": 0.2,
        "steer": 0.3,
    }
    assert adapter._vehicle.get_control() is action
    assert action.throttle == 0.0
    assert action.brake == 0.2
    assert action.steer == 0.3


def test_shape_center_and_actor_origin_offsets_are_composed():
    adapter = configured_adapter()
    shape = ShapeData(
        type=ShapeType.BOUNDING_BOX,
        dimensions=ShapeDimensionData(x=4.0, y=2.0, z=1.5),
        center=ShapeCenterPoseData(x=2.0, y=1.0, z=0.5, yaw=math.pi / 2),
        reference_point="rear_axle",
    )
    state = object_state(10, shape=shape)
    agent = observed_agent(state, tracking_id=9)
    adapter._update_and_tick(observation(agents=[agent]))
    actor = adapter._other_actors_by_key[9]
    actor.bounding_box.location.x = 0.5
    adapter._update_and_tick(observation(agents=[agent]))
    transform = actor.transforms[-1]
    assert transform.location.x == pytest.approx(12.0)
    assert transform.location.y == pytest.approx(0.5)
    assert transform.location.z == pytest.approx(0.5)
    assert transform.rotation.yaw == pytest.approx(90.0)


def test_cleanup_clears_tracking_state():
    adapter = configured_adapter()
    adapter._update_and_tick(observation(agents=[observed_agent(object_state(1), tracking_id=4)]))
    adapter._cleanup_wrapper_actors()
    assert adapter._other_actors_by_key == {}
    assert adapter._stateless_other_actors == []
    assert adapter._using_tracking_ids is False


def test_contract_timestamp_grid_and_normalized_snapshot():
    adapter = configured_adapter()
    snapshots = []
    adapter._pcla = SimpleNamespace(
        get_action=lambda snapshot=None: (
            snapshots.append(snapshot) or SimpleNamespace(throttle=0.1, brake=0.0, steer=0.0)
        )
    )
    adapter._data_provider = None

    adapter.step(SimpleNamespace(observation=observation(), timestamp_ns=0))
    next_ego = object_state(kinematic=kinematic(time_ns=50_000_000))
    adapter.step(SimpleNamespace(observation=observation(next_ego), timestamp_ns=50_000_000))

    assert snapshots[0].timestamp.elapsed_seconds == 0.0
    assert snapshots[0].timestamp.delta_seconds == 0.0
    assert snapshots[1].timestamp.elapsed_seconds == pytest.approx(0.05)
    assert snapshots[1].timestamp.delta_seconds == pytest.approx(0.05)
    with pytest.raises(InvalidAvRequest, match="must be 100000000"):
        adapter.step(SimpleNamespace(observation=observation(), timestamp_ns=50_000_000))


def test_reset_pcla_game_time_clears_reused_world_clock():
    adapter = configured_adapter()
    game_time = SimpleNamespace(
        _current_game_time=12.0,
        _carla_time=100.0,
        _last_frame=500,
        _platform_timestamp=object(),
        _init=True,
    )
    adapter._pcla_module = SimpleNamespace(GameTime=game_time)
    adapter._reset_pcla_game_time()
    assert (
        game_time._current_game_time,
        game_time._carla_time,
        game_time._last_frame,
        game_time._platform_timestamp,
        game_time._init,
    ) == (0.0, 0.0, 0, 0, False)


def test_contract_rejects_nonfinite_unsupported_and_invalid_shape():
    adapter = configured_adapter()
    with pytest.raises(InvalidAvRequest, match="finite"):
        adapter._prepare_observation(observation(object_state(kinematic=kinematic(x=math.nan))), 0)

    invalid_dimensions = box_shape(dimensions=(0.0, 2.0, 1.5))
    with pytest.raises(InvalidAvRequest, match="positive"):
        adapter._prepare_observation(observation(object_state(shape=invalid_dimensions)), 0)

    unsupported = ShapeData(
        type=ShapeType.CYLINDER,
        dimensions=ShapeDimensionData(x=1.0, y=1.0, z=2.0),
    )
    with pytest.raises(InvalidAvRequest, match="unsupported"):
        adapter._prepare_observation(observation(object_state(shape=unsupported)), 0)


def test_tracking_shape_cache_reuses_omission_and_rejects_mutation():
    adapter = configured_adapter()
    first_shape = box_shape()
    first = observed_agent(object_state(1, shape=first_shape), tracking_id=8)
    prepared = adapter._prepare_observation(observation(agents=[first]), 0)
    assert prepared.agents[0].state.shape is first_shape

    omitted = observed_agent(object_state(2), tracking_id=8)
    prepared = adapter._prepare_observation(observation(agents=[omitted]), 0)
    assert prepared.agents[0].state.shape is first_shape

    changed = observed_agent(
        object_state(2, shape=box_shape(dimensions=(5.0, 2.0, 1.5))), tracking_id=8
    )
    with pytest.raises(InvalidAvRequest, match="changed shape"):
        adapter._prepare_observation(observation(agents=[changed]), 0)


def test_ego_shape_cache_reuses_omission_and_rejects_reference_change():
    adapter = configured_adapter()
    initial_shape = box_shape()
    adapter._prepare_observation(observation(object_state(shape=initial_shape)), 0)

    prepared = adapter._prepare_observation(observation(), 0)
    assert prepared.ego.shape is initial_shape

    changed_reference = ShapeData(
        type=initial_shape.type,
        dimensions=initial_shape.dimensions,
        center=initial_shape.center,
        reference_point="different_origin",
    )
    with pytest.raises(InvalidAvRequest, match="ego changed shape"):
        adapter._prepare_observation(observation(object_state(shape=changed_reference)), 0)


def test_blueprint_matching_uses_nearest_dimensions_and_warns_once(caplog):
    near = FakeBlueprint("vehicle.near", dimensions=(4.1, 2.0, 1.5))
    far = FakeBlueprint("vehicle.far", dimensions=(8.0, 3.0, 3.0))
    world = FakeWorld(FakeBlueprintLibrary(patterns={"vehicle.*": [far, near]}))
    adapter = configured_adapter(world)
    state = object_state(shape=box_shape(dimensions=(4.0, 2.0, 1.5)))

    with caplog.at_level("WARNING", logger="pcla_wrapper.pcla_av"):
        selected = adapter._pick_blueprint_for_state(state)
        selected_again = adapter._pick_blueprint_for_state(state)

    assert selected.id == "vehicle.near"
    assert selected_again.id == "vehicle.near"
    assert caplog.text.count("nearest CARLA geometry") == 1
    assert all(not actor.is_alive for actor in world.actors)


def test_relative_rotation_uses_full_handedness_basis_conversion():
    adapter = configured_adapter()
    adapter._coordinate_y_sign = -1.0
    adapter._yaw_sign = -1.0
    center = ShapeCenterPoseData(roll=0.2, pitch=0.1, yaw=0.3)
    converted = adapter._relative_rotation_to_carla(center)
    roll, pitch, yaw = adapter._matrix_to_rotation(converted)
    assert roll == pytest.approx(math.degrees(-0.2))
    assert pitch == pytest.approx(math.degrees(0.1))
    assert yaw == pytest.approx(math.degrees(-0.3))


def test_control_contract_enforces_ranges_brake_priority_and_sign():
    adapter = configured_adapter()
    adapter._steer_sign = -1.0
    assert adapter._normalize_control(SimpleNamespace(throttle=0.8, brake=0.2, steer=0.4)) == {
        "throttle": 0.0,
        "brake": 0.2,
        "steer": -0.4,
    }
    with pytest.raises(AvUnavailable, match="missing"):
        adapter._normalize_control(SimpleNamespace(throttle=0.0, brake=0.0))
    with pytest.raises(AvUnavailable, match="finite"):
        adapter._normalize_control(SimpleNamespace(throttle=math.nan, brake=0.0, steer=0.0))
    with pytest.raises(AvUnavailable, match="canonical range"):
        adapter._normalize_control(SimpleNamespace(throttle=1.01, brake=0.0, steer=0.0))
    with pytest.raises(AvUnavailable, match="canonical range"):
        adapter._normalize_control(SimpleNamespace(throttle=0.0, brake=0.0, steer=-1.01))


def test_spawn_overlap_retries_and_teleports_to_observation():
    adapter = configured_adapter()
    actor = FakeActor(9)
    adapter._world.spawn_results = [None, actor]
    blueprint = FakeBlueprint()
    transform = FakeTransform(FakeLocation(1, 2, 3), FakeRotation())
    result = adapter._spawn_actor_allowing_observation_overlap(blueprint, transform)
    assert result is actor
    adapter._apply_state(result, object_state(kinematic=kinematic(x=1, y=2, z=3)), kinematic=True)
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
    response = adapter.step(SimpleNamespace(observation=observation(), timestamp_ns=0))
    assert isinstance(response, StepResponse)
    assert response.ctrl_cmd.mode == ControlMode.THROTTLE_STEER_BREAK
    assert adapter._world.tick_calls == 1
    assert events == [
        "tick",
        "snapshot",
        "provider",
        ("action", events[-1][1]),
    ]
    normalized_snapshot = events[-1][1]
    assert normalized_snapshot.native_snapshot is adapter._world.snapshot
    assert normalized_snapshot.timestamp.elapsed_seconds == 0.0
    assert normalized_snapshot.timestamp.delta_seconds == 0.0
    assert adapter._last_timestamp_ns == 0


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
        adapter.step(SimpleNamespace(observation=observation(), timestamp_ns=0))
    assert adapter.should_quit().should_quit is True

    def fail(snapshot=None):
        raise RuntimeError("model crashed")

    adapter._pcla = SimpleNamespace(get_action=fail)
    with pytest.raises(AvUnavailable, match="model crashed"):
        adapter.step(SimpleNamespace(observation=observation(), timestamp_ns=0))
    assert "model crashed" in adapter.should_quit().msg


def test_none_action_can_use_timeout_policy():
    adapter = configured_adapter()
    adapter._data_provider = None
    adapter._action_none_timeout = 0.001
    adapter._pcla = SimpleNamespace(get_action=lambda snapshot=None: None)
    with pytest.raises(AvTimeout, match="no action"):
        adapter.step(SimpleNamespace(observation=observation(), timestamp_ns=0))


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
        initial_observation=observation(),
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

    path = Path("PCLA/PCLA.py").resolve()
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
