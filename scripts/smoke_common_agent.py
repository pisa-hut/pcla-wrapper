#!/usr/bin/env python3
from __future__ import annotations

import argparse
import importlib.util
import os
import sys
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("agent")
    parser.add_argument("--pcla-root", type=Path, default=Path("/app/PCLA"))
    args = parser.parse_args()

    os.environ.setdefault("PCLA_IMAGE_PROFILE", "common")
    os.environ.setdefault("PCLA_PRETRAINED_ROOT", "/mnt/weights")
    os.environ.setdefault("CUBLAS_WORKSPACE_CONFIG", ":4096:8")

    pcla_root = args.pcla_root.resolve()
    sys.path.insert(0, str(pcla_root))

    import PCLA as pcla_module
    from pcla_wrapper.profiles import isolate_profile_import_paths, validate_image_profile
    from pcla_wrapper.weights import resolve_give_path_result

    isolate_profile_import_paths(pcla_root)
    validate_image_profile(args.agent, Path(os.environ["PCLA_PRETRAINED_ROOT"]))
    agent_path, config_path = pcla_module.give_path(args.agent, str(pcla_root), "")
    agent_path, config_path = resolve_give_path_result(
        args.agent,
        pcla_root,
        Path(os.environ["PCLA_PRETRAINED_ROOT"]),
        agent_path,
        config_path,
    )
    module_dir = str(Path(agent_path).parent)
    sys.path.insert(0, module_dir)

    spec = importlib.util.spec_from_file_location("pcla_smoke_agent", agent_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load agent module: {agent_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    agent_class = getattr(module, module.get_entry_point())

    if args.agent.startswith("plant2_"):
        checkpoint = Path(os.environ["PLANT_CHECKPOINT"])
        model = module.LitHFLM.load_from_checkpoint(checkpoint, map_location="cpu")
        model.eval()
        print(f"loaded agent={args.agent} class={agent_class.__name__} checkpoint={checkpoint}")
        return 0

    instance = agent_class(config_path)
    sensors = instance.sensors()
    print(
        f"loaded agent={args.agent} class={agent_class.__name__} "
        f"sensors={len(sensors)} config={config_path}"
    )
    destroy = getattr(instance, "destroy", None)
    if callable(destroy):
        destroy()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
