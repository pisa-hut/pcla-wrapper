from __future__ import annotations

import os
from pathlib import Path
from typing import Any


def resolve_weight_path(path: str, pcla_root: Path, pretrained_root: Path) -> str:
    """Resolve a PCLA registry path for root and selected-directory weight mounts."""
    candidate = Path(path)
    try:
        relative = candidate.relative_to(pcla_root / "pcla_agents")
    except ValueError:
        return path

    if not relative.parts or not relative.parts[0].endswith("_pretrained"):
        return path

    nested = pretrained_root / relative
    direct = pretrained_root.joinpath(*relative.parts[1:])
    for resolved in (nested, direct):
        if resolved.exists():
            return str(resolved)
    return path


def resolve_give_path_result(
    name: str,
    pcla_root: Path,
    pretrained_root: Path,
    agent_path: str,
    config_path: str,
) -> tuple[str, str]:
    config_path = resolve_weight_path(config_path, pcla_root, pretrained_root)
    if name.startswith("plant2_"):
        os.environ["PLANT_CHECKPOINT"] = config_path
    return agent_path, config_path


def install_give_path_resolver(module: Any, pretrained_root: Path) -> None:
    """Wrap PCLA.py's imported give_path without writing into the image filesystem."""
    original = module.give_path
    if getattr(original, "_pcla_weight_resolver", False):
        return

    def resolved_give_path(name: str, pcla_dir: str, route_path: str):
        agent_path, config_path = original(name, pcla_dir, route_path)
        return resolve_give_path_result(
            name,
            Path(pcla_dir),
            pretrained_root,
            agent_path,
            config_path,
        )

    resolved_give_path._pcla_weight_resolver = True  # type: ignore[attr-defined]
    module.give_path = resolved_give_path
