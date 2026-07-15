from __future__ import annotations

import json
import os
import sys
from pathlib import Path
from typing import Any

from pisa_api.av import InvalidAvRequest


def isolate_profile_import_paths(pcla_root: Path) -> None:
    """Remove private dependency paths for agents excluded from the active image profile."""
    if os.environ.get("PCLA_IMAGE_PROFILE") != "common":
        return

    lmdrive_path = (pcla_root / "pcla_agents" / "lmdrive" / "vision_encoder").resolve()
    sys.path[:] = [entry for entry in sys.path if Path(entry or ".").resolve() != lmdrive_path]

    # Also recover cleanly if a previous agent import failed partway through
    # LMDrive's private timm package before profile isolation ran.
    for name, module in list(sys.modules.items()):
        module_file = getattr(module, "__file__", None)
        if module_file is None:
            continue
        try:
            Path(module_file).resolve().relative_to(lmdrive_path)
        except ValueError:
            continue
        del sys.modules[name]


def load_agent_profiles() -> dict[str, Any]:
    profile_path = Path(__file__).with_name("agent_profiles.json")
    return json.loads(profile_path.read_text(encoding="utf-8"))


def validate_image_profile(agent_name: str, pretrained_root: Path) -> None:
    profile_name = os.environ.get("PCLA_IMAGE_PROFILE")
    if not profile_name:
        return

    profiles = load_agent_profiles()
    profile = profiles.get(profile_name)
    if profile is None:
        raise InvalidAvRequest(f"Unknown PCLA image profile: {profile_name!r}")

    required_paths = profile["agents"].get(agent_name)
    if required_paths is None:
        supported = ", ".join(sorted(profile["agents"]))
        raise InvalidAvRequest(
            f"PCLA agent {agent_name!r} is not supported by image profile "
            f"{profile_name!r}. Supported agents: {supported}"
        )

    missing = []
    for path in required_paths:
        relative_path = Path(path)
        expected_path = pretrained_root / relative_path
        direct_relative_path = (
            Path(*relative_path.parts[1:])
            if relative_path.parts and relative_path.parts[0].endswith("_pretrained")
            else relative_path
        )
        direct_path = pretrained_root / direct_relative_path
        if not expected_path.is_file() and not direct_path.is_file():
            missing.append(expected_path)
    if missing:
        formatted = ", ".join(str(path) for path in missing)
        raise InvalidAvRequest(
            f"PCLA agent {agent_name!r} weights are unavailable for image profile "
            f"{profile_name!r}: {formatted}. Mount the selected weight directory at "
            f"{pretrained_root}."
        )
