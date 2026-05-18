from __future__ import annotations

import os
from pathlib import Path


def find_workspace_root(start: str | Path | None = None) -> Path:
    env_root = os.getenv("RHW_MODEL_SCHEDULER_ROOT", "").strip()
    if env_root:
        return Path(env_root).expanduser().resolve()

    cursor = Path(start or __file__).resolve()
    if cursor.is_file():
        cursor = cursor.parent
    for parent in (cursor, *cursor.parents):
        if (parent / "models" / "current" / "manifest.json").is_file():
            return parent
    return Path(__file__).resolve().parents[3]
