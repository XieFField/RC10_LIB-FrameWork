from __future__ import annotations

import argparse
import sys
from pathlib import Path


def tests_root() -> Path:
    return Path(__file__).resolve().parents[1]


def repo_root() -> Path:
    return tests_root().parents[1]


def artifacts_root(*parts: str) -> Path:
    path = tests_root() / "artifacts"
    for part in parts:
        path /= part
    path.mkdir(parents=True, exist_ok=True)
    return path


def make_mode_parser(*choices: str, default: str) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=choices, default=default)
    return parser


def ensure_repo_on_syspath() -> None:
    root = str(repo_root())
    if root not in sys.path:
        sys.path.insert(0, root)
