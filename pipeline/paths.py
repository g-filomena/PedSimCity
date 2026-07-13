"""Folder conventions shared by every pipeline script.

The pipeline reads raw, user-supplied material from ``inputData/<City>/`` and writes
everything the Java simulation loads to ``src/main/resources/<City>/``. Intermediates
(step outputs consumed only by later steps, staging checkpoints, the per-city prep
config) also live in ``inputData/<City>/``, so the resources folder holds nothing but
simulation inputs.

When resolving an input, scripts search ``inputData/<City>/`` first and the resources
folder second — so a file produced by an earlier step is found wherever it lives.
"""

from __future__ import annotations

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]


def raw_dir(city: str) -> Path:
    """``inputData/<City>/`` — raw material and pipeline intermediates (created on demand)."""
    folder = REPO_ROOT / "inputData" / city
    folder.mkdir(parents=True, exist_ok=True)
    return folder


def resources_dir(city: str) -> Path:
    """``src/main/resources/<City>/`` — the files the Java simulation reads (created on demand)."""
    folder = REPO_ROOT / "src" / "main" / "resources" / city
    folder.mkdir(parents=True, exist_ok=True)
    return folder


def find_input(city: str, filename_suffix: str) -> Path | None:
    """First existing ``<City>_<filename_suffix>``, searching inputData/ then resources/."""
    for folder in (raw_dir(city), resources_dir(city)):
        candidate = folder / f"{city}_{filename_suffix}"
        if candidate.exists():
            return candidate
    return None


def require_input(city: str, filename_suffix: str, label: str) -> Path:
    """Like :func:`find_input`, but raises a clear error listing the searched locations."""
    found = find_input(city, filename_suffix)
    if found is None:
        raise FileNotFoundError(
            f"Could not find {label} ({city}_{filename_suffix}). Searched:\n"
            f"  {raw_dir(city)}\n  {resources_dir(city)}"
        )
    return found
