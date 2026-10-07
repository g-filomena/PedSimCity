#!/usr/bin/env python
"""How a city's layers were produced: ``src/main/resources/<City>/<City>_provenance.json``.

Each stage of ``00_city_preparation.py`` records, when it computes its checkpoints, what computed
them (``inputData/<City>/prep_staging/stage_provenance.json``): stages are checkpointed and a later
run reuses them, so a city's layers can come from different days and different cityImage versions.
``finalize`` gathers those records with the command line, the saved options, the inputs and the
outputs (size, sha256, feature count) into the city's provenance file. ``build_lighting.py`` adds a
``lighting`` section to the same file, and a raw input with a ``<name>.provenance.json`` beside it
(``os_mastermap_buildings.py`` writes one) is carried into it.

For layers built before this existed, the stages can be read back from the run's log:

  python pipeline/provenance.py --city Torino --from-log ~/runs/prep_torino_220.log

Give several logs, oldest first, when the layers came from several runs: a stage takes the record
of the last run that wrote its checkpoints.
"""
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import platform
import re
import subprocess
import sys
from importlib import metadata
from pathlib import Path

import paths

STAGE_RECORDS = "stage_provenance.json"
PACKAGES = ("cityImage", "osmnx", "geopandas", "shapely", "pyogrio", "networkx", "numpy",
            "pandas", "rasterio")


def now() -> str:
    return dt.datetime.now().astimezone().isoformat(timespec="seconds")


def _git(directory: Path) -> dict | None:
    """Commit and local changes of the git checkout holding ``directory``, if any."""
    try:
        commit = subprocess.run(["git", "-C", str(directory), "rev-parse", "HEAD"],
                                capture_output=True, text=True, check=True).stdout.strip()
        status = subprocess.run(["git", "-C", str(directory), "status", "--porcelain",
                                 "--untracked-files=no"],
                                capture_output=True, text=True, check=True).stdout
    except (OSError, subprocess.CalledProcessError):
        return None
    # Each line is two status characters, a space and the path; the first may be a space.
    return {"commit": commit, "local_changes": [line[3:] for line in status.splitlines() if line]}


def environment() -> dict:
    """Python, the packages that shape the layers, and the code that ran them."""
    packages = {}
    for name in PACKAGES:
        try:
            packages[name] = metadata.version(name)
        except metadata.PackageNotFoundError:
            continue
    env = {"python": platform.python_version(), "host": platform.node(), "packages": packages,
           "pedsimcity": _git(Path(__file__).resolve().parent)}
    try:
        import cityImage
        location = Path(cityImage.__file__).resolve().parent
        env["cityImage"] = {"path": str(location), "git": _git(location)}
    except ImportError:
        pass
    return env


def file_info(path: Path) -> dict:
    """Name, size, sha256 and, for a GeoPackage, the feature count of each layer."""
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    info = {"file": path.name, "bytes": path.stat().st_size, "sha256": digest.hexdigest(),
            "modified": dt.datetime.fromtimestamp(path.stat().st_mtime).astimezone()
            .isoformat(timespec="seconds")}
    if path.suffix == ".gpkg":
        try:
            import pyogrio
            info["features"] = {layer: pyogrio.read_info(path, layer=layer)["features"]
                                for layer, _ in pyogrio.list_layers(path)}
        except Exception:  # an unreadable layer is reported by the readers, not here
            pass
    sidecar = path.with_name(path.name + ".provenance.json")
    if sidecar.exists():
        info["provenance"] = json.loads(sidecar.read_text(encoding="utf-8"))
    return info


def settings(args) -> dict:
    """The run's arguments that can be written as JSON (the query geometry is not)."""
    keep = {}
    for key, value in vars(args).items():
        if isinstance(value, (str, int, float, bool)) or value is None:
            keep[key] = value
        elif isinstance(value, (list, tuple)) and all(isinstance(v, str) for v in value):
            keep[key] = list(value)
        elif isinstance(value, Path):
            keep[key] = str(value)
    return keep


def _records_path(staging_dir: Path) -> Path:
    return staging_dir / STAGE_RECORDS


def load_records(staging_dir: Path) -> dict:
    path = _records_path(staging_dir)
    return json.loads(path.read_text(encoding="utf-8")) if path.exists() else {}


def record_stage(staging_dir: Path, stage: str, args, checkpoints: list[str]) -> None:
    """Records that ``stage`` computed its checkpoints now, with this environment and arguments."""
    records = load_records(staging_dir)
    records[stage] = {"computed": now(), "checkpoints": checkpoints,
                      "environment": environment(), "settings": settings(args)}
    _records_path(staging_dir).write_text(json.dumps(records, indent=2), encoding="utf-8")


def forget_stages(staging_dir: Path, stages) -> None:
    """Drops the records of stages whose checkpoints were removed."""
    records = load_records(staging_dir)
    if any(records.pop(stage, None) is not None for stage in stages):
        _records_path(staging_dir).write_text(json.dumps(records, indent=2), encoding="utf-8")


def output_path(city: str) -> Path:
    return paths.resources_dir(city) / f"{city}_provenance.json"


def update(city: str, section: str, content: dict) -> Path:
    """Writes one section of the city's provenance file, keeping the others."""
    path = output_path(city)
    document = json.loads(path.read_text(encoding="utf-8")) if path.exists() else {"city": city}
    document[section] = content
    document["updated"] = now()
    path.write_text(json.dumps(document, indent=2), encoding="utf-8")
    return path


def write_preparation(args, staging_dir: Path, inputs: list[Path], outputs: list[Path],
                      this_run: set[str]) -> Path:
    """The ``preparation`` section: what 00_city_preparation.py produced, and from what."""
    records = load_records(staging_dir)
    stages = {stage: dict(record, reused=stage not in this_run)
              for stage, record in records.items()}
    content = {
        "written": now(),
        "command": sys.argv,
        "settings": settings(args),
        "environment": environment(),
        "stages": stages,
        "inputs": [file_info(p) for p in inputs if p is not None and p.exists()],
        "outputs": [file_info(p) for p in outputs if p.exists()],
    }
    return update(args.city_name, "preparation", content)


_STAGE = re.compile(r"^(\S+ \S+)\s+INFO\s+=+ stage: (\w+) =+")
_SAVED = re.compile(r"checkpoint saved: (\w+)\.gpkg")
_OPTIONS = re.compile(r"(?:saved|network) options: (.*)$")
_CITYIMAGE = re.compile(r"^cityImage (\S+) (\S+)")


def _read_log(log_file: Path) -> dict:
    """The stages one run's log says wrote checkpoints, with the options and cityImage it printed."""
    stages, current, options, city_image = {}, None, None, None
    for line in log_file.read_text(encoding="utf-8", errors="replace").splitlines():
        if match := _CITYIMAGE.match(line):
            city_image = {"version": match.group(1), "path": match.group(2)}
        if match := _OPTIONS.search(line):
            options = match.group(1)
        if match := _STAGE.match(line):
            current = match.group(2)
            stages.setdefault(current, {"started": match.group(1), "checkpoints": []})
        elif current and (match := _SAVED.search(line)):
            stages[current]["checkpoints"].append(match.group(1))
            stages[current]["computed"] = line.split("  ")[0]
    run = {"log": str(log_file), "options": options, "cityImage": city_image}
    return {stage: dict(record, run=run) for stage, record in stages.items()
            if record["checkpoints"]}


def from_log(city: str, log_files: list[Path]) -> Path:
    """The ``preparation`` section of layers built before stages recorded themselves, read back
    from the logs of the runs that built them, oldest first."""
    stages = {}
    for log_file in log_files:
        stages.update(_read_log(log_file))
    resources = paths.resources_dir(city)
    content = {
        "written": now(),
        "source": "read back from run logs: " + ", ".join(str(p) for p in log_files),
        "stages": stages,
        "recorded_with": environment(),
        "outputs": [file_info(p) for p in sorted(resources.glob(f"{city}_*.gpkg"))],
    }
    return update(city, "preparation", content)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--city", required=True)
    parser.add_argument("--from-log", type=Path, required=True, nargs="+",
                        help="the 00_city_preparation.py logs of the runs that built the layers, "
                             "oldest first")
    args = parser.parse_args(argv)
    print(from_log(args.city, args.from_log))
    return 0


if __name__ == "__main__":
    sys.exit(main())
