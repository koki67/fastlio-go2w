#!/usr/bin/env python3
"""Resolve the sensor calibration used to display a saved FAST-LIO run."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence


REPO_ROOT = Path(__file__).resolve().parents[2]


class CalibrationResolutionError(ValueError):
    """Raised when a run cannot be associated with a sensor calibration."""


@dataclass(frozen=True)
class CalibrationResolution:
    path: Path
    source: str
    sensor: str


def _mapping(value: Any) -> Mapping[str, Any]:
    return value if isinstance(value, Mapping) else {}


def _infer_sensor(manifest: Mapping[str, Any]) -> str:
    sensor = str(manifest.get("sensor") or "").strip()
    if sensor in {"mid360", "xt16"}:
        return sensor

    playback = _mapping(manifest.get("playback"))
    topics = set(playback.get("topics") or [])
    if {"/points_raw", "/go2w/imu"}.issubset(topics):
        return "xt16"
    if {"/livox/lidar", "/livox/imu"}.issubset(topics):
        return "mid360"
    return ""


def resolve_run_calibration(
    run_dir: Path | str,
    *,
    repo_root: Path | str = REPO_ROOT,
) -> CalibrationResolution:
    run_path = Path(run_dir).expanduser().resolve()
    snapshot = run_path / "sensor_calibration.yaml"
    if snapshot.is_file():
        return CalibrationResolution(snapshot, "run snapshot", "")

    manifest_path = run_path / "manifest.json"
    try:
        with manifest_path.open(encoding="utf-8") as stream:
            manifest = json.load(stream)
    except FileNotFoundError as error:
        raise CalibrationResolutionError(
            f"run calibration and manifest are both missing: {run_path}"
        ) from error
    except (json.JSONDecodeError, OSError) as error:
        raise CalibrationResolutionError(
            f"could not read run manifest: {manifest_path}: {error}"
        ) from error

    sensor = _infer_sensor(_mapping(manifest))
    if not sensor:
        raise CalibrationResolutionError(
            f"could not infer sensor calibration for legacy run: {run_path}"
        )

    calibration = (
        Path(repo_root).expanduser().resolve()
        / "config"
        / "sensor"
        / f"go2w_{sensor}_calibration.yaml"
    )
    if not calibration.is_file():
        raise CalibrationResolutionError(
            f"sensor calibration not found: {calibration}"
        )
    return CalibrationResolution(
        calibration,
        f"repository {sensor} fallback",
        sensor,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        resolution = resolve_run_calibration(args.run_dir)
    except CalibrationResolutionError as error:
        print(f"Error: {error}", file=sys.stderr)
        return 2
    print(f"{resolution.source}\t{resolution.path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
