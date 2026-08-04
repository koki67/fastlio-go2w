#!/usr/bin/env python3

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from resolve_fastlio_run_calibration import (  # noqa: E402
    CalibrationResolutionError,
    resolve_run_calibration,
)


def _repo_root(tmp_path: Path) -> Path:
    root = tmp_path / "repo"
    sensor_dir = root / "config" / "sensor"
    sensor_dir.mkdir(parents=True)
    for sensor in ("mid360", "xt16"):
        (sensor_dir / f"go2w_{sensor}_calibration.yaml").write_text(
            f"sensor: {sensor}\n",
            encoding="utf-8",
        )
    return root


def _write_manifest(run_dir: Path, manifest: dict) -> None:
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "manifest.json").write_text(
        json.dumps(manifest),
        encoding="utf-8",
    )


def test_saved_run_snapshot_takes_precedence(tmp_path):
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    snapshot = run_dir / "sensor_calibration.yaml"
    snapshot.write_text("sensor: xt16\n", encoding="utf-8")

    resolution = resolve_run_calibration(run_dir, repo_root=tmp_path / "missing")

    assert resolution.path == snapshot.resolve()
    assert resolution.source == "run snapshot"


@pytest.mark.parametrize("sensor", ["mid360", "xt16"])
def test_legacy_run_uses_explicit_manifest_sensor(tmp_path, sensor):
    root = _repo_root(tmp_path)
    run_dir = tmp_path / "run"
    _write_manifest(run_dir, {"sensor": sensor})

    resolution = resolve_run_calibration(run_dir, repo_root=root)

    assert resolution.path.name == f"go2w_{sensor}_calibration.yaml"
    assert resolution.source == f"repository {sensor} fallback"


@pytest.mark.parametrize(
    ("topics", "sensor"),
    [
        (["/points_raw", "/go2w/imu"], "xt16"),
        (["/livox/lidar", "/livox/imu"], "mid360"),
    ],
)
def test_legacy_run_infers_sensor_from_input_pair(tmp_path, topics, sensor):
    root = _repo_root(tmp_path)
    run_dir = tmp_path / "run"
    _write_manifest(run_dir, {"playback": {"topics": topics}})

    resolution = resolve_run_calibration(run_dir, repo_root=root)

    assert resolution.path.name == f"go2w_{sensor}_calibration.yaml"


def test_unknown_legacy_sensor_fails_closed(tmp_path):
    root = _repo_root(tmp_path)
    run_dir = tmp_path / "run"
    _write_manifest(run_dir, {"playback": {"topics": ["/other"]}})

    with pytest.raises(CalibrationResolutionError, match="could not infer"):
        resolve_run_calibration(run_dir, repo_root=root)
