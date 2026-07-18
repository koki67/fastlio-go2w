#!/usr/bin/env python3

import csv
import json
import struct
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from analyze_multilidar_run import (  # noqa: E402
    PoseSample,
    _validate_comparison_runs,
    DiagnosticCollector,
    VoxelAccumulator,
    pointcloud2_xyz,
    resource_metrics,
    trajectory_divergence,
    trajectory_metrics,
    write_pcd,
)


def _pose(stamp_ns, x, quaternion=(0.0, 0.0, 0.0, 1.0)):
    return PoseSample(stamp_ns, stamp_ns + 10, (x, 0.0, 0.0), quaternion)


def test_trajectory_metrics_reports_gaps_jumps_and_path():
    samples = [
        _pose(0, 0.0),
        _pose(100_000_000, 0.2),
        _pose(600_000_000, 2.0, (0.0, 0.0, 1.0, 0.0)),
    ]
    metrics = trajectory_metrics(
        samples,
        gap_threshold_s=0.2,
        translation_jump_threshold_m=1.0,
        orientation_jump_threshold_deg=15.0,
    )

    assert metrics["sample_count"] == 3
    assert metrics["gap_count"] == 1
    assert metrics["translation_jump_count"] == 1
    assert metrics["orientation_jump_count"] == 1
    assert np.isclose(metrics["translation_jump_first_elapsed_s"], 0.6)
    assert np.isclose(metrics["translation_max_step_elapsed_s"], 0.6)
    assert np.isclose(metrics["orientation_jump_first_elapsed_s"], 0.6)
    assert np.isclose(metrics["orientation_max_step_elapsed_s"], 0.6)
    assert np.isclose(metrics["path_length_m"], 2.0)
    assert np.isclose(metrics["terminal_displacement_m"], 2.0)


def test_quaternion_sign_flip_is_not_an_orientation_jump():
    metrics = trajectory_metrics(
        [_pose(0, 0.0), _pose(100_000_000, 0.0, (0.0, 0.0, 0.0, -1.0))],
        gap_threshold_s=0.2,
        translation_jump_threshold_m=1.0,
        orientation_jump_threshold_deg=1.0,
    )

    assert metrics["orientation_jump_count"] == 0
    assert np.isclose(metrics["orientation_steps_deg"]["max"], 0.0)


def test_voxel_accumulator_centroids_and_plane_statistics():
    accumulator = VoxelAccumulator(0.5, chunk_points=8)
    x, y = np.meshgrid(np.linspace(-1.0, 1.0, 9), np.linspace(-1.0, 1.0, 9))
    plane = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))
    accumulator.add(plane[:40])
    accumulator.add(plane[40:])

    centroids, counts, keys = accumulator.sorted_centroids()
    local = accumulator.local_plane_metrics(
        radius_m=0.8, min_points=5, max_samples=100, random_seed=1
    )

    assert len(keys) == centroids.shape[0] == counts.shape[0]
    assert int(np.sum(counts)) == plane.shape[0]
    assert local["valid_neighborhood_count"] > 0
    assert np.isclose(local["plane_thickness_m"]["max"], 0.0, atol=1.0e-9)
    assert local["planarity"]["median"] > 0.0


def test_write_pcd_supports_ascii_and_binary(tmp_path):
    points = np.asarray([[1.0, 2.0, 3.0], [-1.0, 0.5, 4.0]])
    counts = np.asarray([2, 7], dtype=np.uint32)
    ascii_path = tmp_path / "ascii.pcd"
    binary_path = tmp_path / "binary.pcd"

    write_pcd(ascii_path, points, counts, "ascii")
    write_pcd(binary_path, points, counts, "binary")

    ascii_data = ascii_path.read_text(encoding="utf-8")
    assert "DATA ascii\n" in ascii_data
    assert "1 2 3 2\n" in ascii_data
    binary_data = binary_path.read_bytes()
    marker = b"DATA binary\n"
    assert marker in binary_data
    assert len(binary_data.split(marker, 1)[1]) == 2 * 16


class _Field:
    def __init__(self, name, offset, datatype=7, count=1):
        self.name = name
        self.offset = offset
        self.datatype = datatype
        self.count = count


class _PointCloud:
    pass


def test_pointcloud2_xyz_handles_organized_row_padding():
    message = _PointCloud()
    message.fields = [_Field("x", 0), _Field("y", 4), _Field("z", 8)]
    message.is_bigendian = False
    message.point_step = 12
    message.width = 2
    message.height = 2
    message.row_step = 28

    row1 = struct.pack("<ffffff", 1, 2, 3, 4, 5, 6) + b"pad!"
    row2 = struct.pack("<ffffff", 7, 8, 9, 10, 11, 12) + b"pad!"
    message.data = row1 + row2

    points = pointcloud2_xyz(message)

    np.testing.assert_allclose(
        points,
        np.asarray([[1, 2, 3], [4, 5, 6], [7, 8, 9], [10, 11, 12]]),
    )


def test_diagnostic_collector_handles_humble_uint8_bytes():
    collector = DiagnosticCollector()
    collector.add(
        "/diagnostics",
        SimpleNamespace(
            status=[
                SimpleNamespace(
                    name="fusion",
                    level=b"\x01",
                    message="idle flush",
                    values=[],
                )
            ]
        ),
    )
    assert collector.summary()["level_counts"] == {"WARN": 1}


def _comparison_run(label, rate=1.0):
    return {
        "label": label,
        "manifest": {
            "state": "completed",
            "exit_code": 0,
            "bag": {"path": "/bag", "metadata_sha256": "abc"},
            "playback": {
                "topics": ["/livox/lidar", "/livox/imu", "/points_raw"],
                "start_offset_s": 0.0,
                "duration_s": None,
                "rate": rate,
            },
        },
        "summary": {
            "primary_trajectory_topic": "/odom",
            "thresholds": {
                "trajectory_gap_s": 0.2,
                "translation_jump_m": 1.0,
                "orientation_jump_deg": 15.0,
            },
            "trajectory": {"sample_count": 3, "finite_sample_count": 3},
            "map": {
                "frame_count": 3,
                "finite_point_count": 10,
                "voxel_size_m": 0.2,
                "pcd_data_format": "binary",
                "local_planes": {"radius_m": 0.6, "minimum_points": 20},
            },
        },
    }


def test_comparison_validation_requires_matching_replay_and_analysis_settings():
    baseline = _comparison_run("baseline")
    fused = _comparison_run("fused")

    validated = _validate_comparison_runs([baseline, fused])
    assert validated["manifest.playback.rate"] == 1.0

    fused["manifest"]["playback"]["rate"] = 2.0
    with pytest.raises(ValueError, match="manifest.playback.rate"):
        _validate_comparison_runs([baseline, fused])


def test_trajectory_divergence_interpolates_reference():
    reference_times = np.asarray([0, 1_000_000_000, 2_000_000_000], dtype=np.int64)
    reference_positions = np.asarray([[0, 0, 0], [1, 0, 0], [2, 0, 0]], dtype=float)
    candidate_times = np.asarray([500_000_000, 1_500_000_000], dtype=np.int64)
    candidate_positions = np.asarray([[0.5, 1, 0], [1.5, 1, 0]], dtype=float)

    divergence = trajectory_divergence(
        reference_times, reference_positions, candidate_times, candidate_positions
    )

    assert divergence["common_sample_count"] == 2
    assert np.isclose(divergence["translation_difference_m"]["mean"], 1.0)


def test_resource_metrics_derives_cpu_and_peak_rss(tmp_path):
    csv_path = tmp_path / "resource_metrics.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=["wall_time_s", "pid", "process", "cpu_time_s", "rss_kib"],
        )
        writer.writeheader()
        writer.writerows(
            [
                {
                    "wall_time_s": 0,
                    "pid": 10,
                    "process": "fastlio",
                    "cpu_time_s": 2,
                    "rss_kib": 100,
                },
                {
                    "wall_time_s": 2,
                    "pid": 10,
                    "process": "fastlio",
                    "cpu_time_s": 5,
                    "rss_kib": 120,
                },
            ]
        )
    (tmp_path / "resource_summary.json").write_text(
        json.dumps({"runner": "ok"}), encoding="utf-8"
    )

    result = resource_metrics(tmp_path)

    assert result["available"]
    assert np.isclose(result["processes"]["fastlio"]["average_cpu_cores"], 1.5)
    assert result["processes"]["fastlio"]["peak_rss_bytes"] == 120 * 1024
