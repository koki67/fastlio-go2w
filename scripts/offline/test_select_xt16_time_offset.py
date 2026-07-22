from copy import deepcopy

from select_xt16_time_offset import EXPECTED_OFFSETS, select_runs


def _run(offset, *, p95=0.10, median=0.05, planarity=0.8, **metrics):
    trajectory = {
        "sample_count": 1000,
        "finite_sample_count": 1000,
        "nonfinite_sample_count": 0,
        "gap_count": 0,
        "translation_jump_count": 0,
        "orientation_jump_count": 0,
    }
    map_metrics = {"frame_count": 1000}
    trajectory.update(
        {
            key: value
            for key, value in metrics.items()
            if key in trajectory
        }
    )
    map_metrics.update(
        {key: value for key, value in metrics.items() if key in map_metrics}
    )
    return {
        "run_dir": f"/runs/{offset:+.3f}",
        "manifest": {
            "state": "completed",
            "exit_code": 0,
            "sensor": "xt16",
            "bag": {"path": "/bags/stair2", "metadata_sha256": "bag-hash"},
            "playback": {
                "topics": ["/points_raw", "/go2w/imu"],
                "start_offset_s": 20.0,
                "duration_s": None,
                "rate": 1.0,
                "lidar_time_offset_sec": offset,
            },
            "analysis": {
                "voxel_size_m": 0.2,
                "preview_max_points": 500000,
                "plane_random_seed": 7,
            },
            "hesai_adapter": {
                "diagnostics": {
                    "values": {
                        "received_frames": 1000,
                        "invalid_points": 0,
                        "dropped_nonfinite_timestamp": 0,
                    }
                }
            },
        },
        "summary": {
            "trajectory": trajectory,
            "map": {
                **map_metrics,
                "input_point_count": 100000,
                "finite_point_count": 100000,
                "pointcloud_parse_error_count": 0,
                "local_planes": {
                    "plane_thickness_m": {"median": median, "p95": p95},
                    "planarity": {"median": planarity},
                },
            },
        },
    }


def _runs():
    return [_run(offset) for offset in EXPECTED_OFFSETS]


def test_selects_zero_when_nonzero_does_not_improve_p95_over_five_percent():
    report = select_runs(_runs())

    assert report["selected_offset_sec"] == 0.0
    assert report["selection_reason"] == "no_nonzero_candidate_met_all_adoption_gates"


def test_selects_eligible_nonzero_candidate():
    runs = _runs()
    candidate = next(run for run in runs if run["manifest"]["playback"]["lidar_time_offset_sec"] == -0.005)
    candidate["summary"]["map"]["local_planes"]["plane_thickness_m"] = {
        "median": 0.049,
        "p95": 0.090,
    }
    candidate["summary"]["map"]["local_planes"]["planarity"]["median"] = 0.81

    report = select_runs(runs)

    assert report["selected_offset_sec"] == -0.005
    selected = next(item for item in report["candidates"] if item["offset_sec"] == -0.005)
    assert selected["eligible"]
    assert selected["p95_improvement_fraction"] > 0.05


def test_tie_break_prefers_absolute_then_numeric_offset():
    runs = _runs()
    for offset in (-0.005, 0.005, 0.010):
        candidate = next(
            run
            for run in runs
            if run["manifest"]["playback"]["lidar_time_offset_sec"] == offset
        )
        candidate["summary"]["map"]["local_planes"]["plane_thickness_m"] = {
            "median": 0.049,
            "p95": 0.090,
        }
        candidate["summary"]["map"]["local_planes"]["planarity"]["median"] = 0.81

    report = select_runs(runs)

    assert report["selected_offset_sec"] == -0.005


def test_disqualifies_coverage_and_continuity_regressions():
    runs = _runs()
    candidate = next(run for run in runs if run["manifest"]["playback"]["lidar_time_offset_sec"] == 0.005)
    candidate["summary"]["trajectory"].update(
        {"finite_sample_count": 989, "gap_count": 1}
    )
    candidate["summary"]["map"]["frame_count"] = 989
    candidate["summary"]["map"]["local_planes"]["plane_thickness_m"] = {
        "median": 0.04,
        "p95": 0.08,
    }
    candidate["summary"]["map"]["local_planes"]["planarity"]["median"] = 0.9

    report = select_runs(runs)
    entry = next(item for item in report["candidates"] if item["offset_sec"] == 0.005)

    assert not entry["eligible"]
    assert "sample_coverage_decreased_at_least_1_percent" in entry[
        "disqualification_reasons"
    ]
    assert "gap_count_increased" in entry["disqualification_reasons"]


def test_disqualifies_exactly_one_percent_coverage_decrease():
    runs = _runs()
    candidate = next(
        run
        for run in runs
        if run["manifest"]["playback"]["lidar_time_offset_sec"] == 0.005
    )
    candidate["summary"]["trajectory"]["finite_sample_count"] = 990
    candidate["summary"]["map"]["frame_count"] = 990
    candidate["summary"]["map"]["local_planes"]["plane_thickness_m"] = {
        "median": 0.04,
        "p95": 0.08,
    }
    candidate["summary"]["map"]["local_planes"]["planarity"]["median"] = 0.9

    report = select_runs(runs)
    entry = next(item for item in report["candidates"] if item["offset_sec"] == 0.005)

    assert "sample_coverage_decreased_at_least_1_percent" in entry[
        "disqualification_reasons"
    ]


def test_input_order_does_not_change_selection():
    runs = _runs()
    first = select_runs(runs)
    second = select_runs(list(reversed(deepcopy(runs))))
    assert first["selected_offset_sec"] == second["selected_offset_sec"]
