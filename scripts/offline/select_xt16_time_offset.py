#!/usr/bin/env python3
"""Select an XT16 fixed LiDAR time offset from five analyzed offline runs."""

from __future__ import annotations

import argparse
import csv
import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Sequence


EXPECTED_OFFSETS = (-0.010, -0.005, 0.0, 0.005, 0.010)


def _nested(document: Mapping[str, Any], path: str) -> Any:
    current: Any = document
    for part in path.split("."):
        if not isinstance(current, Mapping) or part not in current:
            return None
        current = current[part]
    return current


def _finite_number(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _load_run(run_dir: Path) -> dict[str, Any]:
    result: dict[str, Any] = {"run_dir": str(run_dir), "manifest": {}, "summary": {}}
    for name in ("manifest", "summary"):
        path = run_dir / f"{name}.json"
        try:
            result[name] = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            result[f"{name}_error"] = str(error)
    return result


def _metrics(run: Mapping[str, Any]) -> dict[str, Any]:
    summary = run["summary"]
    input_points = _nested(summary, "map.input_point_count")
    finite_points = _nested(summary, "map.finite_point_count")
    map_nonfinite_points = (
        input_points - finite_points
        if isinstance(input_points, int) and isinstance(finite_points, int)
        else None
    )
    return {
        "adapter_received_frames": _nested(
            run["manifest"], "hesai_adapter.diagnostics.values.received_frames"
        ),
        "adapter_invalid_points": _nested(
            run["manifest"], "hesai_adapter.diagnostics.values.invalid_points"
        ),
        "adapter_nonfinite_timestamp_frames": _nested(
            run["manifest"],
            "hesai_adapter.diagnostics.values.dropped_nonfinite_timestamp",
        ),
        "trajectory_sample_count": _nested(summary, "trajectory.sample_count"),
        "trajectory_finite_sample_count": _nested(
            summary, "trajectory.finite_sample_count"
        ),
        "trajectory_nonfinite_sample_count": _nested(
            summary, "trajectory.nonfinite_sample_count"
        ),
        "gap_count": _nested(summary, "trajectory.gap_count"),
        "translation_jump_count": _nested(
            summary, "trajectory.translation_jump_count"
        ),
        "orientation_jump_count": _nested(
            summary, "trajectory.orientation_jump_count"
        ),
        "map_frame_count": _nested(summary, "map.frame_count"),
        "map_nonfinite_point_count": map_nonfinite_points,
        "pointcloud_parse_error_count": _nested(
            summary, "map.pointcloud_parse_error_count"
        ),
        "plane_thickness_median_m": _nested(
            summary, "map.local_planes.plane_thickness_m.median"
        ),
        "plane_thickness_p95_m": _nested(
            summary, "map.local_planes.plane_thickness_m.p95"
        ),
        "planarity_median": _nested(
            summary, "map.local_planes.planarity.median"
        ),
    }


def _offset(run: Mapping[str, Any]) -> float | None:
    return _finite_number(_nested(run["manifest"], "playback.lidar_time_offset_sec"))


def _is_completed(run: Mapping[str, Any]) -> bool:
    return (
        run["manifest"].get("state") == "completed"
        and run["manifest"].get("exit_code") == 0
        and not run.get("summary_error")
    )


def _same_run_contract(reference: Mapping[str, Any], candidate: Mapping[str, Any]) -> bool:
    paths = (
        "bag.path",
        "bag.metadata_sha256",
        "sensor",
        "playback.topics",
        "playback.start_offset_s",
        "playback.duration_s",
        "playback.rate",
        "analysis.voxel_size_m",
        "analysis.preview_max_points",
        "analysis.plane_random_seed",
        "fastlio.config_sha256",
        "calibration.sha256",
        "git.commit",
        "launch_sha256",
        "runtime_executables.fastlio.sha256",
        "runtime_executables.odom_adapter.sha256",
        "runtime_executables.hesai_pointcloud_adapter.sha256",
    )
    return all(
        _nested(reference["manifest"], path) == _nested(candidate["manifest"], path)
        for path in paths
    )


def select_runs(runs: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    by_offset: dict[float, Mapping[str, Any]] = {}
    for run in runs:
        offset = _offset(run)
        if offset is None:
            continue
        canonical = min(EXPECTED_OFFSETS, key=lambda expected: abs(expected - offset))
        if abs(canonical - offset) > 1e-9 or canonical in by_offset:
            continue
        by_offset[canonical] = run
    missing = [offset for offset in EXPECTED_OFFSETS if offset not in by_offset]
    if missing:
        raise ValueError(f"missing or duplicate expected offsets: {missing}")

    reference = by_offset[0.0]
    if not _is_completed(reference):
        raise ValueError("0 ms reference run is not completed and analyzed")
    reference_metrics = _metrics(reference)
    required_numeric = tuple(reference_metrics)
    if any(_finite_number(reference_metrics[key]) is None for key in required_numeric):
        raise ValueError("0 ms reference has missing or non-finite selection metrics")
    if (
        reference_metrics["trajectory_finite_sample_count"] <= 0
        or reference_metrics["map_frame_count"] <= 0
        or reference_metrics["adapter_received_frames"] <= 0
    ):
        raise ValueError(
            "0 ms reference has no adapter, finite-trajectory, or map-frame coverage"
        )

    candidates: list[dict[str, Any]] = []
    eligible_nonzero: list[dict[str, Any]] = []
    for offset in EXPECTED_OFFSETS:
        run = by_offset[offset]
        metrics = _metrics(run)
        reasons: list[str] = []
        if not _is_completed(run):
            reasons.append("run_not_completed")
        if not _same_run_contract(reference, run):
            reasons.append("run_contract_differs")
        missing_metrics = [
            key for key, value in metrics.items() if _finite_number(value) is None
        ]
        if missing_metrics:
            reasons.append("missing_or_nonfinite_metrics:" + ",".join(missing_metrics))

        sample_coverage_ratio = None
        map_coverage_ratio = None
        adapter_input_coverage_ratio = None
        p95_improvement_fraction = None
        if not missing_metrics:
            sample_coverage_ratio = (
                float(metrics["trajectory_finite_sample_count"])
                / float(reference_metrics["trajectory_finite_sample_count"])
            )
            map_coverage_ratio = float(metrics["map_frame_count"]) / float(
                reference_metrics["map_frame_count"]
            )
            adapter_input_coverage_ratio = float(metrics["adapter_received_frames"]) / float(
                reference_metrics["adapter_received_frames"]
            )
            reference_p95 = float(reference_metrics["plane_thickness_p95_m"])
            candidate_p95 = float(metrics["plane_thickness_p95_m"])
            p95_improvement_fraction = (
                (reference_p95 - candidate_p95) / reference_p95
                if reference_p95 > 0.0
                else 0.0
            )
            if metrics["trajectory_nonfinite_sample_count"] > reference_metrics[
                "trajectory_nonfinite_sample_count"
            ]:
                reasons.append("nonfinite_samples_increased")
            if metrics["map_nonfinite_point_count"] > reference_metrics[
                "map_nonfinite_point_count"
            ]:
                reasons.append("nonfinite_map_points_increased")
            if metrics["pointcloud_parse_error_count"] > reference_metrics[
                "pointcloud_parse_error_count"
            ]:
                reasons.append("pointcloud_parse_errors_increased")
            if metrics["adapter_invalid_points"] > reference_metrics[
                "adapter_invalid_points"
            ]:
                reasons.append("adapter_invalid_points_increased")
            if metrics["adapter_nonfinite_timestamp_frames"] > reference_metrics[
                "adapter_nonfinite_timestamp_frames"
            ]:
                reasons.append("adapter_nonfinite_timestamp_frames_increased")
            if (
                sample_coverage_ratio <= 0.99
                or map_coverage_ratio <= 0.99
                or adapter_input_coverage_ratio <= 0.99
            ):
                reasons.append("sample_coverage_decreased_at_least_1_percent")
            for metric in ("gap_count", "translation_jump_count", "orientation_jump_count"):
                if metrics[metric] > reference_metrics[metric]:
                    reasons.append(f"{metric}_increased")
            if offset != 0.0:
                if p95_improvement_fraction <= 0.05:
                    reasons.append("plane_thickness_p95_improvement_not_over_5_percent")
                if metrics["plane_thickness_median_m"] > reference_metrics[
                    "plane_thickness_median_m"
                ]:
                    reasons.append("plane_thickness_median_worsened")
                if metrics["planarity_median"] < reference_metrics["planarity_median"]:
                    reasons.append("planarity_median_worsened")

        entry = {
            "offset_sec": offset,
            "offset_ms": offset * 1000.0,
            "run_dir": run["run_dir"],
            "eligible": not reasons,
            "disqualification_reasons": reasons,
            "sample_coverage_ratio": sample_coverage_ratio,
            "map_frame_coverage_ratio": map_coverage_ratio,
            "adapter_input_coverage_ratio": adapter_input_coverage_ratio,
            "p95_improvement_fraction": p95_improvement_fraction,
            "metrics": metrics,
        }
        candidates.append(entry)
        if offset != 0.0 and not reasons:
            eligible_nonzero.append(entry)

    if eligible_nonzero:
        selected = min(
            eligible_nonzero,
            key=lambda entry: (
                entry["metrics"]["plane_thickness_p95_m"],
                abs(entry["offset_sec"]),
                entry["offset_sec"],
            ),
        )
        selection_reason = "eligible_nonzero_candidate_with_best_p95"
    else:
        selected = next(entry for entry in candidates if entry["offset_sec"] == 0.0)
        selection_reason = "no_nonzero_candidate_met_all_adoption_gates"

    return {
        "schema_version": 1,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "selection_policy": {
            "offset_candidates_ms": [-10, -5, 0, 5, 10],
            "maximum_coverage_decrease_fraction": 0.01,
            "coverage_decrease_at_or_above_threshold_disqualifies": True,
            "required_p95_improvement_fraction_strictly_greater_than": 0.05,
            "continuity_must_not_worsen": True,
            "median_thickness_must_not_worsen": True,
            "median_planarity_must_not_worsen": True,
            "tie_break": ["p95_thickness", "absolute_offset", "numeric_offset"],
        },
        "selected_offset_sec": selected["offset_sec"],
        "selected_offset_ms": selected["offset_ms"],
        "selection_reason": selection_reason,
        "reference_run_dir": reference["run_dir"],
        "candidates": candidates,
        "limitations": [
            "No ground truth is available; this is an internal consistency selection.",
            "The selected value is provisional until a corrected-driver recording is tested.",
        ],
    }


def write_report(report: Mapping[str, Any], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + ".tmp")
    temporary.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(output)
    csv_path = output.with_suffix(".csv")
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        fields = [
            "offset_ms",
            "run_dir",
            "eligible",
            "disqualification_reasons",
            "sample_coverage_ratio",
            "map_frame_coverage_ratio",
            "adapter_input_coverage_ratio",
            "p95_improvement_fraction",
            "plane_thickness_median_m",
            "plane_thickness_p95_m",
            "planarity_median",
            "gap_count",
            "translation_jump_count",
            "orientation_jump_count",
            "trajectory_nonfinite_sample_count",
            "map_nonfinite_point_count",
            "pointcloud_parse_error_count",
            "adapter_received_frames",
            "adapter_invalid_points",
            "adapter_nonfinite_timestamp_frames",
        ]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for candidate in report["candidates"]:
            metrics = candidate["metrics"]
            writer.writerow(
                {
                    **{field: candidate.get(field) for field in fields},
                    "disqualification_reasons": ";".join(
                        candidate["disqualification_reasons"]
                    ),
                    **{field: metrics.get(field) for field in fields if field in metrics},
                }
            )


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dirs", nargs=5, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        report = select_runs([_load_run(path.expanduser().resolve()) for path in args.run_dirs])
        write_report(report, args.output.expanduser().resolve())
    except (OSError, ValueError) as error:
        print(f"error: {error}")
        return 2
    print(f"selected {report['selected_offset_ms']:+g} ms")
    print(f"wrote {args.output.expanduser().resolve()}")
    print(f"wrote {args.output.expanduser().resolve().with_suffix('.csv')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
