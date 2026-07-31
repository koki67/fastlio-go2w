#!/usr/bin/env bash
# Run reproducible headless FAST-LIO against a recorded MID-360 ROS 2 bag.

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

usage() {
    cat <<'EOF'
Usage:
  scripts/offline/run_fastlio_offline.sh BAG [options]

Options:
  --start-offset SEC  Start this many seconds into the bag (default: 0)
  --duration SEC      Approximate bag seconds via wall timer (smoke tests only)
  --rate RATE         Rosbag playback multiplier (default: 1.0)
  --domain-id ID      Isolated ROS domain ID (default: 77)
  --output DIR        Result directory (default: ${FASTLIO_RESULTS_ROOT}/fastlio/<bag>/...)
  --config YAML       Override tuning while preserving the MID-360 input and
                      headless publisher contract
  --lidar-format NAME Livox input format: auto, custom-msg, or pointcloud2
                      (default: auto; explicit values must match metadata)
  --no-analyze        Keep the result bag but skip automatic artifact generation
  --map-voxel-size M  Final map voxel edge length (default: 0.20)
  --preview-max-points N
                      Maximum points in the RViz preview map (default: 500000)
  --plane-random-seed N
                      Deterministic local-plane sample seed (default: 7)
  -h, --help          Show this help

The workspace must already be built. The runner plays only /livox/lidar and
/livox/imu, starts playback paused, waits for every endpoint, then resumes
through the rosbag player service.
EOF
}

die() {
    echo "Error: $*" >&2
    exit 2
}

BAG=""
START_OFFSET="0"
DURATION=""
RATE="1.0"
DOMAIN_ID="77"
OUTPUT_DIR=""
CONFIG_OVERRIDE=""
LIDAR_FORMAT_OVERRIDE="auto"
ANALYZE="true"
MAP_VOXEL_SIZE="0.20"
PREVIEW_MAX_POINTS="500000"
PLANE_RANDOM_SEED="7"
FASTLIO_RESULTS_ROOT="${FASTLIO_RESULTS_ROOT:-$REPO_ROOT/results}"

while [ "$#" -gt 0 ]; do
    case "$1" in
        --start-offset)
            START_OFFSET="${2:?Error: --start-offset requires a value}"
            shift 2
            ;;
        --duration)
            DURATION="${2:?Error: --duration requires a value}"
            shift 2
            ;;
        --rate)
            RATE="${2:?Error: --rate requires a value}"
            shift 2
            ;;
        --domain-id)
            DOMAIN_ID="${2:?Error: --domain-id requires a value}"
            shift 2
            ;;
        --output)
            OUTPUT_DIR="${2:?Error: --output requires a value}"
            shift 2
            ;;
        --config)
            CONFIG_OVERRIDE="${2:?Error: --config requires a value}"
            shift 2
            ;;
        --lidar-format)
            LIDAR_FORMAT_OVERRIDE="${2:?Error: --lidar-format requires a value}"
            shift 2
            ;;
        --no-analyze)
            ANALYZE="false"
            shift
            ;;
        --map-voxel-size)
            MAP_VOXEL_SIZE="${2:?Error: --map-voxel-size requires a value}"
            shift 2
            ;;
        --preview-max-points)
            PREVIEW_MAX_POINTS="${2:?Error: --preview-max-points requires a value}"
            shift 2
            ;;
        --plane-random-seed)
            PLANE_RANDOM_SEED="${2:?Error: --plane-random-seed requires a value}"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        -*)
            die "unknown option: $1"
            ;;
        *)
            [ -z "$BAG" ] || die "only one bag path may be supplied"
            BAG="$1"
            shift
            ;;
    esac
done

[ -n "$BAG" ] || die "a bag directory is required"

python3 -c 'import sys; value=float(sys.argv[1]); sys.exit(not value >= 0)' "$START_OFFSET" \
    || die "--start-offset must be a non-negative number"
python3 -c 'import sys; value=float(sys.argv[1]); sys.exit(not value > 0)' "$RATE" \
    || die "--rate must be greater than zero"
if [ "$ANALYZE" = "true" ]; then
    python3 -c 'import sys; value=float(sys.argv[1]); sys.exit(not value > 0)' "$MAP_VOXEL_SIZE" \
        || die "--map-voxel-size must be greater than zero"
    [[ "$PREVIEW_MAX_POINTS" =~ ^[1-9][0-9]*$ ]] \
        || die "--preview-max-points must be a positive integer"
    [[ "$PLANE_RANDOM_SEED" =~ ^[0-9]+$ ]] \
        || die "--plane-random-seed must be a non-negative integer"
fi
if [ -n "$DURATION" ]; then
    python3 -c 'import sys; value=float(sys.argv[1]); sys.exit(not value > 0)' "$DURATION" \
        || die "--duration must be greater than zero"
fi
[[ "$DOMAIN_ID" =~ ^[0-9]+$ ]] || die "--domain-id must be a non-negative integer"
case "$LIDAR_FORMAT_OVERRIDE" in
    auto|custom-msg|pointcloud2) ;;
    *) die "--lidar-format must be auto, custom-msg, or pointcloud2" ;;
esac

BAG="$(realpath "$BAG")"
[ -d "$BAG" ] || die "bag directory not found: $BAG"
[ -f "$BAG/metadata.yaml" ] || die "bag metadata not found: $BAG/metadata.yaml"
grep -Eq "name: /livox/imu$" "$BAG/metadata.yaml" \
    || die "required topic /livox/imu is absent from the bag"

FORMAT_DETECTOR="$REPO_ROOT/scripts/fastlio/detect_livox_bag_format.py"
[ -f "$FORMAT_DETECTOR" ] || die "Livox bag format detector not found: $FORMAT_DETECTOR"
if ! DETECTED_LIDAR_FORMAT="$(python3 "$FORMAT_DETECTOR" "$BAG/metadata.yaml")"; then
    die "could not establish a supported Livox input format"
fi
if [ "$LIDAR_FORMAT_OVERRIDE" != "auto" ] && \
   [ "$LIDAR_FORMAT_OVERRIDE" != "$DETECTED_LIDAR_FORMAT" ]; then
    die "--lidar-format $LIDAR_FORMAT_OVERRIDE conflicts with metadata format $DETECTED_LIDAR_FORMAT"
fi
RESOLVED_LIDAR_FORMAT="$DETECTED_LIDAR_FORMAT"
case "$RESOLVED_LIDAR_FORMAT" in
    custom-msg)
        DETECTED_LIDAR_ROS_TYPE="livox_ros_driver2/msg/CustomMsg"
        FASTLIO_LID_TOPIC="/livox/lidar"
        LIVOX_ADAPTER_ENABLED="false"
        ;;
    pointcloud2)
        DETECTED_LIDAR_ROS_TYPE="sensor_msgs/msg/PointCloud2"
        FASTLIO_LID_TOPIC="/livox/lidar_fastlio"
        LIVOX_ADAPTER_ENABLED="true"
        ;;
esac
LIVOX_ADAPTER_INPUT_TOPIC="/livox/lidar"
LIVOX_ADAPTER_OUTPUT_TOPIC="/livox/lidar_fastlio"
LIVOX_ADAPTER_DIAGNOSTICS_TOPIC="/fastlio_go2w_livox/diagnostics"
LIVOX_ADAPTER_MAX_DELTA_SEC="0.2"
LIVOX_ADAPTER_MINIMUM_POINTS="10"

CONFIG_DIR="$REPO_ROOT/humble_ws/src/fastlio_go2w_bringup/config"
if [ -n "$CONFIG_OVERRIDE" ]; then
    if [ -f "$CONFIG_OVERRIDE" ]; then
        FASTLIO_CONFIG="$(realpath "$CONFIG_OVERRIDE")"
    elif [ -f "$REPO_ROOT/$CONFIG_OVERRIDE" ]; then
        FASTLIO_CONFIG="$(realpath "$REPO_ROOT/$CONFIG_OVERRIDE")"
    elif [ -f "$CONFIG_DIR/$CONFIG_OVERRIDE" ]; then
        FASTLIO_CONFIG="$(realpath "$CONFIG_DIR/$CONFIG_OVERRIDE")"
    else
        die "FAST-LIO config not found: $CONFIG_OVERRIDE"
    fi
else
    FASTLIO_CONFIG="$CONFIG_DIR/mid360_go2w_accuracy_offline.yaml"
fi
[ -f "$FASTLIO_CONFIG" ] || die "offline config not found: $FASTLIO_CONFIG"

if [ -z "$OUTPUT_DIR" ]; then
    BAG_NAME="$(basename "$BAG")"
    RUN_STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
    OUTPUT_DIR="$(realpath -m "${FASTLIO_RESULTS_ROOT}/fastlio/$BAG_NAME/$RUN_STAMP")"
else
    OUTPUT_DIR="$(realpath -m "$OUTPUT_DIR")"
fi
if [ -d "$OUTPUT_DIR" ] && [ -n "$(find "$OUTPUT_DIR" -mindepth 1 -print -quit)" ]; then
    die "output directory is not empty: $OUTPUT_DIR"
fi

if [ -f /opt/ros/humble/setup.bash ]; then
    set +u
    source /opt/ros/humble/setup.bash
    set -u
elif [ "${ROS_DISTRO:-}" != "humble" ]; then
    die "ROS 2 Humble is required (run this inside the project container)"
fi

for command in ros2 setsid pgrep python3 sha256sum cp git realpath; do
    command -v "$command" >/dev/null || die "required command not found: $command"
done
python3 -c 'import yaml' >/dev/null 2>&1 \
    || die "PyYAML is required for result validation"

LAUNCH_SOURCE="$REPO_ROOT/humble_ws/src/fastlio_go2w_bringup/launch/offline_fastlio.launch.py"
[ -f "$LAUNCH_SOURCE" ] || die "experiment launch source not found: $LAUNCH_SOURCE"
LAUNCH_SHA256="$(sha256sum "$LAUNCH_SOURCE" | awk '{print $1}')"
GRAPH_SOURCE="$REPO_ROOT/humble_ws/src/fastlio_go2w_bringup/src/fastlio_go2w_bringup/livox_replay.py"
[ -f "$GRAPH_SOURCE" ] || die "Livox replay graph source not found: $GRAPH_SOURCE"
GRAPH_SHA256="$(sha256sum "$GRAPH_SOURCE" | awk '{print $1}')"

LIVOX_ADAPTER_SOURCE=""
LIVOX_ADAPTER_SOURCE_SHA256=""
DIAGNOSTICS_EXTRACTOR_SOURCE=""
if [ "$LIVOX_ADAPTER_ENABLED" = "true" ]; then
    LIVOX_ADAPTER_SOURCE="$REPO_ROOT/humble_ws/src/fastlio_go2w_livox"
    [ -d "$LIVOX_ADAPTER_SOURCE" ] || die "Livox adapter source package not found"
    LIVOX_ADAPTER_SOURCE_SHA256="$(
        sha256sum \
            "$LIVOX_ADAPTER_SOURCE/include/fastlio_go2w_livox/pointcloud_adapter.hpp" \
            "$LIVOX_ADAPTER_SOURCE/src/pointcloud_adapter.cpp" \
            "$LIVOX_ADAPTER_SOURCE/src/livox_pointcloud_adapter_node.cpp" | sha256sum | awk '{print $1}'
    )"
    DIAGNOSTICS_EXTRACTOR_SOURCE="$SCRIPT_DIR/extract_livox_adapter_diagnostics.py"
    [ -f "$DIAGNOSTICS_EXTRACTOR_SOURCE" ] \
        || die "Livox diagnostics extractor not found: $DIAGNOSTICS_EXTRACTOR_SOURCE"
fi

ANALYZER_SOURCE=""
if [ "$ANALYZE" = "true" ]; then
    ANALYZER_SOURCE="$SCRIPT_DIR/analyze_fastlio_run.py"
    [ -f "$ANALYZER_SOURCE" ] || die "analyzer not found: $ANALYZER_SOURCE"
fi

candidate_is_usable() {
    local candidate="$1"
    local install_root bringup_prefix launch_runtime graph_runtime fastlio_runtime
    local odom_runtime adapter_runtime graph_egg_link graph_python_root
    install_root="$(dirname "$candidate")"
    bringup_prefix="$install_root/fastlio_go2w_bringup"
    launch_runtime="$bringup_prefix/share/fastlio_go2w_bringup/launch/offline_fastlio.launch.py"
    graph_runtime="$bringup_prefix/lib/python3.10/site-packages/fastlio_go2w_bringup/livox_replay.py"
    if [ ! -f "$graph_runtime" ]; then
        graph_egg_link="$bringup_prefix/lib/python3.10/site-packages/fastlio-go2w-bringup.egg-link"
        [ -f "$graph_egg_link" ] || return 1
        IFS= read -r graph_python_root < "$graph_egg_link" || return 1
        graph_runtime="$graph_python_root/fastlio_go2w_bringup/livox_replay.py"
    fi
    fastlio_runtime="$install_root/fast_lio/lib/fast_lio/fastlio_mapping"
    odom_runtime="$bringup_prefix/lib/fastlio_go2w_bringup/fastlio_odom_adapter"
    adapter_runtime="$install_root/fastlio_go2w_livox/lib/fastlio_go2w_livox/livox_pointcloud_adapter"

    [ -f "$candidate" ] || return 1
    [ -f "$launch_runtime" ] || return 1
    [ -f "$graph_runtime" ] || return 1
    [ -x "$fastlio_runtime" ] || return 1
    [ -x "$odom_runtime" ] || return 1
    [ "$(sha256sum "$launch_runtime" | awk '{print $1}')" = "$LAUNCH_SHA256" ] \
        || return 1
    [ "$(sha256sum "$graph_runtime" | awk '{print $1}')" = "$GRAPH_SHA256" ] \
        || return 1
    if [ "$LIVOX_ADAPTER_ENABLED" = "true" ]; then
        [ -x "$adapter_runtime" ] || return 1
    fi

    (
        set +u
        source "$candidate"
        set -u
        [ "$(ros2 pkg prefix fastlio_go2w_bringup 2>/dev/null)" = "$bringup_prefix" ] \
            || exit 1
        [ "$(ros2 pkg prefix fast_lio 2>/dev/null)" = "$install_root/fast_lio" ] \
            || exit 1
        if [ "$LIVOX_ADAPTER_ENABLED" = "true" ]; then
            [ "$(ros2 pkg prefix fastlio_go2w_livox 2>/dev/null)" = \
              "$install_root/fastlio_go2w_livox" ] || exit 1
        fi
    )
}

WORKSPACE_SETUP=""
for candidate in \
    "$REPO_ROOT/.devcontainer/desktop_ws/install/setup.bash" \
    "$REPO_ROOT/humble_ws/install/setup.bash"; do
    if candidate_is_usable "$candidate"; then
        WORKSPACE_SETUP="$candidate"
        break
    fi
done
[ -n "$WORKSPACE_SETUP" ] \
    || die "no usable current Humble workspace overlay was found; rebuild the workspace"
set +u
source "$WORKSPACE_SETUP"
set -u

INSTALL_ROOT="$(dirname "$WORKSPACE_SETUP")"
BRINGUP_PREFIX="$INSTALL_ROOT/fastlio_go2w_bringup"
LAUNCH_RUNTIME="$BRINGUP_PREFIX/share/fastlio_go2w_bringup/launch/offline_fastlio.launch.py"
LAUNCH_RUNTIME_SHA256="$(sha256sum "$LAUNCH_RUNTIME" | awk '{print $1}')"
GRAPH_RUNTIME="$BRINGUP_PREFIX/lib/python3.10/site-packages/fastlio_go2w_bringup/livox_replay.py"
if [ ! -f "$GRAPH_RUNTIME" ]; then
    GRAPH_EGG_LINK="$BRINGUP_PREFIX/lib/python3.10/site-packages/fastlio-go2w-bringup.egg-link"
    IFS= read -r GRAPH_PYTHON_ROOT < "$GRAPH_EGG_LINK"
    GRAPH_RUNTIME="$GRAPH_PYTHON_ROOT/fastlio_go2w_bringup/livox_replay.py"
fi
GRAPH_RUNTIME_SHA256="$(sha256sum "$GRAPH_RUNTIME" | awk '{print $1}')"
FASTLIO_RUNTIME="$INSTALL_ROOT/fast_lio/lib/fast_lio/fastlio_mapping"
ODOM_ADAPTER_RUNTIME="$BRINGUP_PREFIX/lib/fastlio_go2w_bringup/fastlio_odom_adapter"
FASTLIO_RUNTIME_SHA256="$(sha256sum "$FASTLIO_RUNTIME" | awk '{print $1}')"
ODOM_ADAPTER_RUNTIME_SHA256="$(sha256sum "$ODOM_ADAPTER_RUNTIME" | awk '{print $1}')"
LIVOX_ADAPTER_RUNTIME=""
LIVOX_ADAPTER_RUNTIME_SHA256=""
if [ "$LIVOX_ADAPTER_ENABLED" = "true" ]; then
    LIVOX_ADAPTER_RUNTIME="$INSTALL_ROOT/fastlio_go2w_livox/lib/fastlio_go2w_livox/livox_pointcloud_adapter"
    LIVOX_ADAPTER_RUNTIME_SHA256="$(sha256sum "$LIVOX_ADAPTER_RUNTIME" | awk '{print $1}')"
fi

python3 -c 'import rosbag2_py' >/dev/null 2>&1 \
    || die "rosbag2 Python support is required for result validation"
if [ "$ANALYZE" = "true" ]; then
    python3 -c 'import numpy, rclpy; from rosidl_runtime_py.utilities import get_message' \
        >/dev/null 2>&1 || die "automatic analysis dependencies are unavailable"
    python3 "$ANALYZER_SOURCE" analyze --help >/dev/null \
        || die "analyzer CLI preflight failed"
fi

# Create the run directory only after all read-only source and overlay preflight passes.
if [ -d "$OUTPUT_DIR" ] && [ -n "$(find "$OUTPUT_DIR" -mindepth 1 -print -quit)" ]; then
    die "output directory is not empty: $OUTPUT_DIR"
fi
mkdir -p "$OUTPUT_DIR"

export ROS_DOMAIN_ID="$DOMAIN_ID"
export RCUTILS_COLORIZED_OUTPUT=0
export ROS_LOG_DIR="$OUTPUT_DIR/ros-logs"
mkdir -p "$ROS_LOG_DIR"

RESULT_BAG="$OUTPUT_DIR/rosbag"
FASTLIO_CONFIG_SNAPSHOT="$OUTPUT_DIR/fastlio_config.yaml"
FASTLIO_PARAMETERS_SNAPSHOT="$OUTPUT_DIR/fastlio_mapping.yaml"
FASTLIO_PARAMETERS_SHA256=""
ANALYZER_SNAPSHOT=""
ANALYZER_SHA256=""
ANALYSIS_LOG=""
if [ "$ANALYZE" = "true" ]; then
    ANALYZER_SNAPSHOT="$OUTPUT_DIR/analyze_fastlio_run.py"
    ANALYSIS_LOG="$OUTPUT_DIR/analysis.log"
    cp "$ANALYZER_SOURCE" "$ANALYZER_SNAPSHOT"
    ANALYZER_SHA256="$(sha256sum "$ANALYZER_SNAPSHOT" | awk '{print $1}')"
fi
LIVOX_ADAPTER_DIAGNOSTICS=""
LIVOX_ADAPTER_DIAGNOSTICS_SHA256=""
DIAGNOSTICS_EXTRACTOR_SNAPSHOT=""
DIAGNOSTICS_EXTRACTOR_SHA256=""
DIAGNOSTICS_LOG=""
if [ "$LIVOX_ADAPTER_ENABLED" = "true" ]; then
    LIVOX_ADAPTER_DIAGNOSTICS="$OUTPUT_DIR/livox_adapter_diagnostics.json"
    DIAGNOSTICS_EXTRACTOR_SNAPSHOT="$OUTPUT_DIR/extract_livox_adapter_diagnostics.py"
    DIAGNOSTICS_LOG="$OUTPUT_DIR/livox-adapter-diagnostics.log"
    cp "$DIAGNOSTICS_EXTRACTOR_SOURCE" "$DIAGNOSTICS_EXTRACTOR_SNAPSHOT"
    DIAGNOSTICS_EXTRACTOR_SHA256="$(sha256sum "$DIAGNOSTICS_EXTRACTOR_SNAPSHOT" | awk '{print $1}')"
fi
PARAMETER_DUMP_LOG="$OUTPUT_DIR/parameter-dump.log"
PARAMETER_VALIDATION_LOG="$OUTPUT_DIR/fastlio-parameter-validation.log"
RESULT_BAG_VALIDATION_LOG="$OUTPUT_DIR/result-bag-validation.log"
RESOURCE_VALIDATION_LOG="$OUTPUT_DIR/resource-validation.log"

cp "$FASTLIO_CONFIG" "$FASTLIO_CONFIG_SNAPSHOT"
git -C "$REPO_ROOT" status --short > "$OUTPUT_DIR/git-status.txt"

MANIFEST="$OUTPUT_DIR/manifest.json"
METRICS_CSV="$OUTPUT_DIR/resource_metrics.csv"
METRICS_SUMMARY="$OUTPUT_DIR/resource_summary.json"
STOP_FILE="$OUTPUT_DIR/.stop-metrics"
DURATION_MARKER="$OUTPUT_DIR/.duration-reached"
RUN_STARTED_AT="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
CONFIG_SHA256="$(sha256sum "$FASTLIO_CONFIG_SNAPSHOT" | awk '{print $1}')"
METADATA_SHA256="$(sha256sum "$BAG/metadata.yaml" | awk '{print $1}')"
RESULT_METADATA_SHA256=""
RESOURCE_METRICS_SHA256=""
RESOURCE_SUMMARY_SHA256=""
DRAIN_STABLE_SECONDS="5"
DRAIN_TIMEOUT_SECONDS="120"
DRAIN_POLL_SECONDS="1"
DRAIN_OUTCOME="not_started"
DRAIN_ELAPSED_SECONDS=""
DRAIN_FINAL_SIZE_BYTES=""
DRAIN_FINAL_MTIME_NS=""

GIT_COMMIT="$(git -C "$REPO_ROOT" rev-parse HEAD)"

LAUNCH_PID=""
RECORDER_PID=""
PLAYER_PID=""
SAMPLER_PID=""
WATCHDOG_PID=""
PLAYER_STATUS=""
CLEANUP_COMPLETE="false"

write_manifest() {
    local state="$1"
    local exit_code="${2:-}"
    MANIFEST_STATE="$state" MANIFEST_EXIT_CODE="$exit_code" \
    EXP_BAG="$BAG" EXP_START_OFFSET="$START_OFFSET" \
    EXP_DURATION="$DURATION" EXP_RATE="$RATE" EXP_DOMAIN_ID="$DOMAIN_ID" \
    EXP_STARTED_AT="$RUN_STARTED_AT" \
    EXP_CONFIG="$FASTLIO_CONFIG" EXP_CONFIG_SHA="$CONFIG_SHA256" \
    EXP_RUNTIME_CONFIG="$FASTLIO_CONFIG_SNAPSHOT" \
    EXP_FASTLIO_PARAMETERS="$FASTLIO_PARAMETERS_SNAPSHOT" \
    EXP_FASTLIO_PARAMETERS_SHA="$FASTLIO_PARAMETERS_SHA256" \
    EXP_METADATA_SHA="$METADATA_SHA256" EXP_LAUNCH_SHA="$LAUNCH_SHA256" \
    EXP_GRAPH_SOURCE="$GRAPH_SOURCE" EXP_GRAPH_SHA="$GRAPH_SHA256" \
    EXP_GRAPH_RUNTIME="$GRAPH_RUNTIME" EXP_GRAPH_RUNTIME_SHA="$GRAPH_RUNTIME_SHA256" \
    EXP_LIDAR_ROS_TYPE="$DETECTED_LIDAR_ROS_TYPE" \
    EXP_DETECTED_LIDAR_FORMAT="$DETECTED_LIDAR_FORMAT" \
    EXP_RESOLVED_LIDAR_FORMAT="$RESOLVED_LIDAR_FORMAT" \
    EXP_LIDAR_FORMAT_OVERRIDE="$LIDAR_FORMAT_OVERRIDE" \
    EXP_FASTLIO_LID_TOPIC="$FASTLIO_LID_TOPIC" \
    EXP_LIVOX_ADAPTER_ENABLED="$LIVOX_ADAPTER_ENABLED" \
    EXP_LIVOX_ADAPTER_INPUT_TOPIC="$LIVOX_ADAPTER_INPUT_TOPIC" \
    EXP_LIVOX_ADAPTER_OUTPUT_TOPIC="$LIVOX_ADAPTER_OUTPUT_TOPIC" \
    EXP_LIVOX_ADAPTER_DIAGNOSTICS_TOPIC="$LIVOX_ADAPTER_DIAGNOSTICS_TOPIC" \
    EXP_LIVOX_ADAPTER_MAX_DELTA="$LIVOX_ADAPTER_MAX_DELTA_SEC" \
    EXP_LIVOX_ADAPTER_MINIMUM_POINTS="$LIVOX_ADAPTER_MINIMUM_POINTS" \
    EXP_LIVOX_ADAPTER_SOURCE="$LIVOX_ADAPTER_SOURCE" \
    EXP_LIVOX_ADAPTER_SOURCE_SHA="$LIVOX_ADAPTER_SOURCE_SHA256" \
    EXP_LIVOX_ADAPTER_RUNTIME="$LIVOX_ADAPTER_RUNTIME" \
    EXP_LIVOX_ADAPTER_RUNTIME_SHA="$LIVOX_ADAPTER_RUNTIME_SHA256" \
    EXP_LIVOX_ADAPTER_DIAGNOSTICS="$LIVOX_ADAPTER_DIAGNOSTICS" \
    EXP_LIVOX_ADAPTER_DIAGNOSTICS_SHA="$LIVOX_ADAPTER_DIAGNOSTICS_SHA256" \
    EXP_DIAGNOSTICS_EXTRACTOR_SOURCE="$DIAGNOSTICS_EXTRACTOR_SOURCE" \
    EXP_DIAGNOSTICS_EXTRACTOR_SNAPSHOT="$DIAGNOSTICS_EXTRACTOR_SNAPSHOT" \
    EXP_DIAGNOSTICS_EXTRACTOR_SHA="$DIAGNOSTICS_EXTRACTOR_SHA256" \
    EXP_DIAGNOSTICS_LOG="$DIAGNOSTICS_LOG" \
    EXP_GIT_COMMIT="$GIT_COMMIT" EXP_OUTPUT="$OUTPUT_DIR" \
    EXP_WORKSPACE_SETUP="$WORKSPACE_SETUP" \
    EXP_LAUNCH_SOURCE="$LAUNCH_SOURCE" EXP_LAUNCH_RUNTIME="$LAUNCH_RUNTIME" \
    EXP_FASTLIO_RUNTIME="$FASTLIO_RUNTIME" \
    EXP_FASTLIO_RUNTIME_SHA="$FASTLIO_RUNTIME_SHA256" \
    EXP_ODOM_RUNTIME="$ODOM_ADAPTER_RUNTIME" \
    EXP_ODOM_RUNTIME_SHA="$ODOM_ADAPTER_RUNTIME_SHA256" \
    EXP_LAUNCH_PID="$LAUNCH_PID" EXP_RECORDER_PID="$RECORDER_PID" \
    EXP_PLAYER_PID="$PLAYER_PID" EXP_SAMPLER_PID="$SAMPLER_PID" \
    EXP_ANALYZE="$ANALYZE" EXP_MAP_VOXEL_SIZE="$MAP_VOXEL_SIZE" \
    EXP_PREVIEW_MAX_POINTS="$PREVIEW_MAX_POINTS" \
    EXP_PLANE_RANDOM_SEED="$PLANE_RANDOM_SEED" \
    EXP_ANALYZER_SOURCE="$ANALYZER_SOURCE" \
    EXP_ANALYZER_SNAPSHOT="$ANALYZER_SNAPSHOT" \
    EXP_ANALYZER_SHA="$ANALYZER_SHA256" \
    EXP_ANALYSIS_LOG="$ANALYSIS_LOG" \
    EXP_RESULT_METADATA_SHA="$RESULT_METADATA_SHA256" \
    EXP_RESOURCE_METRICS_SHA="$RESOURCE_METRICS_SHA256" \
    EXP_RESOURCE_SUMMARY_SHA="$RESOURCE_SUMMARY_SHA256" \
    EXP_PARAMETER_DUMP_LOG="$PARAMETER_DUMP_LOG" \
    EXP_PARAMETER_VALIDATION_LOG="$PARAMETER_VALIDATION_LOG" \
    EXP_RESULT_VALIDATION_LOG="$RESULT_BAG_VALIDATION_LOG" \
    EXP_RESOURCE_VALIDATION_LOG="$RESOURCE_VALIDATION_LOG" \
    EXP_DRAIN_STABLE_SECONDS="$DRAIN_STABLE_SECONDS" \
    EXP_DRAIN_TIMEOUT_SECONDS="$DRAIN_TIMEOUT_SECONDS" \
    EXP_DRAIN_POLL_SECONDS="$DRAIN_POLL_SECONDS" \
    EXP_DRAIN_OUTCOME="$DRAIN_OUTCOME" \
    EXP_DRAIN_ELAPSED_SECONDS="$DRAIN_ELAPSED_SECONDS" \
    EXP_DRAIN_FINAL_SIZE_BYTES="$DRAIN_FINAL_SIZE_BYTES" \
    EXP_DRAIN_FINAL_MTIME_NS="$DRAIN_FINAL_MTIME_NS" \
    python3 - "$MANIFEST" <<'PY'
import json
import os
import pathlib
import sys
from datetime import datetime, timezone

env = os.environ

def optional_float(value):
    return float(value) if value else None

def optional_int(value):
    return int(value) if value else None

def optional_name(value):
    return pathlib.Path(value).name if value else None

analysis_enabled = env["EXP_ANALYZE"] == "true"
analysis_details = {
    "enabled": analysis_enabled,
    "voxel_size_m": None,
    "preview_max_points": None,
    "plane_random_seed": None,
    "analyzer_source": None,
    "analyzer_snapshot": None,
    "analyzer_sha256": None,
    "log": None,
    "artifacts": None,
}
if analysis_enabled:
    analysis_details.update(
        {
            "voxel_size_m": float(env["EXP_MAP_VOXEL_SIZE"]),
            "preview_max_points": int(env["EXP_PREVIEW_MAX_POINTS"]),
            "plane_random_seed": int(env["EXP_PLANE_RANDOM_SEED"]),
            "analyzer_source": env["EXP_ANALYZER_SOURCE"],
            "analyzer_snapshot": optional_name(env["EXP_ANALYZER_SNAPSHOT"]),
            "analyzer_sha256": env["EXP_ANALYZER_SHA"],
            "log": optional_name(env["EXP_ANALYSIS_LOG"]),
            "artifacts": {
                "summary": "summary.json",
                "map": "map_voxelized.pcd",
                "map_preview": "map_preview.pcd",
                "primary_trajectory": "trajectory.csv",
                "camera_init_trajectory": "trajectory_camera_init.csv",
            },
        }
    )

adapter_enabled = env["EXP_LIVOX_ADAPTER_ENABLED"] == "true"
adapter_diagnostics = None
adapter_diagnostics_path = env.get("EXP_LIVOX_ADAPTER_DIAGNOSTICS", "")
if adapter_diagnostics_path and pathlib.Path(adapter_diagnostics_path).is_file():
    try:
        adapter_diagnostics = json.loads(
            pathlib.Path(adapter_diagnostics_path).read_text(encoding="utf-8")
        )
    except (OSError, json.JSONDecodeError) as error:
        adapter_diagnostics = {"read_error": str(error)}

runtime_executables = {
    "fastlio": {
        "path": env["EXP_FASTLIO_RUNTIME"],
        "sha256": env["EXP_FASTLIO_RUNTIME_SHA"],
    },
    "odom_adapter": {
        "path": env["EXP_ODOM_RUNTIME"],
        "sha256": env["EXP_ODOM_RUNTIME_SHA"],
    },
}
if adapter_enabled:
    runtime_executables["livox_adapter"] = {
        "path": env["EXP_LIVOX_ADAPTER_RUNTIME"],
        "sha256": env["EXP_LIVOX_ADAPTER_RUNTIME_SHA"],
    }

run_logs = [
    "launch.log",
    "recorder.log",
    "player.log",
    "resume.log",
    "metrics.log",
    optional_name(env["EXP_PARAMETER_DUMP_LOG"]),
    optional_name(env["EXP_PARAMETER_VALIDATION_LOG"]),
    optional_name(env["EXP_RESULT_VALIDATION_LOG"]),
    optional_name(env["EXP_RESOURCE_VALIDATION_LOG"]),
]
if analysis_enabled:
    run_logs.append(optional_name(env["EXP_ANALYSIS_LOG"]))
if adapter_enabled:
    run_logs.append(optional_name(env["EXP_DIAGNOSTICS_LOG"]))


document = {
    "state": env["MANIFEST_STATE"],
    "exit_code": optional_int(env.get("MANIFEST_EXIT_CODE", "")),
    "started_at_utc": env["EXP_STARTED_AT"],
    "updated_at_utc": datetime.now(timezone.utc).replace(microsecond=0).isoformat(),
    "bag": {
        "path": env["EXP_BAG"],
        "metadata_sha256": env["EXP_METADATA_SHA"],
        "lidar_topic": "/livox/lidar",
        "lidar_ros_type": env["EXP_LIDAR_ROS_TYPE"],
        "detected_lidar_format": env["EXP_DETECTED_LIDAR_FORMAT"],
    },
    "profile": "baseline",
    "playback": {
        "topics": ["/livox/lidar", "/livox/imu"],
        "start_offset_s": float(env["EXP_START_OFFSET"]),
        "duration_s": optional_float(env["EXP_DURATION"]),
        "rate": float(env["EXP_RATE"]),
        "ros_domain_id": int(env["EXP_DOMAIN_ID"]),
    },
    "drain": {
        "stable_seconds": float(env["EXP_DRAIN_STABLE_SECONDS"]),
        "timeout_seconds": float(env["EXP_DRAIN_TIMEOUT_SECONDS"]),
        "poll_seconds": float(env["EXP_DRAIN_POLL_SECONDS"]),
        "outcome": env["EXP_DRAIN_OUTCOME"],
        "elapsed_seconds": optional_float(env["EXP_DRAIN_ELAPSED_SECONDS"]),
        "final_storage_size_bytes": optional_int(env["EXP_DRAIN_FINAL_SIZE_BYTES"]),
        "final_storage_mtime_ns": optional_int(env["EXP_DRAIN_FINAL_MTIME_NS"]),
    },
    "fastlio": {
        "config_source": env["EXP_CONFIG"],
        "config_snapshot": "fastlio_config.yaml",
        "runtime_config": env["EXP_RUNTIME_CONFIG"],
        "config_sha256": env["EXP_CONFIG_SHA"],
        "parameters_snapshot": pathlib.Path(env["EXP_FASTLIO_PARAMETERS"]).name,
        "parameters_sha256": env.get("EXP_FASTLIO_PARAMETERS_SHA") or None,
        "lid_topic": env["EXP_FASTLIO_LID_TOPIC"],
        "map_en": False,
        "path_en": False,
        "effect_map_en": False,
        "scan_bodyframe_pub_en": False,
        "scan_publish_en": True,
        "dense_publish_en": False,
        "pcd_save_en": False,
        "runtime_pos_log_enable": False,
    },
    "livox_input": {
        "format_override": env["EXP_LIDAR_FORMAT_OVERRIDE"],
        "detected_format": env["EXP_DETECTED_LIDAR_FORMAT"],
        "resolved_format": env["EXP_RESOLVED_LIDAR_FORMAT"],
        "ros_type": env["EXP_LIDAR_ROS_TYPE"],
    },
    "livox_adapter": {
        "enabled": adapter_enabled,
        "input_topic": env["EXP_LIVOX_ADAPTER_INPUT_TOPIC"] if adapter_enabled else None,
        "output_topic": env["EXP_LIVOX_ADAPTER_OUTPUT_TOPIC"] if adapter_enabled else None,
        "diagnostics_topic": (
            env["EXP_LIVOX_ADAPTER_DIAGNOSTICS_TOPIC"] if adapter_enabled else None
        ),
        "config": (
            {
                "max_point_header_delta_sec": float(env["EXP_LIVOX_ADAPTER_MAX_DELTA"]),
                "minimum_points": int(env["EXP_LIVOX_ADAPTER_MINIMUM_POINTS"]),
            }
            if adapter_enabled
            else None
        ),
        "source": env.get("EXP_LIVOX_ADAPTER_SOURCE") or None,
        "source_sha256": env.get("EXP_LIVOX_ADAPTER_SOURCE_SHA") or None,
        "runtime": env.get("EXP_LIVOX_ADAPTER_RUNTIME") or None,
        "runtime_sha256": env.get("EXP_LIVOX_ADAPTER_RUNTIME_SHA") or None,
        "diagnostics_path": optional_name(adapter_diagnostics_path),
        "diagnostics_sha256": env.get("EXP_LIVOX_ADAPTER_DIAGNOSTICS_SHA") or None,
        "diagnostics": adapter_diagnostics,
        "extractor_source": env.get("EXP_DIAGNOSTICS_EXTRACTOR_SOURCE") or None,
        "extractor_snapshot": optional_name(
            env.get("EXP_DIAGNOSTICS_EXTRACTOR_SNAPSHOT", "")
        ),
        "extractor_sha256": env.get("EXP_DIAGNOSTICS_EXTRACTOR_SHA") or None,
    },
    "analysis": analysis_details,
    "git": {
        "commit": env["EXP_GIT_COMMIT"],
        "status_file": "git-status.txt",
    },
    "workspace_setup": env["EXP_WORKSPACE_SETUP"],
    "launch_sha256": env["EXP_LAUNCH_SHA"],
    "launch_source": env["EXP_LAUNCH_SOURCE"],
    "launch_runtime": env["EXP_LAUNCH_RUNTIME"],
    "livox_graph": {
        "source": env["EXP_GRAPH_SOURCE"],
        "source_sha256": env["EXP_GRAPH_SHA"],
        "runtime": env["EXP_GRAPH_RUNTIME"],
        "runtime_sha256": env["EXP_GRAPH_RUNTIME_SHA"],
    },
    "runtime_executables": runtime_executables,
    "process_ids": {
        "launch": optional_int(env["EXP_LAUNCH_PID"]),
        "recorder": optional_int(env["EXP_RECORDER_PID"]),
        "player": optional_int(env["EXP_PLAYER_PID"]),
        "sampler": optional_int(env["EXP_SAMPLER_PID"]),
    },
    "artifacts": {
        "recorded_bag": "rosbag",
        "recorded_bag_metadata_sha256": env.get("EXP_RESULT_METADATA_SHA") or None,
        "resource_metrics": "resource_metrics.csv",
        "resource_metrics_sha256": env.get("EXP_RESOURCE_METRICS_SHA") or None,
        "resource_summary": "resource_summary.json",
        "resource_summary_sha256": env.get("EXP_RESOURCE_SUMMARY_SHA") or None,
        "commands": "commands.log",
        "logs": run_logs,
        "livox_adapter_diagnostics": optional_name(adapter_diagnostics_path),
        "livox_adapter_diagnostics_sha256": (
            env.get("EXP_LIVOX_ADAPTER_DIAGNOSTICS_SHA") or None
        ),
    },
}
path = pathlib.Path(sys.argv[1])
temporary = path.with_suffix(".json.tmp")
temporary.write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")
temporary.replace(path)
PY
}

process_group_alive() {
    local pgid="${1:-}"
    local stat_file record state ppid process_group
    [ -n "$pgid" ] || return 1
    for stat_file in /proc/[0-9]*/stat; do
        IFS= read -r record 2>/dev/null < "$stat_file" || continue
        record="${record##*) }"
        read -r state ppid process_group _ <<<"$record"
        [ "$process_group" = "$pgid" ] || continue
        case "$state" in
            Z|X) ;;
            *) return 0 ;;
        esac
    done
    return 1
}

stop_group() {
    local pid="${1:-}"
    local name="${2:-process}"
    [ -n "$pid" ] || return 0
    if process_group_alive "$pid"; then
        kill -INT -- "-$pid" 2>/dev/null || true
        for _ in $(seq 1 50); do
            process_group_alive "$pid" || break
            sleep 0.2
        done
        if process_group_alive "$pid"; then
            echo "Escalating $name cleanup to SIGTERM." >&2
            kill -TERM -- "-$pid" 2>/dev/null || true
            for _ in $(seq 1 25); do
                process_group_alive "$pid" || break
                sleep 0.2
            done
        fi
        if process_group_alive "$pid"; then
            echo "Escalating $name cleanup to SIGKILL." >&2
            kill -KILL -- "-$pid" 2>/dev/null || true
        fi
    fi
    wait "$pid" 2>/dev/null || true
}

cleanup() {
    local status=$?
    trap - EXIT INT TERM
    set +e
    if [ "$CLEANUP_COMPLETE" != "true" ]; then
        [ -z "$WATCHDOG_PID" ] || kill "$WATCHDOG_PID" 2>/dev/null || true
        stop_group "$PLAYER_PID" "rosbag player"
        stop_group "$RECORDER_PID" "rosbag recorder"
        stop_group "$LAUNCH_PID" "processing launch"
        touch "$STOP_FILE"
        if [ -n "$SAMPLER_PID" ]; then
            for _ in $(seq 1 20); do
                kill -0 "$SAMPLER_PID" 2>/dev/null || break
                sleep 0.1
            done
            stop_group "$SAMPLER_PID" "resource sampler"
        fi
        write_manifest "failed" "$status" || true
    fi
    exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

wait_for_node() {
    local expected="$1"
    local deadline=$((SECONDS + 90))
    while [ "$SECONDS" -lt "$deadline" ]; do
        kill -0 "$LAUNCH_PID" 2>/dev/null || return 1
        if ros2 node list --no-daemon 2>/dev/null | grep -Fxq "$expected"; then
            return 0
        fi
        sleep 0.5
    done
    return 1
}

wait_for_service() {
    local expected="$1"
    local deadline=$((SECONDS + 90))
    while [ "$SECONDS" -lt "$deadline" ]; do
        kill -0 "$PLAYER_PID" 2>/dev/null || return 1
        if ros2 service list --no-daemon 2>/dev/null | grep -Fxq "$expected"; then
            return 0
        fi
        sleep 0.5
    done
    return 1
}

wait_for_topic() {
    local topic="$1"
    local minimum_publishers="$2"
    local minimum_subscribers="$3"
    local deadline=$((SECONDS + 90))
    local info publishers subscribers
    while [ "$SECONDS" -lt "$deadline" ]; do
        kill -0 "$LAUNCH_PID" 2>/dev/null || return 1
        kill -0 "$RECORDER_PID" 2>/dev/null || return 1
        kill -0 "$PLAYER_PID" 2>/dev/null || return 1
        info="$(ros2 topic info --no-daemon "$topic" 2>/dev/null || true)"
        publishers="$(awk '/Publisher count:/{print $3}' <<<"$info")"
        subscribers="$(awk '/Subscription count:/{print $3}' <<<"$info")"
        if [ "${publishers:-0}" -ge "$minimum_publishers" ] \
            && [ "${subscribers:-0}" -ge "$minimum_subscribers" ]; then
            return 0
        fi
        sleep 0.5
    done
    return 1
}

wait_for_topic_type() {
    local topic="$1"
    local expected_type="$2"
    local minimum_publishers="$3"
    local minimum_subscribers="$4"
    local deadline=$((SECONDS + 90))
    local info actual_type publishers subscribers
    while [ "$SECONDS" -lt "$deadline" ]; do
        kill -0 "$LAUNCH_PID" 2>/dev/null || return 1
        kill -0 "$RECORDER_PID" 2>/dev/null || return 1
        kill -0 "$PLAYER_PID" 2>/dev/null || return 1
        info="$(ros2 topic info --no-daemon -v "$topic" 2>/dev/null || true)"
        actual_type="$(awk '/^Type:/{print $2; exit}' <<<"$info")"
        publishers="$(awk '/^Publisher count:/{print $3; exit}' <<<"$info")"
        subscribers="$(awk '/^Subscription count:/{print $3; exit}' <<<"$info")"
        if [ "$actual_type" = "$expected_type" ] \
            && [ "${publishers:-0}" -ge "$minimum_publishers" ] \
            && [ "${subscribers:-0}" -ge "$minimum_subscribers" ]; then
            return 0
        fi
        sleep 0.5
    done
    return 1
}

dump_node_parameters() {
    local node="$1"
    local expected_file="$2"
    local deadline=$((SECONDS + 30))
    while [ "$SECONDS" -lt "$deadline" ]; do
        kill -0 "$LAUNCH_PID" 2>/dev/null || return 1
        if ros2 param dump --no-daemon --output-dir "$OUTPUT_DIR" "$node" \
            >> "$PARAMETER_DUMP_LOG" 2>&1 \
            && [ -s "$expected_file" ]; then
            return 0
        fi
        sleep 0.5
    done
    return 1
}

assert_fastlio_parameter() {
    local name="$1"
    local expected="$2"
    local actual=""
    local deadline=$((SECONDS + 10))
    while [ "$SECONDS" -lt "$deadline" ]; do
        kill -0 "$LAUNCH_PID" 2>/dev/null || break
        if actual="$(ros2 param get --no-daemon --hide-type /fastlio_mapping "$name" 2>/dev/null)"; then
            break
        fi
        actual=""
        sleep 0.2
    done
    if [ -z "$actual" ]; then
        echo "Could not read FAST-LIO parameter $name." >&2
        return 1
    fi
    if [ "$actual" != "$expected" ]; then
        echo "FAST-LIO parameter $name is '$actual', expected '$expected'." >&2
        return 1
    fi
}

validate_fastlio_parameters() {
    assert_fastlio_parameter common.lid_topic "$FASTLIO_LID_TOPIC" || return 1
    assert_fastlio_parameter preprocess.scan_line 4 || return 1
    assert_fastlio_parameter publish.map_en False || return 1
    assert_fastlio_parameter publish.path_en False || return 1
    assert_fastlio_parameter publish.effect_map_en False || return 1
    assert_fastlio_parameter publish.scan_bodyframe_pub_en False || return 1
    assert_fastlio_parameter publish.scan_publish_en True || return 1
    assert_fastlio_parameter publish.dense_publish_en False || return 1
    assert_fastlio_parameter pcd_save.pcd_save_en False || return 1
    assert_fastlio_parameter runtime_pos_log_enable False || return 1
}

required_processing_processes_alive() {
    local required=(fastlio_mapping fastlio_odom_adapter)
    local process_name
    if [ "$LIVOX_ADAPTER_ENABLED" = "true" ]; then
        required+=(livox_pointcloud_adapter)
    fi

    for process_name in "${required[@]}"; do
        pgrep -g "$LAUNCH_PID" -f -- "$process_name" >/dev/null || return 1
    done
    return 0
}

bag_storage_signature() {
    python3 - "$RESULT_BAG" <<'PY'
from pathlib import Path
import sys

root = Path(sys.argv[1])
if not root.is_dir():
    print("0 0")
    raise SystemExit(0)

stats = []
for path in root.rglob("*"):
    try:
        if path.is_file():
            stats.append(path.stat())
    except FileNotFoundError:
        continue
print(sum(stat.st_size for stat in stats), max((stat.st_mtime_ns for stat in stats), default=0))
PY
}

wait_for_processing_quiescence() {
    local started_at="$SECONDS"
    local stable_since="$SECONDS"
    local previous_signature=""
    local signature current_size current_mtime elapsed stable_elapsed
    local process_misses=0

    DRAIN_OUTCOME="waiting"
    DRAIN_ELAPSED_SECONDS="0"
    echo "Waiting for result bag writes to remain unchanged for ${DRAIN_STABLE_SECONDS}s..."

    while [ $((SECONDS - started_at)) -lt "$DRAIN_TIMEOUT_SECONDS" ]; do
        if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
            DRAIN_OUTCOME="processing_launch_exited"
            DRAIN_ELAPSED_SECONDS="$((SECONDS - started_at))"
            echo "Processing launch exited while waiting for quiescence." >&2
            return 1
        fi
        if ! kill -0 "$RECORDER_PID" 2>/dev/null; then
            DRAIN_OUTCOME="recorder_exited"
            DRAIN_ELAPSED_SECONDS="$((SECONDS - started_at))"
            echo "Rosbag recorder exited while waiting for quiescence." >&2
            return 1
        fi
        if ! kill -0 "$SAMPLER_PID" 2>/dev/null; then
            DRAIN_OUTCOME="resource_sampler_exited"
            DRAIN_ELAPSED_SECONDS="$((SECONDS - started_at))"
            echo "Resource sampler exited while waiting for quiescence." >&2
            return 1
        fi

        if required_processing_processes_alive; then
            process_misses=0
        else
            process_misses=$((process_misses + 1))
            if [ "$process_misses" -ge 3 ]; then
                DRAIN_OUTCOME="processing_process_missing"
                DRAIN_ELAPSED_SECONDS="$((SECONDS - started_at))"
                echo "Required processing process disappeared while waiting for quiescence." >&2
                return 1
            fi
        fi

        if ! signature="$(bag_storage_signature)"; then
            DRAIN_OUTCOME="storage_signature_failed"
            DRAIN_ELAPSED_SECONDS="$((SECONDS - started_at))"
            echo "Could not inspect result bag storage while waiting for quiescence." >&2
            return 1
        fi
        read -r current_size current_mtime <<<"$signature"
        if ! [[ "$current_size" =~ ^[0-9]+$ && "$current_mtime" =~ ^[0-9]+$ ]]; then
            DRAIN_OUTCOME="invalid_storage_signature"
            DRAIN_ELAPSED_SECONDS="$((SECONDS - started_at))"
            echo "Result bag storage signature is invalid: $signature" >&2
            return 1
        fi

        DRAIN_FINAL_SIZE_BYTES="$current_size"
        DRAIN_FINAL_MTIME_NS="$current_mtime"
        elapsed=$((SECONDS - started_at))
        DRAIN_ELAPSED_SECONDS="$elapsed"

        if [ -n "$previous_signature" ] && [ "$signature" = "$previous_signature" ]; then
            stable_elapsed=$((SECONDS - stable_since))
        else
            previous_signature="$signature"
            stable_since="$SECONDS"
            stable_elapsed=0
        fi

        if [ "$current_size" -gt 0 ] && [ "$stable_elapsed" -ge "$DRAIN_STABLE_SECONDS" ]; then
            DRAIN_OUTCOME="quiescent"
            echo "Result bag storage is quiescent after ${elapsed}s."
            return 0
        fi
        sleep "$DRAIN_POLL_SECONDS"
    done

    DRAIN_OUTCOME="timeout"
    DRAIN_ELAPSED_SECONDS="$((SECONDS - started_at))"
    echo "Result bag did not remain unchanged for ${DRAIN_STABLE_SECONDS}s within the ${DRAIN_TIMEOUT_SECONDS}s timeout." >&2
    return 1
}

validate_result_bag() {
    local metadata="$RESULT_BAG/metadata.yaml"
    [ -s "$metadata" ] || {
        echo "Finalized result bag metadata is missing or empty: $metadata" >&2
        return 1
    }

    echo "ros2 bag info:"
    ros2 bag info "$RESULT_BAG"
    echo
    echo "metadata and reader validation:"
    python3 - "$RESULT_BAG" "$LIVOX_ADAPTER_ENABLED" <<'PY'
from pathlib import Path
import json
import sys

import rosbag2_py
import yaml

root = Path(sys.argv[1]).resolve()
adapter_enabled = sys.argv[2] == "true"
metadata_path = root / "metadata.yaml"

try:
    document = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
except (OSError, yaml.YAMLError) as exc:
    raise SystemExit(f"could not parse finalized metadata: {exc}") from exc

if not isinstance(document, dict):
    raise SystemExit("finalized metadata is not a mapping")
information = document.get("rosbag2_bagfile_information", document)
if not isinstance(information, dict):
    raise SystemExit("rosbag2_bagfile_information is not a mapping")

storage_id = information.get("storage_identifier")
if not isinstance(storage_id, str) or not storage_id:
    raise SystemExit("metadata has no storage identifier")

relative_paths = information.get("relative_file_paths")
if not isinstance(relative_paths, list) or not relative_paths:
    raise SystemExit("metadata has no storage files")
storage_files = []
for value in relative_paths:
    relative = Path(str(value))
    if relative.is_absolute() or ".." in relative.parts:
        raise SystemExit(f"unsafe storage path in metadata: {value!r}")
    storage_path = (root / relative).resolve()
    try:
        storage_path.relative_to(root)
    except ValueError as exc:
        raise SystemExit(f"storage path escapes result bag: {value!r}") from exc
    if not storage_path.is_file() or storage_path.stat().st_size <= 0:
        raise SystemExit(f"storage file is missing or empty: {relative}")
    storage_files.append(
        {"path": str(relative), "size_bytes": storage_path.stat().st_size}
    )

topics = information.get("topics_with_message_count")
if not isinstance(topics, list):
    raise SystemExit("metadata has no topic message counts")
counts = {}
types = {}
for item in topics:
    if not isinstance(item, dict):
        raise SystemExit("invalid topic message-count entry")
    topic_metadata = item.get("topic_metadata")
    if not isinstance(topic_metadata, dict):
        raise SystemExit("topic entry has no topic_metadata")
    name = topic_metadata.get("name")
    type_name = topic_metadata.get("type")
    if not isinstance(name, str) or not name:
        raise SystemExit("topic entry has no valid name")
    try:
        count = int(item.get("message_count"))
    except (TypeError, ValueError) as exc:
        raise SystemExit(f"invalid message count for {name}") from exc
    if count < 0:
        raise SystemExit(f"negative message count for {name}")
    counts[name] = counts.get(name, 0) + count
    types[name] = type_name

try:
    total_count = int(information.get("message_count"))
except (TypeError, ValueError) as exc:
    raise SystemExit("metadata has no valid total message count") from exc
if total_count <= 0:
    raise SystemExit("result bag contains no messages")
if total_count != sum(counts.values()):
    raise SystemExit(
        f"metadata total message count {total_count} does not match topic sum {sum(counts.values())}"
    )

required = ["/odom", "/Odometry", "/cloud_registered"]
if adapter_enabled:
    required.append("/fastlio_go2w_livox/diagnostics")
missing = [name for name in required if counts.get(name, 0) <= 0]
if missing:
    raise SystemExit(
        "required result topics have no recorded messages: " + ", ".join(missing)
    )

reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri=str(root), storage_id=storage_id),
    rosbag2_py.ConverterOptions("", ""),
)
reader_topics = {entry.name for entry in reader.get_all_topics_and_types()}
missing_from_reader = sorted(set(required) - reader_topics)
if missing_from_reader:
    raise SystemExit(
        "required topics are not readable through rosbag2: "
        + ", ".join(missing_from_reader)
    )
if not reader.has_next():
    raise SystemExit("rosbag2 reader opened the result bag but found no messages")
first_topic, first_data, first_timestamp = reader.read_next()
if not first_data:
    raise SystemExit("the first serialized result message is empty")

print(
    json.dumps(
        {
            "storage_id": storage_id,
            "storage_files": storage_files,
            "total_messages": total_count,
            "required_topic_counts": {name: counts[name] for name in required},
            "required_topic_types": {name: types.get(name) for name in required},
            "first_readable_message": {
                "topic": first_topic,
                "timestamp_ns": first_timestamp,
                "serialized_bytes": len(first_data),
            },
        },
        indent=2,
        sort_keys=True,
    )
)
PY
}

validate_resource_artifacts() {
    python3 - "$METRICS_CSV" "$METRICS_SUMMARY" "$LIVOX_ADAPTER_ENABLED" <<'PY'
from collections import Counter
from pathlib import Path
import csv
import json
import sys

csv_path = Path(sys.argv[1])
summary_path = Path(sys.argv[2])
required = {"fastlio", "player", "recorder"}
if sys.argv[3] == "true":
    required.add("livox_adapter")

if not csv_path.is_file() or csv_path.stat().st_size <= 0:
    raise SystemExit(f"resource metrics CSV is missing or empty: {csv_path}")
if not summary_path.is_file() or summary_path.stat().st_size <= 0:
    raise SystemExit(f"resource summary is missing or empty: {summary_path}")

counts = Counter()
with csv_path.open(newline="", encoding="utf-8") as stream:
    reader = csv.DictReader(stream)
    required_columns = {"wall_time_s", "pid", "process", "cpu_time_s", "rss_kib"}
    missing_columns = required_columns - set(reader.fieldnames or ())
    if missing_columns:
        raise SystemExit(
            "resource metrics CSV is missing columns: "
            + ", ".join(sorted(missing_columns))
        )
    for line_number, row in enumerate(reader, start=2):
        label = row.get("process", "")
        if not label:
            raise SystemExit(f"resource metrics row {line_number} has no process label")
        try:
            wall_time = float(row["wall_time_s"])
            pid = int(row["pid"])
            cpu_time = float(row["cpu_time_s"])
            rss_kib = int(row["rss_kib"])
        except (TypeError, ValueError) as exc:
            raise SystemExit(
                f"resource metrics row {line_number} contains invalid numeric data"
            ) from exc
        if wall_time < 0 or pid <= 0 or cpu_time < 0 or rss_kib < 0:
            raise SystemExit(
                f"resource metrics row {line_number} contains out-of-range data"
            )
        counts[label] += 1

missing_rows = sorted(label for label in required if counts[label] <= 0)
if missing_rows:
    raise SystemExit(
        "resource metrics CSV has no samples for: " + ", ".join(missing_rows)
    )

try:
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
except (OSError, json.JSONDecodeError) as exc:
    raise SystemExit(f"could not parse resource summary: {exc}") from exc
if not isinstance(summary, dict):
    raise SystemExit("resource summary is not an object")
try:
    interval = float(summary["interval_s"])
    elapsed = float(summary["elapsed_wall_time_s"])
except (KeyError, TypeError, ValueError) as exc:
    raise SystemExit("resource summary timing fields are invalid") from exc
if interval <= 0 or elapsed <= 0:
    raise SystemExit("resource summary timing fields must be positive")

processes = summary.get("processes")
if not isinstance(processes, dict):
    raise SystemExit("resource summary has no processes object")
validated = {}
for label in sorted(required):
    details = processes.get(label)
    if not isinstance(details, dict):
        raise SystemExit(f"resource summary has no entry for {label}")
    try:
        samples = int(details["samples"])
    except (KeyError, TypeError, ValueError) as exc:
        raise SystemExit(f"resource summary sample count is invalid for {label}") from exc
    pids = details.get("pids")
    if samples <= 0 or not isinstance(pids, list) or not pids:
        raise SystemExit(f"resource summary has no usable samples for {label}")
    if samples != counts[label]:
        raise SystemExit(
            f"resource sample mismatch for {label}: CSV={counts[label]}, summary={samples}"
        )
    validated[label] = {"samples": samples, "pids": pids}

print(
    json.dumps(
        {
            "elapsed_wall_time_s": elapsed,
            "interval_s": interval,
            "processes": validated,
        },
        indent=2,
        sort_keys=True,
    )
)
PY
}

LAUNCH_CMD=(
    ros2 launch fastlio_go2w_bringup offline_fastlio.launch.py
    "config:=$FASTLIO_CONFIG_SNAPSHOT"
    "lidar_format:=$RESOLVED_LIDAR_FORMAT"
)
RECORD_TOPICS=(/odom /Odometry /cloud_registered)
if [ "$LIVOX_ADAPTER_ENABLED" = "true" ]; then
    RECORD_TOPICS+=("$LIVOX_ADAPTER_DIAGNOSTICS_TOPIC")
fi
RECORDER_CMD=(ros2 bag record --use-sim-time -o "$RESULT_BAG" "${RECORD_TOPICS[@]}")
PLAYER_CMD=(
    ros2 bag play "$BAG" --clock --rate "$RATE" --start-paused
    --disable-keyboard-controls --start-offset "$START_OFFSET"
    --topics /livox/lidar /livox/imu
)
ANALYZE_CMD=()
if [ "$ANALYZE" = "true" ]; then
    ANALYZE_CMD=(
        python3 "$ANALYZER_SNAPSHOT" analyze "$RESULT_BAG"
        --output-dir "$OUTPUT_DIR"
        --label baseline
        --voxel-size "$MAP_VOXEL_SIZE"
        --preview-max-points "$PREVIEW_MAX_POINTS"
        --plane-random-seed "$PLANE_RANDOM_SEED"
    )
fi
DIAGNOSTICS_CMD=()
if [ "$LIVOX_ADAPTER_ENABLED" = "true" ]; then
    DIAGNOSTICS_CMD=(
        python3 "$DIAGNOSTICS_EXTRACTOR_SNAPSHOT" "$RESULT_BAG"
        --output "$LIVOX_ADAPTER_DIAGNOSTICS"
        --topic "$LIVOX_ADAPTER_DIAGNOSTICS_TOPIC"
    )
fi
{
    printf 'launch:'; printf ' %q' "${LAUNCH_CMD[@]}"; printf '\n'
    printf 'record:'; printf ' %q' "${RECORDER_CMD[@]}"; printf '\n'
    printf 'play:'; printf ' %q' "${PLAYER_CMD[@]}"; printf '\n'
    if [ "$ANALYZE" = "true" ]; then
        printf 'analyze:'; printf ' %q' "${ANALYZE_CMD[@]}"; printf '\n'
    fi
    if [ "$LIVOX_ADAPTER_ENABLED" = "true" ]; then
        printf 'diagnostics:'; printf ' %q' "${DIAGNOSTICS_CMD[@]}"; printf '\n'
    fi
} > "$OUTPUT_DIR/commands.log"

write_manifest "starting"

echo "Starting headless FAST-LIO in ROS domain $ROS_DOMAIN_ID"
echo "Bag: $BAG"
echo "Output: $OUTPUT_DIR"
echo "Livox metadata type: $DETECTED_LIDAR_ROS_TYPE"
echo "Resolved Livox format: $RESOLVED_LIDAR_FORMAT"
echo "Livox adapter enabled: $LIVOX_ADAPTER_ENABLED"

setsid "${LAUNCH_CMD[@]}" > "$OUTPUT_DIR/launch.log" 2>&1 &
LAUNCH_PID=$!
setsid "${RECORDER_CMD[@]}" > "$OUTPUT_DIR/recorder.log" 2>&1 &
RECORDER_PID=$!
setsid "${PLAYER_CMD[@]}" > "$OUTPUT_DIR/player.log" 2>&1 &
PLAYER_PID=$!

wait_for_node /fastlio_mapping || die "FAST-LIO did not become ready; see launch.log"
wait_for_node /fastlio_odom_adapter || die "odom adapter did not become ready; see launch.log"
if [ "$LIVOX_ADAPTER_ENABLED" = "true" ]; then
    wait_for_node /livox_pointcloud_adapter \
        || die "Livox adapter did not become ready; see launch.log"
fi
: > "$PARAMETER_DUMP_LOG"
dump_node_parameters /fastlio_mapping "$FASTLIO_PARAMETERS_SNAPSHOT" \
    || die "FAST-LIO parameter snapshot was not created"
validate_fastlio_parameters \
    > "$PARAMETER_VALIDATION_LOG" 2>&1 \
    || {
        tail -n 40 "$PARAMETER_VALIDATION_LOG" >&2 || true
        die "FAST-LIO live parameters violate the offline artifact contract"
    }
FASTLIO_PARAMETERS_SHA256="$(sha256sum "$FASTLIO_PARAMETERS_SNAPSHOT" | awk '{print $1}')"
wait_for_service /rosbag2_player/resume || die "rosbag player resume service did not appear"

wait_for_topic_type /livox/lidar "$DETECTED_LIDAR_ROS_TYPE" 1 1 \
    || die "/livox/lidar endpoints did not become ready with type $DETECTED_LIDAR_ROS_TYPE"
if [ "$LIVOX_ADAPTER_ENABLED" = "true" ]; then
    wait_for_topic_type /livox/lidar_fastlio livox_ros_driver2/msg/CustomMsg 1 1 \
        || die "/livox/lidar_fastlio CustomMsg endpoints did not become ready"
    wait_for_topic_type "$LIVOX_ADAPTER_DIAGNOSTICS_TOPIC" \
        diagnostic_msgs/msg/DiagnosticArray 1 1 \
        || die "Livox adapter diagnostics endpoints did not become ready"
fi
wait_for_topic /livox/imu 1 1 || die "/livox/imu endpoints did not become ready"
wait_for_topic /Odometry 1 1 || die "/Odometry endpoints did not become ready"
wait_for_topic /odom 1 1 || die "/odom endpoints did not become ready"
wait_for_topic /cloud_registered 1 1 || die "/cloud_registered endpoints did not become ready"

SAMPLER_CMD=(
    python3 "$SCRIPT_DIR/sample_process_metrics.py"
    --target "fastlio:$LAUNCH_PID:fastlio_mapping"
    --target "player:$PLAYER_PID:*"
    --target "recorder:$RECORDER_PID:*"
    --output "$METRICS_CSV" --summary "$METRICS_SUMMARY"
    --stop-file "$STOP_FILE" --interval 1.0
)
if [ "$LIVOX_ADAPTER_ENABLED" = "true" ]; then
    SAMPLER_CMD+=(--target "livox_adapter:$LAUNCH_PID:livox_pointcloud_adapter")
fi
setsid "${SAMPLER_CMD[@]}" > "$OUTPUT_DIR/metrics.log" 2>&1 &
SAMPLER_PID=$!

RESUME_TYPE="rosbag2_interfaces/srv/Resume"
ros2 service call /rosbag2_player/resume "$RESUME_TYPE" "{}" \
    > "$OUTPUT_DIR/resume.log" 2>&1
write_manifest "running"

if [ -n "$DURATION" ]; then
    WATCHDOG_WALL_SECONDS="$(python3 -c 'import sys; print(float(sys.argv[1]) / float(sys.argv[2]))' "$DURATION" "$RATE")"
    (
        sleep "$WATCHDOG_WALL_SECONDS"
        touch "$DURATION_MARKER"
        kill -INT -- "-$PLAYER_PID" 2>/dev/null || true
    ) &
    WATCHDOG_PID=$!
fi

PROCESSING_PROCESS_MISSES=0
while kill -0 "$PLAYER_PID" 2>/dev/null; do
    kill -0 "$LAUNCH_PID" 2>/dev/null \
        || die "processing launch exited during playback; see launch.log"
    kill -0 "$RECORDER_PID" 2>/dev/null \
        || die "rosbag recorder exited during playback; see recorder.log"
    kill -0 "$SAMPLER_PID" 2>/dev/null \
        || die "resource sampler exited during playback; see metrics.log"
    if required_processing_processes_alive; then
        PROCESSING_PROCESS_MISSES=0
    else
        PROCESSING_PROCESS_MISSES=$((PROCESSING_PROCESS_MISSES + 1))
        [ "$PROCESSING_PROCESS_MISSES" -lt 3 ] \
            || die "required processing process disappeared during playback"
    fi
    sleep 1
done

set +e
wait "$PLAYER_PID"
PLAYER_STATUS=$?
set -e
if [ -n "$WATCHDOG_PID" ]; then
    kill "$WATCHDOG_PID" 2>/dev/null || true
    wait "$WATCHDOG_PID" 2>/dev/null || true
fi
if [ "$PLAYER_STATUS" -ne 0 ] && [ ! -f "$DURATION_MARKER" ]; then
    die "rosbag player exited with status $PLAYER_STATUS; see player.log"
fi
kill -0 "$LAUNCH_PID" 2>/dev/null || die "processing launch exited before drain"
kill -0 "$RECORDER_PID" 2>/dev/null || die "rosbag recorder exited before drain"
kill -0 "$SAMPLER_PID" 2>/dev/null || die "resource sampler exited before drain"
required_processing_processes_alive \
    || die "required processing process missing before drain"

write_manifest "draining"
if ! wait_for_processing_quiescence; then
    die "processing drain failed ($DRAIN_OUTCOME); result bag never became safely quiescent"
fi
write_manifest "drained"

stop_group "$RECORDER_PID" "rosbag recorder"
stop_group "$LAUNCH_PID" "processing launch"
touch "$STOP_FILE"
for _ in $(seq 1 30); do
    kill -0 "$SAMPLER_PID" 2>/dev/null || break
    sleep 0.1
done
stop_group "$SAMPLER_PID" "resource sampler"

write_manifest "validating"
if ! validate_result_bag > "$RESULT_BAG_VALIDATION_LOG" 2>&1; then
    tail -n 40 "$RESULT_BAG_VALIDATION_LOG" >&2 || true
    die "finalized result bag validation failed; see $RESULT_BAG_VALIDATION_LOG"
fi
RESULT_METADATA_SHA256="$(sha256sum "$RESULT_BAG/metadata.yaml" | awk '{print $1}')"

if [ "$LIVOX_ADAPTER_ENABLED" = "true" ]; then
    if ! "${DIAGNOSTICS_CMD[@]}" > "$DIAGNOSTICS_LOG" 2>&1; then
        tail -n 40 "$DIAGNOSTICS_LOG" >&2 || true
        die "Livox adapter diagnostics extraction failed"
    fi
    [ -s "$LIVOX_ADAPTER_DIAGNOSTICS" ] \
        || die "Livox adapter diagnostics artifact was not created"
    LIVOX_ADAPTER_DIAGNOSTICS_SHA256="$(
        sha256sum "$LIVOX_ADAPTER_DIAGNOSTICS" | awk '{print $1}'
    )"
fi

if ! validate_resource_artifacts > "$RESOURCE_VALIDATION_LOG" 2>&1; then
    tail -n 40 "$RESOURCE_VALIDATION_LOG" >&2 || true
    die "resource artifact validation failed; see $RESOURCE_VALIDATION_LOG"
fi
RESOURCE_METRICS_SHA256="$(sha256sum "$METRICS_CSV" | awk '{print $1}')"
RESOURCE_SUMMARY_SHA256="$(sha256sum "$METRICS_SUMMARY" | awk '{print $1}')"

if [ "$ANALYZE" = "true" ]; then
    write_manifest "analyzing"
    echo "Generating final map and trajectory artifacts..."
    if ! "${ANALYZE_CMD[@]}" > "$ANALYSIS_LOG" 2>&1; then
        tail -n 40 "$ANALYSIS_LOG" >&2 || true
        die "artifact analysis failed; see $ANALYSIS_LOG"
    fi
    for artifact in summary.json map_voxelized.pcd map_preview.pcd trajectory.csv trajectory_camera_init.csv; do
        [ -s "$OUTPUT_DIR/$artifact" ] \
            || die "artifact analysis did not create $artifact"
    done
fi

rm -f "$STOP_FILE" "$DURATION_MARKER"
write_manifest "completed" 0
CLEANUP_COMPLETE="true"
trap - EXIT INT TERM

echo "Experiment complete: $OUTPUT_DIR"
