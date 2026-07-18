#!/usr/bin/env bash
# Run one reproducible offline FAST-LIO profile against a recorded ROS 2 bag.

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

usage() {
    cat <<'EOF'
Usage:
  scripts/offline/run_multilidar_experiment.sh BAG --profile PROFILE [options]

Profiles:
  baseline       MID-360 only, using the existing FAST-LIO point stride of 3
  fused-high     MID-360 stride 3 plus XT16 firing stride 3
  fused-matched  MID-360 stride 6 plus XT16 firing stride 22

Options:
  --start-offset SEC  Start this many seconds into the bag (default: 0)
  --duration SEC      Approximate bag seconds via wall timer (smoke tests only)
  --rate RATE         Rosbag playback multiplier (default: 1.0)
  --domain-id ID      Isolated ROS domain ID (default: 77)
  --output DIR        Result directory (default: results/multilidar/...)
  --debug-cloud       Publish and record /livox/lidar_fused_debug
  -h, --help          Show this help

The workspace must already be built. The runner plays only /livox/lidar,
/livox/imu, and /points_raw, starts playback paused, waits for every endpoint,
then resumes through the rosbag player service.
EOF
}

die() {
    echo "Error: $*" >&2
    exit 2
}

BAG=""
PROFILE=""
START_OFFSET="0"
DURATION=""
RATE="1.0"
DOMAIN_ID="77"
OUTPUT_DIR=""
DEBUG_CLOUD="false"

while [ "$#" -gt 0 ]; do
    case "$1" in
        --profile)
            PROFILE="${2:?Error: --profile requires a value}"
            shift 2
            ;;
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
        --debug-cloud)
            DEBUG_CLOUD="true"
            shift
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
case "$PROFILE" in
    baseline|fused-high|fused-matched) ;;
    "") die "--profile is required" ;;
    *) die "unknown profile '$PROFILE'" ;;
esac

python3 -c 'import sys; value=float(sys.argv[1]); sys.exit(not value >= 0)' "$START_OFFSET" \
    || die "--start-offset must be a non-negative number"
python3 -c 'import sys; value=float(sys.argv[1]); sys.exit(not value > 0)' "$RATE" \
    || die "--rate must be greater than zero"
if [ -n "$DURATION" ]; then
    python3 -c 'import sys; value=float(sys.argv[1]); sys.exit(not value > 0)' "$DURATION" \
        || die "--duration must be greater than zero"
fi
[[ "$DOMAIN_ID" =~ ^[0-9]+$ ]] || die "--domain-id must be a non-negative integer"

BAG="$(realpath "$BAG")"
[ -d "$BAG" ] || die "bag directory not found: $BAG"
[ -f "$BAG/metadata.yaml" ] || die "bag metadata not found: $BAG/metadata.yaml"
for topic in /livox/lidar /livox/imu /points_raw; do
    grep -Eq "name: ${topic}$" "$BAG/metadata.yaml" \
        || die "required topic $topic is absent from the bag"
done

if [ -z "$OUTPUT_DIR" ]; then
    BAG_NAME="$(basename "$BAG")"
    RUN_STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
    OUTPUT_DIR="$REPO_ROOT/results/multilidar/$BAG_NAME/${RUN_STAMP}-${PROFILE}"
else
    OUTPUT_DIR="$(realpath -m "$OUTPUT_DIR")"
fi
if [ -d "$OUTPUT_DIR" ] && [ -n "$(find "$OUTPUT_DIR" -mindepth 1 -print -quit)" ]; then
    die "output directory is not empty: $OUTPUT_DIR"
fi
mkdir -p "$OUTPUT_DIR"

if [ -f /opt/ros/humble/setup.bash ]; then
    set +u
    source /opt/ros/humble/setup.bash
    set -u
elif [ "${ROS_DISTRO:-}" != "humble" ]; then
    die "ROS 2 Humble is required (run this inside the project container)"
fi

WORKSPACE_SETUP=""
for candidate in \
    "$REPO_ROOT/humble_ws/install/setup.bash" \
    "$REPO_ROOT/.devcontainer/desktop_ws/install/setup.bash"; do
    if [ -f "$candidate" ]; then
        set +u
        source "$candidate"
        set -u
        if ! ros2 pkg prefix fastlio_go2w_bringup >/dev/null 2>&1; then
            continue
        fi
        if [ "$PROFILE" != "baseline" ] \
            && ! ros2 pkg prefix fastlio_go2w_fusion >/dev/null 2>&1; then
            continue
        fi
        WORKSPACE_SETUP="$candidate"
        break
    fi
done
[ -n "$WORKSPACE_SETUP" ] || die "workspace is not built (no Humble install overlay found)"
set +u
source "$WORKSPACE_SETUP"
set -u

for command in ros2 setsid python3 sha256sum; do
    command -v "$command" >/dev/null || die "required command not found: $command"
done
ros2 pkg prefix fastlio_go2w_bringup >/dev/null 2>&1 \
    || die "fastlio_go2w_bringup is absent from $WORKSPACE_SETUP; rebuild the workspace"
if [ "$PROFILE" != "baseline" ]; then
    ros2 pkg prefix fastlio_go2w_fusion >/dev/null 2>&1 \
        || die "fastlio_go2w_fusion is absent from $WORKSPACE_SETUP; rebuild the workspace"
fi

export ROS_DOMAIN_ID="$DOMAIN_ID"
export RCUTILS_COLORIZED_OUTPUT=0
export ROS_LOG_DIR="$OUTPUT_DIR/ros-logs"
mkdir -p "$ROS_LOG_DIR"

if [ "$PROFILE" = "baseline" ]; then
    FASTLIO_CONFIG="$REPO_ROOT/humble_ws/src/fastlio_go2w_bringup/config/mid360_offline_eval.yaml"
else
    FASTLIO_CONFIG="$REPO_ROOT/humble_ws/src/fastlio_go2w_bringup/config/mid360_xt16_fused.yaml"
fi
LAUNCH_SOURCE="$REPO_ROOT/humble_ws/src/fastlio_go2w_bringup/launch/offline_multilidar.launch.py"
FASTLIO_CONFIG_SNAPSHOT="$OUTPUT_DIR/fastlio_config.yaml"
BRINGUP_PREFIX="$(ros2 pkg prefix fastlio_go2w_bringup)"
LAUNCH_RUNTIME="$BRINGUP_PREFIX/share/fastlio_go2w_bringup/launch/offline_multilidar.launch.py"
[ -f "$LAUNCH_RUNTIME" ] \
    || die "installed experiment launch not found: $LAUNCH_RUNTIME"
LAUNCH_SHA256="$(sha256sum "$LAUNCH_SOURCE" | awk '{print $1}')"
LAUNCH_RUNTIME_SHA256="$(sha256sum "$LAUNCH_RUNTIME" | awk '{print $1}')"
[ "$LAUNCH_SHA256" = "$LAUNCH_RUNTIME_SHA256" ] \
    || die "installed experiment launch is stale; rebuild $WORKSPACE_SETUP"

FASTLIO_RUNTIME="$(ros2 pkg prefix fast_lio)/lib/fast_lio/fastlio_mapping"
ODOM_ADAPTER_RUNTIME="$BRINGUP_PREFIX/lib/fastlio_go2w_bringup/fastlio_odom_adapter"
[ -x "$FASTLIO_RUNTIME" ] || die "FAST-LIO executable not found: $FASTLIO_RUNTIME"
[ -x "$ODOM_ADAPTER_RUNTIME" ] \
    || die "odom adapter executable not found: $ODOM_ADAPTER_RUNTIME"
FASTLIO_RUNTIME_SHA256="$(sha256sum "$FASTLIO_RUNTIME" | awk '{print $1}')"
ODOM_ADAPTER_RUNTIME_SHA256="$(sha256sum "$ODOM_ADAPTER_RUNTIME" | awk '{print $1}')"
FUSION_RUNTIME=""
FUSION_RUNTIME_SHA256=""
if [ "$PROFILE" != "baseline" ]; then
    FUSION_RUNTIME="$(ros2 pkg prefix fastlio_go2w_fusion)/lib/fastlio_go2w_fusion/dual_lidar_fusion_node"
    [ -x "$FUSION_RUNTIME" ] \
        || die "fusion executable not found: $FUSION_RUNTIME"
    FUSION_RUNTIME_SHA256="$(sha256sum "$FUSION_RUNTIME" | awk '{print $1}')"
fi

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
FUSION_PARAMETERS_SNAPSHOT=""
FUSION_PARAMETERS_SHA256=""
if [ "$PROFILE" != "baseline" ]; then
    FUSION_PARAMETERS_SNAPSHOT="$OUTPUT_DIR/dual_lidar_fusion.yaml"
fi

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
    EXP_BAG="$BAG" EXP_PROFILE="$PROFILE" EXP_START_OFFSET="$START_OFFSET" \
    EXP_DURATION="$DURATION" EXP_RATE="$RATE" EXP_DOMAIN_ID="$DOMAIN_ID" \
    EXP_DEBUG_CLOUD="$DEBUG_CLOUD" EXP_STARTED_AT="$RUN_STARTED_AT" \
    EXP_CONFIG="$FASTLIO_CONFIG" EXP_CONFIG_SHA="$CONFIG_SHA256" \
    EXP_FUSION_PARAMETERS="$FUSION_PARAMETERS_SNAPSHOT" \
    EXP_FUSION_PARAMETERS_SHA="$FUSION_PARAMETERS_SHA256" \
    EXP_RUNTIME_CONFIG="$FASTLIO_CONFIG_SNAPSHOT" \
    EXP_METADATA_SHA="$METADATA_SHA256" EXP_LAUNCH_SHA="$LAUNCH_SHA256" \
    EXP_GIT_COMMIT="$GIT_COMMIT" EXP_OUTPUT="$OUTPUT_DIR" \
    EXP_WORKSPACE_SETUP="$WORKSPACE_SETUP" \
    EXP_LAUNCH_SOURCE="$LAUNCH_SOURCE" EXP_LAUNCH_RUNTIME="$LAUNCH_RUNTIME" \
    EXP_FASTLIO_RUNTIME="$FASTLIO_RUNTIME" \
    EXP_FASTLIO_RUNTIME_SHA="$FASTLIO_RUNTIME_SHA256" \
    EXP_ODOM_RUNTIME="$ODOM_ADAPTER_RUNTIME" \
    EXP_ODOM_RUNTIME_SHA="$ODOM_ADAPTER_RUNTIME_SHA256" \
    EXP_FUSION_RUNTIME="$FUSION_RUNTIME" \
    EXP_FUSION_RUNTIME_SHA="$FUSION_RUNTIME_SHA256" \
    EXP_LAUNCH_PID="$LAUNCH_PID" EXP_RECORDER_PID="$RECORDER_PID" \
    EXP_PLAYER_PID="$PLAYER_PID" EXP_SAMPLER_PID="$SAMPLER_PID" \
    python3 - "$MANIFEST" <<'PY'
import json
import os
import pathlib
import sys
from datetime import datetime, timezone

env = os.environ
profile = env["EXP_PROFILE"]
fusion = None
if profile != "baseline":
    fusion = {
        "parameters_snapshot": pathlib.Path(env["EXP_FUSION_PARAMETERS"]).name,
        "parameters_sha256": env.get("EXP_FUSION_PARAMETERS_SHA") or None,
    }

fusion_runtime = None
if env["EXP_FUSION_RUNTIME"]:
    fusion_runtime = {
        "path": env["EXP_FUSION_RUNTIME"],
        "sha256": env["EXP_FUSION_RUNTIME_SHA"],
    }

def optional_float(value):
    return float(value) if value else None

def optional_int(value):
    return int(value) if value else None

document = {
    "state": env["MANIFEST_STATE"],
    "exit_code": optional_int(env.get("MANIFEST_EXIT_CODE", "")),
    "started_at_utc": env["EXP_STARTED_AT"],
    "updated_at_utc": datetime.now(timezone.utc).replace(microsecond=0).isoformat(),
    "bag": {
        "path": env["EXP_BAG"],
        "metadata_sha256": env["EXP_METADATA_SHA"],
    },
    "profile": profile,
    "playback": {
        "topics": ["/livox/lidar", "/livox/imu", "/points_raw"],
        "start_offset_s": float(env["EXP_START_OFFSET"]),
        "duration_s": optional_float(env["EXP_DURATION"]),
        "rate": float(env["EXP_RATE"]),
        "ros_domain_id": int(env["EXP_DOMAIN_ID"]),
    },
    "fastlio": {
        "config_source": env["EXP_CONFIG"],
        "config_snapshot": "fastlio_config.yaml",
        "runtime_config": env["EXP_RUNTIME_CONFIG"],
        "config_sha256": env["EXP_CONFIG_SHA"],
        "map_en": False,
    },
    "fusion": fusion,
    "publish_debug_cloud": env["EXP_DEBUG_CLOUD"] == "true",
    "git": {
        "commit": env["EXP_GIT_COMMIT"],
        "status_file": "git-status.txt",
    },
    "workspace_setup": env["EXP_WORKSPACE_SETUP"],
    "launch_sha256": env["EXP_LAUNCH_SHA"],
    "launch_source": env["EXP_LAUNCH_SOURCE"],
    "launch_runtime": env["EXP_LAUNCH_RUNTIME"],
    "runtime_executables": {
        "fastlio": {
            "path": env["EXP_FASTLIO_RUNTIME"],
            "sha256": env["EXP_FASTLIO_RUNTIME_SHA"],
        },
        "odom_adapter": {
            "path": env["EXP_ODOM_RUNTIME"],
            "sha256": env["EXP_ODOM_RUNTIME_SHA"],
        },
        "fusion": fusion_runtime,
    },
    "process_ids": {
        "launch": optional_int(env["EXP_LAUNCH_PID"]),
        "recorder": optional_int(env["EXP_RECORDER_PID"]),
        "player": optional_int(env["EXP_PLAYER_PID"]),
        "sampler": optional_int(env["EXP_SAMPLER_PID"]),
    },
    "artifacts": {
        "recorded_bag": "rosbag",
        "resource_metrics": "resource_metrics.csv",
        "resource_summary": "resource_summary.json",
        "commands": "commands.log",
        "logs": ["launch.log", "recorder.log", "player.log", "resume.log", "metrics.log"],
    },
}
path = pathlib.Path(sys.argv[1])
temporary = path.with_suffix(".json.tmp")
temporary.write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")
temporary.replace(path)
PY
}

stop_group() {
    local pid="${1:-}"
    local name="${2:-process}"
    [ -n "$pid" ] || return 0
    if ! kill -0 "$pid" 2>/dev/null; then
        wait "$pid" 2>/dev/null || true
        return 0
    fi
    kill -INT -- "-$pid" 2>/dev/null || true
    for _ in $(seq 1 50); do
        kill -0 "$pid" 2>/dev/null || break
        sleep 0.2
    done
    if kill -0 "$pid" 2>/dev/null; then
        echo "Escalating $name cleanup to SIGTERM." >&2
        kill -TERM -- "-$pid" 2>/dev/null || true
        for _ in $(seq 1 25); do
            kill -0 "$pid" 2>/dev/null || break
            sleep 0.2
        done
    fi
    if kill -0 "$pid" 2>/dev/null; then
        echo "Escalating $name cleanup to SIGKILL." >&2
        kill -KILL -- "-$pid" 2>/dev/null || true
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
        if ros2 node list 2>/dev/null | grep -Fxq "$expected"; then
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
        if ros2 service list 2>/dev/null | grep -Fxq "$expected"; then
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
        info="$(ros2 topic info "$topic" 2>/dev/null || true)"
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

required_processing_nodes_alive() {
    local nodes
    local required=(/fastlio_mapping /fastlio_odom_adapter)
    if [ "$PROFILE" != "baseline" ]; then
        required+=(/dual_lidar_fusion)
    fi

    nodes="$(ros2 node list 2>/dev/null)" || return 1
    for node in "${required[@]}"; do
        grep -Fxq "$node" <<<"$nodes" || return 1
    done
    return 0
}

LAUNCH_CMD=(
    ros2 launch fastlio_go2w_bringup offline_multilidar.launch.py
    "profile:=$PROFILE" "config:=$FASTLIO_CONFIG_SNAPSHOT"
    "publish_debug_cloud:=$DEBUG_CLOUD"
)
RECORD_TOPICS=(/odom /Odometry /cloud_registered /fastlio_go2w_fusion/diagnostics)
if [ "$DEBUG_CLOUD" = "true" ]; then
    RECORD_TOPICS+=(/livox/lidar_fused_debug)
fi
RECORDER_CMD=(ros2 bag record --use-sim-time -o "$OUTPUT_DIR/rosbag" "${RECORD_TOPICS[@]}")
PLAYER_CMD=(
    ros2 bag play "$BAG" --clock --rate "$RATE" --start-paused
    --disable-keyboard-controls --start-offset "$START_OFFSET"
    --topics /livox/lidar /livox/imu /points_raw
)

{
    printf 'launch:'; printf ' %q' "${LAUNCH_CMD[@]}"; printf '\n'
    printf 'record:'; printf ' %q' "${RECORDER_CMD[@]}"; printf '\n'
    printf 'play:'; printf ' %q' "${PLAYER_CMD[@]}"; printf '\n'
} > "$OUTPUT_DIR/commands.log"

write_manifest "starting"

echo "Starting $PROFILE experiment in ROS domain $ROS_DOMAIN_ID"
echo "Bag: $BAG"
echo "Output: $OUTPUT_DIR"

setsid "${LAUNCH_CMD[@]}" > "$OUTPUT_DIR/launch.log" 2>&1 &
LAUNCH_PID=$!
setsid "${RECORDER_CMD[@]}" > "$OUTPUT_DIR/recorder.log" 2>&1 &
RECORDER_PID=$!
setsid "${PLAYER_CMD[@]}" > "$OUTPUT_DIR/player.log" 2>&1 &
PLAYER_PID=$!

wait_for_node /fastlio_mapping || die "FAST-LIO did not become ready; see launch.log"
wait_for_node /fastlio_odom_adapter || die "odom adapter did not become ready; see launch.log"
if [ "$PROFILE" != "baseline" ]; then
    wait_for_node /dual_lidar_fusion || die "fusion node did not become ready; see launch.log"
fi
wait_for_service /rosbag2_player/resume || die "rosbag player resume service did not appear"

wait_for_topic /livox/lidar 1 1 || die "/livox/lidar endpoints did not become ready"
wait_for_topic /livox/imu 1 1 || die "/livox/imu endpoints did not become ready"
if [ "$PROFILE" = "baseline" ]; then
    wait_for_topic /points_raw 1 0 || die "/points_raw publisher did not become ready"
else
    wait_for_topic /points_raw 1 1 || die "/points_raw endpoints did not become ready"
    wait_for_topic /livox/lidar_fused 1 1 || die "fused cloud endpoints did not become ready"
    wait_for_topic /fastlio_go2w_fusion/diagnostics 1 1 \
        || die "fusion diagnostics endpoints did not become ready"
fi
wait_for_topic /Odometry 1 1 || die "/Odometry endpoints did not become ready"
wait_for_topic /odom 1 1 || die "/odom endpoints did not become ready"
wait_for_topic /cloud_registered 1 1 || die "/cloud_registered endpoints did not become ready"
if [ "$PROFILE" != "baseline" ]; then
    ros2 param dump --output-dir "$OUTPUT_DIR" /dual_lidar_fusion \
        > "$OUTPUT_DIR/parameter-dump.log" 2>&1
    [ -s "$FUSION_PARAMETERS_SNAPSHOT" ] \
        || die "fusion parameter snapshot was not created"
    FUSION_PARAMETERS_SHA256="$(sha256sum "$FUSION_PARAMETERS_SNAPSHOT" | awk '{print $1}')"
fi

SAMPLER_CMD=(
    python3 "$SCRIPT_DIR/sample_process_metrics.py"
    --target "fastlio:$LAUNCH_PID:fastlio_mapping"
    --target "player:$PLAYER_PID:*"
    --target "recorder:$RECORDER_PID:*"
    --output "$METRICS_CSV" --summary "$METRICS_SUMMARY"
    --stop-file "$STOP_FILE" --interval 1.0
)
if [ "$PROFILE" != "baseline" ]; then
    SAMPLER_CMD+=(--target "fusion:$LAUNCH_PID:dual_lidar_fusion_node")
fi
setsid "${SAMPLER_CMD[@]}" > "$OUTPUT_DIR/metrics.log" 2>&1 &
SAMPLER_PID=$!

RESUME_TYPE="$(ros2 service type /rosbag2_player/resume 2>/dev/null)"
[ -n "$RESUME_TYPE" ] || die "could not determine rosbag resume service type"
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

PROCESSING_NODE_MISSES=0
while kill -0 "$PLAYER_PID" 2>/dev/null; do
    kill -0 "$LAUNCH_PID" 2>/dev/null \
        || die "processing launch exited during playback; see launch.log"
    kill -0 "$RECORDER_PID" 2>/dev/null \
        || die "rosbag recorder exited during playback; see recorder.log"
    kill -0 "$SAMPLER_PID" 2>/dev/null \
        || die "resource sampler exited during playback; see metrics.log"
    if required_processing_nodes_alive; then
        PROCESSING_NODE_MISSES=0
    else
        PROCESSING_NODE_MISSES=$((PROCESSING_NODE_MISSES + 1))
        [ "$PROCESSING_NODE_MISSES" -lt 3 ] \
            || die "required processing node disappeared during playback"
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
required_processing_nodes_alive || die "required processing node missing before drain"

# Give FAST-LIO and the recorder a short opportunity to drain final messages.
sleep 2
kill -0 "$LAUNCH_PID" 2>/dev/null || die "processing launch exited during drain"
kill -0 "$RECORDER_PID" 2>/dev/null || die "rosbag recorder exited during drain"
kill -0 "$SAMPLER_PID" 2>/dev/null || die "resource sampler exited during drain"
required_processing_nodes_alive || die "required processing node missing during drain"
stop_group "$RECORDER_PID" "rosbag recorder"
stop_group "$LAUNCH_PID" "processing launch"
touch "$STOP_FILE"
for _ in $(seq 1 30); do
    kill -0 "$SAMPLER_PID" 2>/dev/null || break
    sleep 0.1
done
stop_group "$SAMPLER_PID" "resource sampler"

write_manifest "completed" 0
CLEANUP_COMPLETE="true"
trap - EXIT INT TERM

echo "Experiment complete: $OUTPUT_DIR"
