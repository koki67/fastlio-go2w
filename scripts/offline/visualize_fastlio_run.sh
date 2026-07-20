#!/usr/bin/env bash
# Display an analyzed offline FAST-LIO run without rerunning FAST-LIO.

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
PUBLISHER="$SCRIPT_DIR/publish_fastlio_artifacts.py"
RVIZ_CONFIG="$SCRIPT_DIR/rviz/offline_result.rviz"

usage() {
    cat <<'EOF'
Usage:
  scripts/offline/visualize_fastlio_run.sh RUN_DIR [options]

Options:
  --dynamic     Replay RUN_DIR/rosbag outputs (/cloud_registered, /Odometry)
  --rate RATE   Dynamic result replay rate (default: 1.0)
  --no-rviz     Keep publishers running without RViz; this is not a one-shot check
  -h, --help    Show this help

Static mode publishes the frozen preview map and matching-frame trajectory only.
Dynamic mode additionally replays the already-computed result bag; neither mode
runs FAST-LIO again.
EOF
}

die() {
    echo "Error: $*" >&2
    exit 2
}

RUN_DIR=""
DYNAMIC=false
RATE="1.0"
USE_RVIZ=true

while [ "$#" -gt 0 ]; do
    case "$1" in
        --dynamic)
            DYNAMIC=true
            shift
            ;;
        --rate)
            RATE="${2:?Error: --rate requires a value}"
            shift 2
            ;;
        --no-rviz)
            USE_RVIZ=false
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
            [ -z "$RUN_DIR" ] || die "only one run directory may be supplied"
            RUN_DIR="$1"
            shift
            ;;
    esac
done

[ -n "$RUN_DIR" ] || die "an analyzed run directory is required"
python3 -c 'import math,sys; value=float(sys.argv[1]); sys.exit(not (math.isfinite(value) and value > 0.0))' "$RATE" \
    || die "--rate must be a finite number greater than zero"

RUN_DIR="$(realpath -m "$RUN_DIR")"
[ -d "$RUN_DIR" ] || die "run directory not found: $RUN_DIR"
[ -f "$PUBLISHER" ] || die "artifact publisher not found: $PUBLISHER"
[ -f "$RVIZ_CONFIG" ] || die "RViz config not found: $RVIZ_CONFIG"

if [ -f /opt/ros/humble/setup.bash ]; then
    set +u
    source /opt/ros/humble/setup.bash
    set -u
elif [ "${ROS_DISTRO:-}" != "humble" ]; then
    die "ROS 2 Humble is required (run inside the project container)"
fi

overlay_usable() (
    candidate="$1"
    install_root="$(realpath -m "$(dirname "$candidate")")"
    set +u
    source "$candidate" >/dev/null 2>&1 || exit 1
    set -u
    prefix="$(ros2 pkg prefix fastlio_go2w_bringup 2>/dev/null)" || exit 1
    prefix="$(realpath -m "$prefix")"
    case "$prefix" in
        "$install_root"/*) exit 0 ;;
        *) exit 1 ;;
    esac
)

WORKSPACE_SETUP=""
for candidate in \
    "$REPO_ROOT/.devcontainer/desktop_ws/install/setup.bash" \
    "$REPO_ROOT/humble_ws/install/setup.bash"; do
    if [ -f "$candidate" ] && overlay_usable "$candidate"; then
        WORKSPACE_SETUP="$candidate"
        break
    fi
done
[ -n "$WORKSPACE_SETUP" ] \
    || die "no usable fastlio_go2w_bringup workspace overlay was found; build the workspace"
set +u
source "$WORKSPACE_SETUP"
set -u

for command in python3 ros2 setsid; do
    command -v "$command" >/dev/null || die "required command not found: $command"
done
if [ "$USE_RVIZ" = true ]; then
    command -v rviz2 >/dev/null || die "rviz2 is not installed"
    if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
        die "no graphical display; --no-rviz keeps publishers running headlessly"
    fi
fi
if [ "$DYNAMIC" = true ]; then
    [ -f "$RUN_DIR/rosbag/metadata.yaml" ] \
        || die "dynamic result bag not found: $RUN_DIR/rosbag"
fi

MAP_FRAME_ID="$(python3 "$PUBLISHER" "$RUN_DIR" --print-frame-id)"
[ -n "$MAP_FRAME_ID" ] || die "validated artifact map frame is empty"
echo "Validated artifact frame: $MAP_FRAME_ID"

PUBLISHER_PID=""
PLAYER_PID=""
CLEANED_UP=false

stop_group() {
    local pid="${1:-}"
    [ -n "$pid" ] || return 0
    if ! kill -0 "$pid" 2>/dev/null; then
        wait "$pid" 2>/dev/null || true
        return 0
    fi
    kill -INT -- "-$pid" 2>/dev/null || true
    for _ in $(seq 1 25); do
        kill -0 "$pid" 2>/dev/null || break
        sleep 0.1
    done
    if kill -0 "$pid" 2>/dev/null; then
        kill -TERM -- "-$pid" 2>/dev/null || true
    fi
    wait "$pid" 2>/dev/null || true
}

cleanup() {
    local status=$?
    trap - EXIT INT TERM
    if [ "$CLEANED_UP" != true ]; then
        stop_group "$PLAYER_PID"
        stop_group "$PUBLISHER_PID"
        CLEANED_UP=true
    fi
    exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

PUBLISHER_CMD=(python3 "$PUBLISHER" "$RUN_DIR")
echo "Run: $RUN_DIR"
echo "Workspace: $WORKSPACE_SETUP"
echo "Starting frozen map/path publisher..."
setsid "${PUBLISHER_CMD[@]}" &
PUBLISHER_PID=$!
sleep 0.5
kill -0 "$PUBLISHER_PID" 2>/dev/null \
    || die "artifact publisher exited during startup"

if [ "$DYNAMIC" = true ]; then
    PLAYER_CMD=(
        ros2 bag play "$RUN_DIR/rosbag"
        --clock --rate "$RATE" --delay 2.0 --disable-keyboard-controls
        --topics /cloud_registered /Odometry
    )
    echo "Starting frozen result replay at ${RATE}x..."
    setsid "${PLAYER_CMD[@]}" &
    PLAYER_PID=$!
fi

if [ "$USE_RVIZ" = true ]; then
    RVIZ_CMD=(rviz2 -d "$RVIZ_CONFIG" -f "$MAP_FRAME_ID")
    if [ "$DYNAMIC" = true ]; then
        RVIZ_CMD+=(--ros-args -p use_sim_time:=true)
    fi
    echo "Starting RViz..."
    "${RVIZ_CMD[@]}"
elif [ "$DYNAMIC" = true ]; then
    wait "$PLAYER_PID"
else
    echo "Headless static publisher is running; press Ctrl-C to stop."
    wait "$PUBLISHER_PID"
fi
