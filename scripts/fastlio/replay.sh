#!/bin/bash
# Replay a raw FAST-LIO bag and optionally visualize output.
#
# Usage:
#   bash scripts/fastlio/replay.sh <bag_dir> [--rviz] [--rate 2.0]

set -euo pipefail

if [ "${1:-}" = "" ]; then
    echo "Error: bag directory required." >&2
    echo "Usage: $0 <bag_directory> [--rviz] [--rate <rate>]" >&2
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BAG="${1:?}"
shift || true

RVIZ=false
RATE=1.0

while [ "$#" -gt 0 ]; do
    case "$1" in
        --rviz)
            RVIZ=true
            shift
            ;;
        --rate)
            RATE="${2:?Error: --rate requires a value}" 
            shift 2
            ;;
        -h|--help)
            sed -n '2,14p' "$0"
            exit 0
            ;;
        *)
            echo "Error: unknown argument: $1" >&2
            exit 1
            ;;
    esac
done

if [ ! -d "$BAG" ] && [ -d "$REPO_ROOT/$BAG" ]; then
    BAG="$REPO_ROOT/$BAG"
fi

if [ ! -d "$BAG" ]; then
    echo "Error: bag directory not found: $BAG" >&2
    exit 1
fi

if [ ! -f "$BAG/metadata.yaml" ]; then
    echo "Error: invalid bag directory (missing metadata.yaml): $BAG" >&2
    exit 1
fi

if [ -z "${ROS_DISTRO:-}" ]; then
    source /opt/ros/humble/setup.bash
fi

DESKTOP_SETUP="$REPO_ROOT/.devcontainer/desktop_ws/install/setup.bash"
if [ -f "$DESKTOP_SETUP" ]; then
    source "$DESKTOP_SETUP"
fi

PLAY_PID=""
cleanup() {
    if [ -n "$PLAY_PID" ]; then
        kill "$PLAY_PID" 2>/dev/null || true
        wait "$PLAY_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT

echo "Replaying bag: $BAG"
echo "Rate: $RATE"

ros2 bag play "$BAG" --clock --rate "$RATE" &
PLAY_PID=$!

sleep 2

if [ "$RVIZ" = true ]; then
    bash "$REPO_ROOT/scripts/fastlio/live_rviz.sh"
else
    bash "$REPO_ROOT/scripts/fastlio/check_tf.sh"
fi
