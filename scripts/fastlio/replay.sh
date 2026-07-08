#!/bin/bash
# Replay a raw FAST-LIO bag and optionally visualize output.
#
# Usage:
#   bash scripts/fastlio/replay.sh <bag_directory> [--rviz] [--no-rviz] [--rate <rate>] [--config <yaml>]

set -euo pipefail

if [ "${1:-}" = "" ]; then
    echo "Error: bag directory required." >&2
    echo "Usage: $0 <bag_directory> [--no-rviz] [--rate <rate>] [--config <yaml>]" >&2
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BAG="${1:?}"
shift || true

RVIZ=true
RATE=1.0
CONFIG=""

while [ "$#" -gt 0 ]; do
    case "$1" in
        --rviz)
            RVIZ=true
            shift
            ;;
        --no-rviz)
            RVIZ=false
            shift
            ;;
        --rate)
            RATE="${2:?Error: --rate requires a value}"
            shift 2
            ;;
        --config)
            CONFIG="${2:?Error: --config requires a value}"
            shift 2
            ;;
        -h|--help)
            sed -n '1,14p' "$0"
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

if [ -n "$CONFIG" ]; then
    if [ -f "$CONFIG" ]; then
        CONFIG="$(cd "$(dirname "$CONFIG")" && pwd)/$(basename "$CONFIG")"
    elif [ -f "$REPO_ROOT/$CONFIG" ]; then
        CONFIG="$(cd "$(dirname "$REPO_ROOT/$CONFIG")" && pwd)/$(basename "$CONFIG")"
    elif [ -f "$REPO_ROOT/humble_ws/src/fastlio_go2w_bringup/config/$CONFIG" ]; then
        CONFIG="$REPO_ROOT/humble_ws/src/fastlio_go2w_bringup/config/$CONFIG"
    else
        echo "Error: config file not found: $CONFIG" >&2
        echo "Looked relative to current directory, repository root, and fastlio_go2w_bringup/config." >&2
        exit 1
    fi
fi

if [ -z "${ROS_DISTRO:-}" ]; then
    set +u
    source /opt/ros/humble/setup.bash
    set -u
fi

DESKTOP_SETUP="$REPO_ROOT/.devcontainer/desktop_ws/install/setup.bash"
if [ -f "$DESKTOP_SETUP" ]; then
    set +u
    source "$DESKTOP_SETUP"
    set -u
fi

echo "Replaying bag: $BAG"
echo "Rate: $RATE"
echo "RViz enabled: $RVIZ"
if [ -n "$CONFIG" ]; then
    echo "FAST-LIO config: $CONFIG"
fi

launch_args=(bag:="$BAG" rviz:="$RVIZ" rate:="$RATE")
if [ -n "$CONFIG" ]; then
    launch_args+=(config:="$CONFIG")
fi

ros2 launch fastlio_go2w_bringup replay.launch.py "${launch_args[@]}"
