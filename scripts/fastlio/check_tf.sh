#!/bin/bash
# Launch check_tf visualization/check workflow.
#
# Usage:
#   bash scripts/fastlio/check_tf.sh [--rviz false]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
RVIZ=true

while [ "$#" -gt 0 ]; do
    case "$1" in
        --rviz)
            RVIZ="$2"
            shift 2
            ;;
        -h|--help)
            sed -n '1,12p' "$0"
            exit 0
            ;;
        *)
            echo "Error: unknown argument: $1" >&2
            echo "Usage: bash scripts/fastlio/check_tf.sh [--rviz false]" >&2
            exit 1
            ;;
    esac
done

case "$RVIZ" in
    true|false)
        ;;
    *)
        echo "Error: --rviz must be true or false." >&2
        exit 1
        ;;
esac

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

if [ "$RVIZ" = true ]; then
    ros2 launch fastlio_go2w_bringup check_tf.launch.py use_rviz:=true
else
    ros2 launch fastlio_go2w_bringup check_tf.launch.py use_rviz:=false
fi
