#!/bin/bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_ROOT="$REPO_ROOT/humble_ws"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/$ROS_DISTRO_NAME/setup.bash"

if [ ! -f "$ROS_SETUP" ]; then
    echo "Error: ROS setup not found: $ROS_SETUP" >&2
    exit 1
fi

if [ ! -d "$WORKSPACE_ROOT/src" ]; then
    echo "Error: workspace source not found: $WORKSPACE_ROOT/src" >&2
    exit 1
fi

set +u
source "$ROS_SETUP"
set -u

cd "$WORKSPACE_ROOT"

colcon build --symlink-install \
    "" \
    --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=""
