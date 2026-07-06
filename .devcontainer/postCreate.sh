#!/bin/bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_ROOT="$REPO_ROOT/humble_ws"
DESKTOP_WS_ROOT="$REPO_ROOT/.devcontainer/desktop_ws"
DESKTOP_INSTALL="$DESKTOP_WS_ROOT/install"

if [ ! -f "$ROS_SETUP" ]; then
    echo "Error: ROS 2 setup not found: $ROS_SETUP" >&2
    exit 1
fi

if [ ! -d "$WORKSPACE_ROOT/src" ]; then
    echo "Error: Workspace source not found: $WORKSPACE_ROOT/src" >&2
    exit 1
fi

bash "$REPO_ROOT/.devcontainer/configure_git_safe_directory.sh"

source "$ROS_SETUP"

mkdir -p "$DESKTOP_WS_ROOT"
rm -rf "$DESKTOP_WS_ROOT/build" "$DESKTOP_WS_ROOT/install" "$DESKTOP_WS_ROOT/log"

cd "$WORKSPACE_ROOT"

colcon build --symlink-install \
    --build-base "$DESKTOP_WS_ROOT/build" \
    --install-base "$DESKTOP_INSTALL" \
    --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble

if ! grep -Fxq "source $DESKTOP_INSTALL/setup.bash" ~/.bashrc; then
    echo "source $DESKTOP_INSTALL/setup.bash" >> ~/.bashrc
fi

if ! grep -Fxq "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

echo "FAST-LIO desktop environment is ready."
echo "Source: $DESKTOP_INSTALL/setup.bash"
