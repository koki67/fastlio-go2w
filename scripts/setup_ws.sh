#!/bin/bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ ! -d "$REPO_ROOT/.git" ]; then
    echo "Error: this is not a git repository: $REPO_ROOT" >&2
    exit 1
fi

echo "Syncing and updating git submodules..."
git -C "$REPO_ROOT" submodule sync --recursive
git -C "$REPO_ROOT" submodule update --init --recursive

DRIVER_PKG_DIR="$REPO_ROOT/humble_ws/src/livox_ros_driver2"
DRIVER_MANIFEST_ROS1="$DRIVER_PKG_DIR/package_ROS1.xml"
DRIVER_MANIFEST_ROS2="$DRIVER_PKG_DIR/package_ROS2.xml"
DRIVER_MANIFEST="$DRIVER_PKG_DIR/package.xml"

if [ -f "$DRIVER_MANIFEST_ROS2" ]; then
    cp -f "$DRIVER_MANIFEST_ROS2" "$DRIVER_MANIFEST"
elif [ -f "$DRIVER_MANIFEST_ROS1" ]; then
    cp -f "$DRIVER_MANIFEST_ROS1" "$DRIVER_MANIFEST"
else
    echo "Warning: no Livox ROS manifest template found at:" >&2
    echo "  - $DRIVER_MANIFEST_ROS2" >&2
    echo "  - $DRIVER_MANIFEST_ROS1" >&2
    echo "  continue with existing package.xml if present." >&2
fi

if [ -f "$DRIVER_MANIFEST" ]; then
    echo "Prepared livox_ros_driver2/package.xml"
else
    echo "Warning: livox_ros_driver2 package.xml is still missing." >&2
fi

