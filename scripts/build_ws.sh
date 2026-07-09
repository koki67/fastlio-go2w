#!/bin/bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_ROOT="$REPO_ROOT/humble_ws"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/$ROS_DISTRO_NAME/setup.bash"
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"

if [ ! -f "$ROS_SETUP" ]; then
    echo "Error: ROS setup not found: $ROS_SETUP" >&2
    exit 1
fi

if [ ! -d "$WORKSPACE_ROOT/src" ]; then
    echo "Error: workspace source not found: $WORKSPACE_ROOT/src" >&2
    exit 1
fi

if [ -z "$PYTHON_BIN" ] || ! command -v "$PYTHON_BIN" >/dev/null 2>&1; then
    PYTHON_BIN="$(command -v python3 || true)"
    if [ -z "$PYTHON_BIN" ]; then
        echo "Error: python3 is not available. Set PYTHON_BIN explicitly." >&2
        exit 1
    fi
fi

WORK_BUILD_DIR="$WORKSPACE_ROOT/build"
WORK_INSTALL_DIR="$WORKSPACE_ROOT/install"

for required_dir in "$WORK_BUILD_DIR" "$WORK_INSTALL_DIR"; do
    mkdir -p "$required_dir"

done

check_workspace_dir_permissions() {
    local dir="$1"
    local bad_paths=()

    for entry in "$dir"/*; do
        [ -e "$entry" ] || continue

        if [ -d "$entry" ]; then
            local owner_uid
            owner_uid="$(stat -c '%u' "$entry")"
            if [ ! -w "$entry" ] || [ "$owner_uid" -eq 0 ]; then
                bad_paths+=("$entry")
            fi
        fi
    done

    if [ "${#bad_paths[@]}" -gt 0 ]; then
        echo "Error: found non-user-owned or non-writable directories under $dir." >&2
        for bad_path in "${bad_paths[@]}"; do
            echo "  - $bad_path (owner: $(stat -c '%U:%G' "$bad_path"), perms: $(stat -c '%A' "$bad_path"))" >&2
        done
        echo "Hint: clean this workspace on-device with user permissions, then rerun." >&2
        echo "Example: rm -rf $WORKSPACE_ROOT/build $WORKSPACE_ROOT/install" >&2
        echo "Then rerun setup_ws.sh and build_ws.sh." >&2
        exit 1
    fi
}

check_workspace_dir_permissions "$WORK_BUILD_DIR"
check_workspace_dir_permissions "$WORK_INSTALL_DIR"

set +u
source "$ROS_SETUP"
set -u

cd "$WORKSPACE_ROOT"

colcon build --symlink-install \
    --cmake-args \
    -DROS_EDITION=ROS2 \
    -DDISTRO_ROS="" \
    -DPython3_EXECUTABLE="$PYTHON_BIN"
