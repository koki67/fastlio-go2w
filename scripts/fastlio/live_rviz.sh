#!/bin/bash
# Launch RViz2 for FAST-LIO on desktop.
#
# Usage:
#   bash scripts/fastlio/live_rviz.sh [--iface <desktop_iface>] [--config <rviz_config>]
#
# Example:
#   bash scripts/fastlio/live_rviz.sh
#   bash scripts/fastlio/live_rviz.sh --iface enp97s0

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
RVIZ_CFG_DEFAULT="$REPO_ROOT/humble_ws/src/fastlio_go2w_bringup/rviz/fastlio.rviz"
IFACE_DEFAULT="${CYCLONEDDS_IFACE:-enp97s0}"

source_setup_safely() {
    local setup_script="$1"
    local rc=0

    set +u
    # shellcheck disable=SC1090
    source "$setup_script" || rc=$?
    set -u
    return "$rc"
}

iface="$IFACE_DEFAULT"
rviz_cfg="$RVIZ_CFG_DEFAULT"

while [ "$#" -gt 0 ]; do
    case "$1" in
        --iface)
            iface="${2:?Error: --iface requires a value}"
            shift 2
            ;;
        --config)
            rviz_cfg="${2:?Error: --config requires a value}"
            shift 2
            ;;
        -h|--help)
            sed -n '2,12p' "$0"
            exit 0
            ;;
        *)
            echo "Error: unknown argument: $1" >&2
            exit 1
            ;;
    esac
done

if ! command -v python3 >/dev/null 2>&1; then
    echo "Error: python3 not found." >&2
    exit 1
fi

if [ -z "${ROS_DISTRO:-}" ]; then
    source_setup_safely /opt/ros/humble/setup.bash
fi

DESKTOP_SETUP="$REPO_ROOT/.devcontainer/desktop_ws/install/setup.bash"
if [ -f "$DESKTOP_SETUP" ]; then
    source_setup_safely "$DESKTOP_SETUP"
fi

if ! command -v rviz2 >/dev/null 2>&1; then
    echo "Error: rviz2 not found in PATH. Use .devcontainer or install ROS 2 Humble Desktop." >&2
    exit 1
fi

if [ -z "${DISPLAY:-}" ]; then
    echo "Error: DISPLAY is not set." >&2
    exit 1
fi

if [ ! -f "$rviz_cfg" ]; then
    echo "Error: RViz config not found: $rviz_cfg" >&2
    exit 1
fi

if [ ! -d "/sys/class/net/$iface" ]; then
    echo "Error: interface not found: $iface" >&2
    exit 1
fi

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$iface\" priority=\"2\" multicast=\"true\" /></Interfaces></General></Domain></CycloneDDS>"

echo "Launching RViz with:"
echo "  rviz_cfg = $rviz_cfg"
echo "  iface   = $iface"
echo "  RMW     = $RMW_IMPLEMENTATION"
echo "  domain  = $ROS_DOMAIN_ID"
rviz2 -d "$rviz_cfg"
