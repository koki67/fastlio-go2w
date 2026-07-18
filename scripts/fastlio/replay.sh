#!/bin/bash
# Replay a raw FAST-LIO bag and optionally visualize output.
#
# Usage:
#   bash scripts/fastlio/replay.sh BAG [--profile PROFILE] [--rviz|--no-rviz]
#       [--rate RATE] [--config YAML] [--debug-cloud]
#
# Profiles:
#   legacy          Original main-branch MID-360 replay (default)
#   baseline        Issue #7 MID-360-only offline evaluation
#   fused-high      MID-360 + XT16 high-density fusion
#   fused-matched   MID-360 + XT16 density-matched fusion

set -euo pipefail

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
    sed -n '1,14p' "$0"
    exit 0
fi

if [ "${1:-}" = "" ]; then
    echo "Error: bag directory required." >&2
    sed -n '4,13p' "$0" >&2
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BAG="${1:?}"
shift || true

RVIZ=true
RATE=1.0
CONFIG=""
PROFILE="legacy"
DEBUG_CLOUD=false

while [ "$#" -gt 0 ]; do
    case "$1" in
        --profile)
            PROFILE="${2:?Error: --profile requires a value}"
            shift 2
            ;;
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
        --debug-cloud)
            DEBUG_CLOUD=true
            shift
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

case "$PROFILE" in
    legacy|baseline|fused-high|fused-matched) ;;
    *)
        echo "Error: unknown profile: $PROFILE" >&2
        exit 1
        ;;
esac

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
        echo "Looked relative to the current directory, repository root, and bringup config directory." >&2
        exit 1
    fi
fi

if [ -z "${ROS_DISTRO:-}" ]; then
    set +u
    source /opt/ros/humble/setup.bash
    set -u
fi

case "$PROFILE" in
    legacy) DEFAULT_CONFIG_NAME="mid360_go2w.yaml" ;;
    baseline) DEFAULT_CONFIG_NAME="mid360_go2w_accuracy_dense_false.yaml" ;;
    fused-high|fused-matched)
        DEFAULT_CONFIG_NAME="mid360_xt16_fused_accuracy_dense_false.yaml"
        ;;
esac

candidates=(
    "$REPO_ROOT/.devcontainer/desktop_ws/install/setup.bash"
    "$REPO_ROOT/humble_ws/install/setup.bash"
)
WORKSPACE_SETUP=""
for candidate in "${candidates[@]}"; do
    [ -f "$candidate" ] || continue
    install_root="$(dirname "$candidate")"
    bringup_prefix="$install_root/fastlio_go2w_bringup"
    bringup_share="$bringup_prefix/share/fastlio_go2w_bringup"
    replay_launch="$bringup_share/launch/replay.launch.py"
    fastlio_executable="$install_root/fast_lio/lib/fast_lio/fastlio_mapping"
    odom_adapter_executable="$bringup_prefix/lib/fastlio_go2w_bringup/fastlio_odom_adapter"
    fusion_executable="$install_root/fastlio_go2w_fusion/lib/fastlio_go2w_fusion/dual_lidar_fusion_node"

    [ -f "$replay_launch" ] || continue
    [ -x "$fastlio_executable" ] || continue
    [ -x "$odom_adapter_executable" ] || continue
    if [ -z "$CONFIG" ] && [ ! -f "$bringup_share/config/$DEFAULT_CONFIG_NAME" ]; then
        continue
    fi
    if [ "$PROFILE" = "fused-high" ] || [ "$PROFILE" = "fused-matched" ]; then
        [ -x "$fusion_executable" ] || continue
    fi

    WORKSPACE_SETUP="$candidate"
    break
done

if [ -z "$WORKSPACE_SETUP" ]; then
    echo "Error: no usable built workspace overlay was found." >&2
    if [ -z "$CONFIG" ]; then
        echo "Required installed config: $DEFAULT_CONFIG_NAME" >&2
    fi
    echo "Rebuild the workspace overlay used in this environment and retry." >&2
    exit 1
fi

set +u
source "$WORKSPACE_SETUP"
set -u

BRINGUP_PREFIX="$(ros2 pkg prefix fastlio_go2w_bringup)" || {
    echo "Error: fastlio_go2w_bringup is not built in $WORKSPACE_SETUP" >&2
    exit 1
}
BRINGUP_SHARE="$BRINGUP_PREFIX/share/fastlio_go2w_bringup"
DEFAULT_CONFIG_PATH="$BRINGUP_SHARE/config/$DEFAULT_CONFIG_NAME"

if [ -z "$CONFIG" ] && [ ! -f "$DEFAULT_CONFIG_PATH" ]; then
    echo "Error: profile config is not installed: $DEFAULT_CONFIG_PATH" >&2
    echo "Rebuild the selected workspace overlay: $WORKSPACE_SETUP" >&2
    exit 1
fi

if [ "$PROFILE" = "fused-high" ] || [ "$PROFILE" = "fused-matched" ]; then
    ros2 pkg prefix fastlio_go2w_fusion >/dev/null 2>&1 || {
        echo "Error: fastlio_go2w_fusion is not built in $WORKSPACE_SETUP" >&2
        exit 1
    }
fi

BAG="$(realpath "$BAG")"
echo "Replaying bag: $BAG"
echo "Profile: $PROFILE"
echo "Rate: $RATE"
echo "RViz enabled: $RVIZ"
if [ -n "$CONFIG" ]; then
    echo "FAST-LIO config override: $CONFIG"
else
    echo "FAST-LIO config: $DEFAULT_CONFIG_PATH"
fi

launch_args=(
    bag:="$BAG"
    rviz:="$RVIZ"
    rate:="$RATE"
    profile:="$PROFILE"
    publish_debug_cloud:="$DEBUG_CLOUD"
)
if [ -n "$CONFIG" ]; then
    launch_args+=(config:="$CONFIG")
fi

ros2 launch fastlio_go2w_bringup replay.launch.py "${launch_args[@]}"
