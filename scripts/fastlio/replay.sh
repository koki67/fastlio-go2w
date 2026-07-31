#!/bin/bash
# Replay a raw FAST-LIO bag and optionally visualize output.
#
# Usage:
#   bash scripts/fastlio/replay.sh BAG [--rviz|--no-rviz] [--rate RATE]
#     [--config YAML] [--lidar-format auto|custom-msg|pointcloud2]

set -euo pipefail

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
    sed -n '1,16p' "$0"
    exit 0
fi
if [ "${1:-}" = "" ]; then
    echo "Error: bag directory required." >&2
    echo "Usage: $0 BAG [--no-rviz] [--rate RATE] [--config YAML] [--lidar-format FORMAT]" >&2
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BAG="${1:?}"
shift || true

RVIZ=true
RATE=1.0
CONFIG=""
LIDAR_FORMAT="auto"

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
        --lidar-format)
            LIDAR_FORMAT="${2:?Error: --lidar-format requires a value}"
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

case "$LIDAR_FORMAT" in
    auto|custom-msg|pointcloud2) ;;
    *)
        echo "Error: --lidar-format must be auto, custom-msg, or pointcloud2 (got: $LIDAR_FORMAT)" >&2
        exit 1
        ;;
esac

if [ ! -d "$BAG" ] && [ -d "$REPO_ROOT/$BAG" ]; then
    BAG="$REPO_ROOT/$BAG"
fi

if [ ! -d "$BAG" ]; then
    echo "Error: bag directory not found: $BAG" >&2
    if [[ "$BAG" == /mnt/data1/* ]]; then
        echo "Inside the devcontainer, use the mounted /mnt/go2w-experiment-recorder/bags/... path." >&2
    fi
    exit 1
fi

DETECTOR="$SCRIPT_DIR/detect_livox_bag_format.py"
if [ ! -f "$BAG/metadata.yaml" ]; then
    echo "Error: invalid bag directory (missing metadata.yaml): $BAG" >&2
    exit 1
fi
if [ ! -f "$DETECTOR" ]; then
    echo "Error: Livox bag format detector not found: $DETECTOR" >&2
    exit 1
fi
if ! DETECTED_FORMAT="$(python3 "$DETECTOR" "$BAG/metadata.yaml")"; then
    echo "Error: could not establish a supported Livox input format before replay." >&2
    exit 1
fi
if [ "$LIDAR_FORMAT" != "auto" ] && [ "$LIDAR_FORMAT" != "$DETECTED_FORMAT" ]; then
    echo "Error: --lidar-format $LIDAR_FORMAT conflicts with metadata format $DETECTED_FORMAT." >&2
    exit 1
fi
RESOLVED_LIDAR_FORMAT="$DETECTED_FORMAT"

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

if [ -f /opt/ros/humble/setup.bash ]; then
    set +u
    source /opt/ros/humble/setup.bash
    set -u
elif [ "${ROS_DISTRO:-}" != "humble" ]; then
    echo "Error: ROS 2 Humble is required. Run replay inside the project devcontainer." >&2
    echo "Use container bag paths under /mnt/go2w-experiment-recorder/bags/." >&2
    exit 1
fi

if [ "${ROS_DISTRO:-}" != "humble" ]; then
    echo "Error: sourcing the ROS environment did not select Humble (ROS_DISTRO=${ROS_DISTRO:-unset})." >&2
    exit 1
fi

if ! command -v ros2 >/dev/null 2>&1; then
    echo "Error: ros2 is unavailable after sourcing ROS 2 Humble." >&2
    exit 1
fi
REPLAY_SOURCE="$REPO_ROOT/humble_ws/src/fastlio_go2w_bringup/launch/replay.launch.py"
GRAPH_SOURCE="$REPO_ROOT/humble_ws/src/fastlio_go2w_bringup/src/fastlio_go2w_bringup/livox_replay.py"
REPLAY_SOURCE_SHA256="$(sha256sum "$REPLAY_SOURCE" | awk '{print $1}')"
GRAPH_SOURCE_SHA256="$(sha256sum "$GRAPH_SOURCE" | awk '{print $1}')"

candidate_is_usable() {
    local candidate="$1"
    local install_root bringup_prefix replay_runtime graph_runtime graph_egg_link
    local graph_python_root adapter_runtime
    [ -f "$candidate" ] || return 1
    install_root="$(dirname "$candidate")"
    bringup_prefix="$install_root/fastlio_go2w_bringup"
    replay_runtime="$bringup_prefix/share/fastlio_go2w_bringup/launch/replay.launch.py"
    graph_runtime="$bringup_prefix/lib/python3.10/site-packages/fastlio_go2w_bringup/livox_replay.py"
    if [ ! -f "$graph_runtime" ]; then
        graph_egg_link="$bringup_prefix/lib/python3.10/site-packages/fastlio-go2w-bringup.egg-link"
        [ -f "$graph_egg_link" ] || return 1
        IFS= read -r graph_python_root < "$graph_egg_link" || return 1
        graph_runtime="$graph_python_root/fastlio_go2w_bringup/livox_replay.py"
    fi
    [ -f "$replay_runtime" ] || return 1
    [ -f "$graph_runtime" ] || return 1
    [ "$(sha256sum "$replay_runtime" | awk '{print $1}')" = "$REPLAY_SOURCE_SHA256" ] \
        || return 1
    [ "$(sha256sum "$graph_runtime" | awk '{print $1}')" = "$GRAPH_SOURCE_SHA256" ] \
        || return 1
    if [ "$RESOLVED_LIDAR_FORMAT" = "pointcloud2" ]; then
        adapter_runtime="$install_root/fastlio_go2w_livox/lib/fastlio_go2w_livox/livox_pointcloud_adapter"
        [ -x "$adapter_runtime" ] || return 1
    fi
    (
        set +u
        source "$candidate"
        set -u
        [ "$(ros2 pkg prefix fastlio_go2w_bringup 2>/dev/null)" = "$bringup_prefix" ] \
            || exit 1
        if [ "$RESOLVED_LIDAR_FORMAT" = "pointcloud2" ]; then
            [ "$(ros2 pkg prefix fastlio_go2w_livox 2>/dev/null)" = \
              "$install_root/fastlio_go2w_livox" ] || exit 1
        fi
    )
}

WORKSPACE_SETUP=""
for candidate in \
    "$REPO_ROOT/.devcontainer/desktop_ws/install/setup.bash" \
    "$REPO_ROOT/humble_ws/install/setup.bash"; do
    if candidate_is_usable "$candidate"; then
        WORKSPACE_SETUP="$candidate"
        break
    fi
done
if [ -z "$WORKSPACE_SETUP" ]; then
    echo "Error: no current Humble overlay matches this checkout and Livox format." >&2
    echo "Rebuild the workspace inside the project devcontainer, then retry." >&2
    exit 1
fi
set +u
source "$WORKSPACE_SETUP"
set -u

echo "Replaying bag: $BAG"
echo "Rate: $RATE"
echo "RViz enabled: $RVIZ"
echo "Livox metadata format: $DETECTED_FORMAT"
echo "Resolved Livox format: $RESOLVED_LIDAR_FORMAT"
if [ -n "$CONFIG" ]; then
    echo "FAST-LIO config: $CONFIG"
fi

launch_args=(
    bag:="$BAG"
    rviz:="$RVIZ"
    rate:="$RATE"
    lidar_format:="$RESOLVED_LIDAR_FORMAT"
)
if [ -n "$CONFIG" ]; then
    launch_args+=(config:="$CONFIG")
fi

ros2 launch fastlio_go2w_bringup replay.launch.py "${launch_args[@]}"
