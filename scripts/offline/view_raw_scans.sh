#!/usr/bin/env bash
# Replay only the latest raw MID-360 and XT16 scans in RViz.

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RVIZ_CONFIG="$SCRIPT_DIR/rviz/raw_scans.rviz"

usage() {
    cat <<'EOF'
Usage:
  scripts/offline/view_raw_scans.sh BAG [options]

Options:
  --rate RATE          Rosbag playback multiplier (default: 1.0)
  --start-offset SEC   Start this many seconds into the bag (default: 0)
  --loop               Restart playback when the bag reaches the end
  --domain-id ID       Isolated ROS domain ID (default: 78)
  --no-rviz            Replay and publish the display TF without launching RViz
  --dry-run            Validate the bag and print commands without launching ROS
  -h, --help           Show this help

The bag must contain these PointCloud2 topics:
  /livox/lidar  (MID-360, frame livox_frame)
  /points_raw   (Pandar XT16, frame hesai_lidar)

The viewer does not run FAST-LIO, register points, or build a map. Each RViz
PointCloud2 display has Decay Time 0, so it retains only the current message.
Use the two checkboxes in RViz to show MID-360 only, XT16 only, or both.
The RViz fixed frame is base_link so the grid is parallel to the robot XY plane
instead of inheriting the MID-360's approximately 18.3 degree mounting pitch.
EOF
}

die() {
    echo "Error: $*" >&2
    exit 2
}

source_ros() {
    if command -v ros2 >/dev/null 2>&1 \
        && { [ "$USE_RVIZ" != "true" ] || command -v rviz2 >/dev/null 2>&1; }; then
        return
    fi

    local setup
    for setup in /opt/ros/humble/setup.bash /opt/ros/jazzy/setup.bash; do
        if [ -f "$setup" ]; then
            set +u
            # shellcheck disable=SC1090
            source "$setup"
            set -u
            break
        fi
    done

    command -v ros2 >/dev/null 2>&1 \
        || die "ros2 was not found; run this in the Humble devcontainer or source ROS 2"
    if [ "$USE_RVIZ" = "true" ]; then
        command -v rviz2 >/dev/null 2>&1 \
            || die "rviz2 was not found; install/source a ROS 2 desktop environment"
    fi
}

print_command() {
    printf '  '
    printf '%q ' "$@"
    printf '\n'
}

BAG=""
RATE="1.0"
START_OFFSET="0"
DOMAIN_ID="78"
LOOP="false"
USE_RVIZ="true"
DRY_RUN="false"

while [ "$#" -gt 0 ]; do
    case "$1" in
        --rate)
            [ "$#" -ge 2 ] || die "--rate requires a value"
            RATE="$2"
            shift 2
            ;;
        --start-offset)
            [ "$#" -ge 2 ] || die "--start-offset requires a value"
            START_OFFSET="$2"
            shift 2
            ;;
        --loop)
            LOOP="true"
            shift
            ;;
        --domain-id)
            [ "$#" -ge 2 ] || die "--domain-id requires a value"
            DOMAIN_ID="$2"
            shift 2
            ;;
        --no-rviz)
            USE_RVIZ="false"
            shift
            ;;
        --dry-run)
            DRY_RUN="true"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        --*)
            die "unknown option: $1"
            ;;
        *)
            [ -z "$BAG" ] || die "only one bag path may be supplied"
            BAG="$1"
            shift
            ;;
    esac
done

[ -n "$BAG" ] || die "a bag directory is required"
[ -d "$BAG" ] || {
    if [[ "$BAG" == /mnt/data1/experimental_data/go2w-experiment-recorder/bags/* ]]; then
        mapped="/mnt/go2w-experiment-recorder/bags/${BAG##*/}"
        die "bag not found: $BAG (inside the devcontainer, try $mapped)"
    fi
    die "bag directory not found: $BAG"
}

BAG="$(realpath "$BAG")"
METADATA="$BAG/metadata.yaml"
[ -r "$METADATA" ] || die "bag metadata is missing or unreadable: $METADATA"
[ -r "$RVIZ_CONFIG" ] || die "RViz configuration is missing: $RVIZ_CONFIG"

command -v python3 >/dev/null 2>&1 || die "python3 is required for metadata validation"

python3 - "$RATE" "$START_OFFSET" "$DOMAIN_ID" <<'PY' \
    || die "invalid numeric playback option"
import math
import sys

try:
    rate = float(sys.argv[1])
    start = float(sys.argv[2])
    domain = int(sys.argv[3])
except ValueError:
    raise SystemExit(1)

if not math.isfinite(rate) or rate <= 0.0:
    raise SystemExit(1)
if not math.isfinite(start) or start < 0.0:
    raise SystemExit(1)
if not 0 <= domain <= 232:
    raise SystemExit(1)
PY

METADATA_SUMMARY="$(python3 - "$METADATA" <<'PY'
import sys

try:
    import yaml
except ImportError as error:
    print(f"PyYAML is required to inspect rosbag metadata: {error}", file=sys.stderr)
    raise SystemExit(2)

path = sys.argv[1]
try:
    with open(path, encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    info = document["rosbag2_bagfile_information"]
    entries = info["topics_with_message_count"]
except (OSError, KeyError, TypeError, yaml.YAMLError) as error:
    print(f"could not read rosbag metadata {path}: {error}", file=sys.stderr)
    raise SystemExit(2)

topics = {}
for entry in entries:
    metadata = entry.get("topic_metadata", {})
    name = metadata.get("name")
    if name:
        topics[name] = (metadata.get("type"), int(entry.get("message_count", 0)))

expected = {
    "/livox/lidar": "sensor_msgs/msg/PointCloud2",
    "/points_raw": "sensor_msgs/msg/PointCloud2",
}
labels = {
    "/livox/lidar": "MID-360",
    "/points_raw": "XT16",
}

for name, expected_type in expected.items():
    if name not in topics:
        print(f"required raw-scan topic is missing: {name}", file=sys.stderr)
        raise SystemExit(2)
    actual_type, count = topics[name]
    if actual_type != expected_type:
        print(
            f"{name} has type {actual_type!r}, expected {expected_type}; "
            "this direct RViz viewer supports the current PointCloud2 recorder contract",
            file=sys.stderr,
        )
        raise SystemExit(2)
    if count <= 0:
        print(f"required raw-scan topic has no messages: {name}", file=sys.stderr)
        raise SystemExit(2)
    print(f"{labels[name]} {name}: {count} PointCloud2 messages")
PY
)" || die "bag is not compatible with the raw-scan viewer"

export ROS_DOMAIN_ID="$DOMAIN_ID"

BASE_TO_LIVOX_TF_COMMAND=(
    ros2 run tf2_ros static_transform_publisher
    --x 0.211 --y 0.0 --z 0.2008
    --qx 0.0 --qy 0.15883 --qz 0.0 --qw 0.987306
    --frame-id base_link --child-frame-id livox_frame
)
LIVOX_TO_HESAI_TF_COMMAND=(
    ros2 run tf2_ros static_transform_publisher
    --x -0.018602675 --y 0.0 --z -0.095450199
    --qx -0.112310121 --qy -0.112310121
    --qz 0.698130673 --qw 0.698130673
    --frame-id livox_frame --child-frame-id hesai_lidar
)
RVIZ_COMMAND=(rviz2 -d "$RVIZ_CONFIG")
PLAY_COMMAND=(
    ros2 bag play "$BAG"
    --topics /livox/lidar /points_raw
    --rate "$RATE"
    --start-offset "$START_OFFSET"
    --delay 2
)
if [ "$LOOP" = "true" ]; then
    PLAY_COMMAND+=(--loop)
fi

echo "$METADATA_SUMMARY"
echo "Fixed frame: base_link (level display grid)"
echo "Display transform: base_link -> livox_frame (MID-360 mounting calibration)"
echo "Display transform: livox_frame -> hesai_lidar (initial composed estimate)"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"

if [ "$DRY_RUN" = "true" ]; then
    echo "Commands:"
    print_command "${BASE_TO_LIVOX_TF_COMMAND[@]}"
    print_command "${LIVOX_TO_HESAI_TF_COMMAND[@]}"
    if [ "$USE_RVIZ" = "true" ]; then
        print_command "${RVIZ_COMMAND[@]}"
    fi
    print_command "${PLAY_COMMAND[@]}"
    exit 0
fi

source_ros

BASE_LOG_DIR="${ROS_LOG_DIR:-/tmp/fastlio-go2w-raw-scan-${UID:-user}}"
mkdir -p "$BASE_LOG_DIR"
RUN_LOG_DIR="$(mktemp -d "$BASE_LOG_DIR/viewer.XXXXXX")"
export ROS_LOG_DIR="$RUN_LOG_DIR"

CHILD_PIDS=()
cleanup() {
    local status=$?
    trap - EXIT INT TERM
    local pid
    for pid in "${CHILD_PIDS[@]}"; do
        kill -INT "$pid" 2>/dev/null || true
    done
    for pid in "${CHILD_PIDS[@]}"; do
        wait "$pid" 2>/dev/null || true
    done
    exit "$status"
}
trap cleanup EXIT INT TERM

"${BASE_TO_LIVOX_TF_COMMAND[@]}" >"$RUN_LOG_DIR/base_to_livox_transform.log" 2>&1 &
BASE_TO_LIVOX_TF_PID=$!
CHILD_PIDS+=("$BASE_TO_LIVOX_TF_PID")
"${LIVOX_TO_HESAI_TF_COMMAND[@]}" >"$RUN_LOG_DIR/livox_to_hesai_transform.log" 2>&1 &
LIVOX_TO_HESAI_TF_PID=$!
CHILD_PIDS+=("$LIVOX_TO_HESAI_TF_PID")
sleep 0.5
if ! kill -0 "$BASE_TO_LIVOX_TF_PID" 2>/dev/null; then
    wait "$BASE_TO_LIVOX_TF_PID" || true
    die "base-to-Livox TF publisher exited; see $RUN_LOG_DIR/base_to_livox_transform.log"
fi
if ! kill -0 "$LIVOX_TO_HESAI_TF_PID" 2>/dev/null; then
    wait "$LIVOX_TO_HESAI_TF_PID" || true
    die "Livox-to-Hesai TF publisher exited; see $RUN_LOG_DIR/livox_to_hesai_transform.log"
fi

if [ "$USE_RVIZ" = "true" ]; then
    "${RVIZ_COMMAND[@]}" >"$RUN_LOG_DIR/rviz.log" 2>&1 &
    RVIZ_PID=$!
    CHILD_PIDS+=("$RVIZ_PID")
    sleep 1
    if ! kill -0 "$RVIZ_PID" 2>/dev/null; then
        wait "$RVIZ_PID" || true
        die "RViz exited during startup; see $RUN_LOG_DIR/rviz.log"
    fi
fi

echo "Logs: $RUN_LOG_DIR"
echo "Replaying only /livox/lidar and /points_raw; press Ctrl-C to stop."
"${PLAY_COMMAND[@]}"
