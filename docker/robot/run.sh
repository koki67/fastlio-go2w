#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

IMAGE="${FASTLIO_GO2W_IMAGE:-fastlio-go2w:latest}"
ROBOT_IFACE="${FASTLIO_GO2W_ROBOT_IFACE:-eth0}"
REMOTE_VIZ_IFACE="${FASTLIO_GO2W_REMOTE_IFACE:-wlan0}"
REMOTE_VIZ="${FASTLIO_GO2W_REMOTE_VIZ:-false}"
HOST_USER="${FASTLIO_GO2W_HOST_USER:-false}"
ROBOT_IFACE_OVERRIDDEN=false
if [ -n "${FASTLIO_GO2W_ROBOT_IFACE:-}" ]; then
    ROBOT_IFACE_OVERRIDDEN=true
fi

usage() {
    cat <<'EOF_USAGE'
Usage:
  bash docker/robot/run.sh [options] [command...]

Options:
  --remote-viz                 Also bind CycloneDDS to the Wi-Fi interface so
                               a desktop RViz session can inspect the ROS graph.
  --remote-viz-iface IFACE     Wi-Fi/interface name used with --remote-viz
                               (default: wlan0 or FASTLIO_GO2W_REMOTE_IFACE).
  --robot-iface IFACE          Robot/internal DDS interface (default: eth0 or
                               FASTLIO_GO2W_ROBOT_IFACE).
  --host-user                  Run the container command as the invoking host
                               UID/GID. Use this for workspace builds so
                               build/install artifacts stay user-owned.
  -h, --help                   Show this help.

Environment:
  FASTLIO_GO2W_IMAGE           Docker image name, default fastlio-go2w:latest.
  FASTLIO_GO2W_REMOTE_VIZ      Set true to enable remote visualization DDS.
  FASTLIO_GO2W_REMOTE_IFACE    Remote visualization interface, default wlan0.
  FASTLIO_GO2W_ROBOT_IFACE     Robot/internal DDS interface, default eth0.
  FASTLIO_GO2W_HOST_USER       Set true to enable --host-user behavior.
  ROS_DOMAIN_ID                Forwarded into the container, default 0.
  CYCLONEDDS_URI               If set, used verbatim and generated DDS profiles
                               are skipped.
EOF_USAGE
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --remote-viz)
            REMOTE_VIZ=true
            shift
            ;;
        --remote-viz-iface)
            if [ "$#" -lt 2 ]; then
                echo "--remote-viz-iface requires an interface name." >&2
                exit 2
            fi
            REMOTE_VIZ_IFACE="$2"
            shift 2
            ;;
        --robot-iface)
            if [ "$#" -lt 2 ]; then
                echo "--robot-iface requires an interface name." >&2
                exit 2
            fi
            ROBOT_IFACE="$2"
            ROBOT_IFACE_OVERRIDDEN=true
            shift 2
            ;;
        --host-user)
            HOST_USER=true
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        --)
            shift
            break
            ;;
        *)
            break
            ;;
    esac
done

build_cyclonedds_uri() {
    local interfaces
    interfaces="<NetworkInterface name=\"${ROBOT_IFACE}\" priority=\"1\" multicast=\"true\" />"

    if [ "$REMOTE_VIZ" = "true" ]; then
        if [ -d "/sys/class/net/${REMOTE_VIZ_IFACE}" ]; then
            interfaces="${interfaces}<NetworkInterface name=\"${REMOTE_VIZ_IFACE}\" priority=\"2\" multicast=\"true\" />"
            echo "Remote visualization DDS enabled on ${REMOTE_VIZ_IFACE}." >&2
        else
            echo "Remote visualization requested, but ${REMOTE_VIZ_IFACE} was not found; using ${ROBOT_IFACE} only." >&2
        fi
    fi

    printf '<CycloneDDS><Domain><General><Interfaces>%s</Interfaces></General></Domain></CycloneDDS>' "$interfaces"
}

should_generate_cyclonedds_uri() {
    [ "$REMOTE_VIZ" = "true" ] || [ "$ROBOT_IFACE_OVERRIDDEN" = "true" ]
}

if [ -n "${CYCLONEDDS_URI:-}" ]; then
    DDS_URI="$CYCLONEDDS_URI"
elif should_generate_cyclonedds_uri; then
    DDS_URI="$(build_cyclonedds_uri)"
else
    DDS_URI="file:///etc/cyclonedds.xml"
fi

# Allow local X11 connections for Docker GUI, if a display is available.
xhost +local:docker 2>/dev/null || true

XAUTH=/tmp/.docker.xauth
if [ ! -f "$XAUTH" ]; then
    touch "$XAUTH"
    if command -v xauth >/dev/null 2>&1 && [ -n "${DISPLAY:-}" ]; then
        xauth_list=$(xauth nlist "$DISPLAY" 2>/dev/null | sed -e 's/^..../ffff/' || true)
        if [ -n "$xauth_list" ]; then
            echo "$xauth_list" | xauth -f "$XAUTH" nmerge - 2>/dev/null || true
        fi
    fi
    chmod a+r "$XAUTH"
fi

if [ "$#" -gt 0 ]; then
    CMD=("$@")
else
    CMD=(bash)
fi

CONTAINER_BOOTSTRAP='source /opt/ros/humble/setup.bash; if [ -f /external/humble_ws/install/setup.bash ]; then source /external/humble_ws/install/setup.bash; fi; exec "$@"'

USER_ARGS=()
if [ "$HOST_USER" = "true" ]; then
    USER_ARGS=(--user "$(id -u):$(id -g)" --env="HOME=/tmp")
fi

docker run -it --rm \
  --privileged \
  --runtime=nvidia \
  --net=host \
  --env="DISPLAY=${DISPLAY:-:0}" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \
  --env="CYCLONEDDS_URI=$DDS_URI" \
  --env="ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}" \
  --env="XAUTHORITY=$XAUTH" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$XAUTH:$XAUTH" \
  --volume="$REPO_ROOT:/external:rw" \
  "${USER_ARGS[@]}" \
  "$IMAGE" \
  bash -lc "$CONTAINER_BOOTSTRAP" \
  bash \
  "${CMD[@]}"
