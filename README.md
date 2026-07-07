# fastlio-go2w

FAST-LIO workspace for Unitree GO2-W with Livox MID-360 over a shared ROS 2 Humble stack.

The repository mirrors the proven `dlio-go2w` structure, replacing the sensing and odometry stack with:

- `livox_ros_driver2` (Mid-360 ROS 2 driver)
- `FAST_LIO` (`ROS2` branch)
- `fastlio_go2w_bringup` (new ROS 2 launch/config/rvt/adapter package)
- `go2w_description` (vendored URDF, with mounted Mid-360 frames)

## Repository layout

```
fastlio-go2w/
├── config/
│   ├── cyclonedds.xml
│   └── sensor/go2w_mid360_calibration.yaml
├── docker/robot/
│   ├── Dockerfile
│   └── run.sh
├── bags/
│   └── .gitkeep
├── humble_ws/src/
│   ├── FAST_LIO/                 (submodule, a.k.a. FAST-LIO2 ROS2)
│   ├── livox_ros_driver2/         (submodule)
│   ├── go2w_description/          (vendored from frontier-fw-go2w)
│   └── fastlio_go2w_bringup/      (launch + adapter + configs)
├── scripts/
│   ├── setup_ws.sh
│   └── fastlio/
│       ├── replay.sh
│       ├── live_rviz.sh
│       └── check_tf.sh
└── catmux/
    ├── fastlio.yaml
    └── record_raw.yaml
```

`config/sensor/go2w_mid360_calibration.yaml` is the single source of truth for topic names and extrinsics in this workspace.

## Submodules

| Package | Repository | Branch | Commit pin |
|---|---|---|---|
| `livox_ros_driver2` | https://github.com/Livox-SDK/livox_ros_driver2 | `master` | `13eb05e` |
| `FAST_LIO` | https://github.com/hku-mars/FAST_LIO | `ROS2` | `a4743b0` |

## Setup

Initialize submodules:

```bash
git submodule sync --recursive
git submodule update --init --recursive
```

Copy the ROS2 driver manifest after checkout:

```bash
cp humble_ws/src/livox_ros_driver2/package_ROS2.xml humble_ws/src/livox_ros_driver2/package.xml
```

Or simply run:

```bash
bash scripts/setup_ws.sh
```

Build (Ubuntu 22.04 / ROS 2 Humble):

```bash
cd humble_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
source install/setup.bash
```

## Robot-side workflow

From Jetson, build and start the ARM64 image:

```bash
docker build -f docker/robot/Dockerfile -t fastlio-go2w:latest .
bash docker/robot/run.sh
```

By default, `bash docker/robot/run.sh` now sources these inside the container shell before running your command:

- `/opt/ros/humble/setup.bash`
- `/external/humble_ws/install/setup.bash` (if present)

Start live FAST-LIO:

```bash
# Build with catmux in the provided robot image (Dockerfile installs it), then:
catmux_create_session /external/catmux/fastlio.yaml
```

If you run a custom robot image, install Catmux first:

```bash
python3 -m pip install --user catmux
```

Then start:

```bash
catmux_create_session /external/catmux/fastlio.yaml
```

Record raw sensor bags for replay/reconstruction:

```bash
catmux_create_session /external/catmux/record_raw.yaml
```

By default the robot container binds CycloneDDS to the onboard `eth0`
interface, which is the robot/sensor DDS network. To also expose the ROS graph
to a desktop RViz session over Wi-Fi, start the robot container with remote DDS
enabled:

```bash
bash docker/robot/run.sh --remote-viz
```

This keeps `eth0` for the robot/internal graph and adds `wlan0` for the remote
desktop. If the robot uses a different interface name, pass it explicitly:

```bash
bash docker/robot/run.sh --remote-viz --remote-viz-iface wlan1
```

Use `--robot-iface <iface>` if the onboard robot DDS interface is not `eth0`.
`ROS_DOMAIN_ID` is forwarded into the container and defaults to `0`.

## Desktop workflow

Use the devcontainer for visualization and bag checks. For a full visual TF check:

```bash
bash scripts/fastlio/check_tf.sh
```

For live RViz while streaming from robot:

```bash
bash scripts/fastlio/live_rviz.sh
```

`live_rviz.sh` defaults to the interface `enp97s0`. If your desktop uses a different
interface, specify it via `--iface`.

Use the desktop interface connected to the robot Wi-Fi network and the same
`ROS_DOMAIN_ID` as the robot container.

For replaying a saved bag:

```bash
bash scripts/fastlio/replay.sh bags/raw_YYYYMMDD_HHMMSS
```

RViz is enabled by default for replay. Add `--no-rviz` if you need headless replay.

## Attribution

FAST-LIO algorithm and many launch/build conventions follow upstream projects:

- `hku-mars/FAST_LIO`
- `Livox-SDK/livox_ros_driver2`
- `Unitree GO2-W description assets` in `frontier-fw-go2w`
