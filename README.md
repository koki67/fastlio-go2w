# fastlio-go2w

FAST-LIO workspace for Unitree GO2-W with Livox MID-360 over a ROS 2 Humble stack.

The robot host can remain on its original Ubuntu / ROS distribution. On the GO2-W Jetson, this repository is intended to run the FAST-LIO stack inside the provided Ubuntu 22.04 / ROS 2 Humble container.

The repository mirrors the proven `dlio-go2w` structure, replacing the sensing and odometry stack with:

- `livox_ros_driver2` (Mid-360 ROS 2 driver)
- `FAST_LIO` (`ROS2` branch)
- `fastlio_go2w_bringup` (ROS 2 launch/config/rviz/adapter package)
- `go2w_description` (vendored URDF, with mounted Mid-360 frames)

## Table of contents

- [Repository layout](#repository-layout)
- [Submodules](#submodules)
- [Host vs container](#host-vs-container)
- [Robot-side workflow](#robot-side-workflow)
- [Desktop workflow](#desktop-workflow)
- [Attribution](#attribution)

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
│   ├── build_ws.sh
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

## Host vs container

Do not build `humble_ws` directly on the GO2-W Jetson host. The Jetson host may be Ubuntu 20.04 / ROS 2 Foxy, while this workspace targets Ubuntu 22.04 / ROS 2 Humble inside Docker.

`docker build` creates the ARM64 Humble runtime image and installs system dependencies such as Livox-SDK2 and `ros-humble-pcl-ros`. It does not build this repository's ROS workspace, because the repository is mounted into the container at runtime as `/external`.

`scripts/setup_ws.sh` prepares the source tree:

- syncs and initializes git submodules
- copies `humble_ws/src/livox_ros_driver2/package_ROS2.xml` to `package.xml`

It does not require ROS. It is safe to run multiple times. Run it before the first workspace build and after submodule updates. The recommended robot workflow runs it inside the container immediately before `scripts/build_ws.sh`.

`scripts/build_ws.sh` performs the ROS workspace build. Run it inside the Humble container, not on the Jetson host.

If old build artifacts were created as root, reset them once from the host:

```bash
sudo rm -rf humble_ws/build humble_ws/install humble_ws/log
```

## Robot-side workflow

From the Jetson host, build or refresh the Docker image when the Dockerfile or system dependencies change:

```bash
cd ~/Projects/fastlio-go2w
docker build -f docker/robot/Dockerfile -t fastlio-go2w:latest .
```

Build the mounted ROS workspace inside the container. Use `--host-user` for build commands so generated `build/`, `install/`, and `log/` files remain owned by the Jetson user:

```bash
bash docker/robot/run.sh --host-user bash -lc 'cd /external && bash scripts/setup_ws.sh && bash scripts/build_ws.sh'
```

After normal source-code changes, rebuild only the workspace inside the container:

```bash
bash docker/robot/run.sh --host-user bash -lc 'cd /external && bash scripts/build_ws.sh'
```

Start an interactive robot container shell:

```bash
bash docker/robot/run.sh
```

By default, `bash docker/robot/run.sh` sources these inside the container shell before running your command:

- `/opt/ros/humble/setup.bash`
- `/external/humble_ws/install/setup.bash` (if present)

Start live FAST-LIO from inside the container:

```bash
catmux_create_session /external/catmux/fastlio.yaml
```

Record raw sensor bags for replay/reconstruction:

```bash
catmux_create_session /external/catmux/record_raw.yaml
```

By default the robot container binds CycloneDDS to the onboard `eth0` interface, which is the robot/sensor DDS network. To also expose the ROS graph to a desktop RViz session over Wi-Fi, start the robot container with remote DDS enabled:

```bash
bash docker/robot/run.sh --remote-viz
```

This keeps `eth0` for the robot/internal graph and adds `wlan0` for the remote desktop. If the robot uses a different interface name, pass it explicitly:

```bash
bash docker/robot/run.sh --remote-viz --remote-viz-iface wlan1
```

Use `--robot-iface <iface>` if the onboard robot DDS interface is not `eth0`. `ROS_DOMAIN_ID` is forwarded into the container and defaults to `0`.

## Desktop workflow

Use the devcontainer for visualization and bag checks. For a full visual TF check:

```bash
bash scripts/fastlio/check_tf.sh
```

For live RViz while streaming from robot:

```bash
bash scripts/fastlio/live_rviz.sh
```

`live_rviz.sh` defaults to the interface `enp97s0`. If your desktop uses a different interface, specify it via `--iface`.

Use the desktop interface connected to the robot Wi-Fi network and the same `ROS_DOMAIN_ID` as the robot container.

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
