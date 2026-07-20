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
- [Headless offline processing and saved results](#headless-offline-processing-and-saved-results)
- [Attribution](#attribution)

## Repository layout

```
fastlio-go2w/
├── config/
│   ├── cyclonedds.xml
│   └── sensor/go2w_mid360_calibration.yaml
├── docker/
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
│   ├── diagnostics/
│   │   └── check_tf.sh
│   ├── fastlio/
│   │   ├── replay.sh
│   │   └── live_rviz.sh
│   └── offline/
│       ├── run_multilidar_experiment.sh
│       └── visualize_multilidar_run.sh
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
docker build -f docker/Dockerfile -t fastlio-go2w:latest .
```

Build the mounted ROS workspace inside the container. Use `--host-user` for build commands so generated `build/`, `install/`, and `log/` files remain owned by the Jetson user:

```bash
bash docker/run.sh --host-user bash -lc 'cd /external && bash scripts/setup_ws.sh && bash scripts/build_ws.sh'
```

After normal source-code changes, rebuild only the workspace inside the container:

```bash
bash docker/run.sh --host-user bash -lc 'cd /external && bash scripts/build_ws.sh'
```

Start an interactive robot container shell:

```bash
bash docker/run.sh
```

By default, `bash docker/run.sh` sources these inside the container shell before running your command:

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
bash docker/run.sh --remote-viz
```

This keeps `eth0` for the robot/internal graph and adds `wlan0` for the remote desktop. If the robot uses a different interface name, pass it explicitly:

```bash
bash docker/run.sh --remote-viz --remote-viz-iface wlan1
```

Use `--robot-iface <iface>` if the onboard robot DDS interface is not `eth0`. `ROS_DOMAIN_ID` is forwarded into the container and defaults to `0`.

## Desktop workflow

Use the devcontainer for visualization and bag checks. For a full visual TF check:

```bash
bash scripts/diagnostics/check_tf.sh
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

For the Issue #7 offline odometry comparison, run the same bag once with each
profile. Each command starts FAST-LIO, RViz, and bag playback together:

```bash
BAG=/mnt/go2w-experiment-recorder/bags/experiment_long3_20260714_014823
bash scripts/fastlio/replay.sh "$BAG" --profile baseline
bash scripts/fastlio/replay.sh "$BAG" --profile fused-high
bash scripts/fastlio/replay.sh "$BAG" --profile fused-matched
```

`baseline` uses only the MID-360 input. `fused-high` adds a high-density Pandar
XT16 sample to the MID-360 cloud and has the highest point count.
`fused-matched` uses stronger input downsampling; it is the density-matched
profile used by the controlled experiment documented below. The interactive
replay profiles automatically select visualization-enabled FAST-LIO YAMLs, so
`--config` is not needed for this visual comparison. Stop each run with Ctrl-C
before starting the next one.

The devcontainer mounts the external bag directories as read-only and the
offline result directory as read-write:

- `/mnt/data1/experimental_data/go2w-experiment-recorder/bags` at
  `/mnt/go2w-experiment-recorder/bags`
- `/mnt/data1/experimental_data/fastlio-go2w/bags` at
  `/mnt/fastlio-go2w/bags`
- `/mnt/data1/experimental_data/fastlio-go2w/results` at
  `/mnt/fastlio-go2w/results`

This lets you replay a bag stored outside this repository without copying it:

```bash
bash scripts/fastlio/replay.sh /mnt/go2w-experiment-recorder/bags/raw_YYYYMMDD_HHMMSS
bash scripts/fastlio/replay.sh /mnt/fastlio-go2w/bags/raw_YYYYMMDD_HHMMSS
```

After pulling this configuration change, use **Dev Containers: Rebuild and
Reopen in Container** once to apply the new mount.

To replay with a specific FAST-LIO parameter YAML, pass `--config`.
Without `--config`, each profile selects the following default:

- `legacy`: `mid360_go2w.yaml`
- `baseline`: `mid360_go2w_accuracy_dense_false.yaml`
- `fused-high` / `fused-matched`: `mid360_xt16_fused_accuracy_dense_false.yaml`

```bash
bash scripts/fastlio/replay.sh bags/raw_YYYYMMDD_HHMMSS --config mid360_go2w_accuracy.yaml
bash scripts/fastlio/replay.sh bags/raw_YYYYMMDD_HHMMSS --config humble_ws/src/fastlio_go2w_bringup/config/mid360_go2w_viz_dense.yaml
```

`--config` accepts an absolute path, a path relative to the current directory
or repository root, or a file name under
`humble_ws/src/fastlio_go2w_bringup/config/`.

The option is a complete FAST-LIO parameter override; it does not select the
RViz layout. For a fused profile, start from
`mid360_xt16_fused_accuracy_dense_false.yaml` so that `common.lid_topic`
remains `/livox/lidar_fused` and `preprocess.scan_line`
remains `20`. The MID-360 visualization configs subscribe to `/livox/lidar`
and therefore intentionally run MID-360-only processing even if a fused
profile starts the fusion node.

RViz is enabled by default for replay. Add `--no-rviz` if you need headless
replay. All replay defaults set `publish.map_en: true`, and the bundled RViz
layout enables `/Laser_map`, so the accumulated lower-density FAST-LIO map is
visible in the same way as on the main branch.

## Headless offline processing and saved results

The offline workflow separates computation from visualization. During the
bounded playback of an existing sensor bag, the runner starts FAST-LIO without
RViz, records the computed outputs, and exits after the bag and processing
queue finish. It then generates final map and trajectory artifacts. A separate
command can visualize those frozen artifacts later without rerunning FAST-LIO
or changing the saved odometry.

All three experiment profiles support this workflow:

| Profile | Processing input |
|---|---|
| `baseline` | MID-360 only |
| `fused-high` | MID-360 + Pandar XT16, higher retained density |
| `fused-matched` | MID-360 + Pandar XT16, density-matched sampling |

Run the commands from the repository root in the ROS 2 Humble project
container. This example creates a `fused-matched` result:

```bash
BAG=/mnt/go2w-experiment-recorder/bags/experiment_long3_20260714_014823
RESULTS_ROOT="${FASTLIO_RESULTS_ROOT:-$PWD/results}"
OUT="$RESULTS_ROOT/multilidar/long3/fused-matched"

bash scripts/offline/run_multilidar_experiment.sh \
  "$BAG" --profile fused-matched --rate 1.0 --output "$OUT"
```

The runner starts bag playback paused, verifies the ROS endpoints and live
parameters, and records only the processing outputs needed for artifacts and
diagnostics: `/odom`, `/Odometry`, `/cloud_registered`, and fusion diagnostics.
The headless FAST-LIO configurations disable the live cumulative `/Laser_map`,
`/path`, and unused body-frame cloud publishers. A successful analyzed run
contains, among other provenance and diagnostic files:

- `rosbag/`: the frozen output topics
- `map_voxelized.pcd`: the final accumulated registered-scan map
- `map_preview.pcd`: a bounded-size preview of that map for RViz
- `trajectory.csv` and `trajectory_camera_init.csv`: frozen trajectories
- `summary.json`: map, trajectory, resource, and artifact metadata

The devcontainer sets `FASTLIO_RESULTS_ROOT=/mnt/fastlio-go2w/results`, which
is backed by the host data disk. `docker/run.sh` uses the same external
directory when it exists and otherwise falls back to the repository's mounted
`results/` directory. Outside these containers, the runner defaults to
`<repository>/results`. If `--output` is omitted, it creates a timestamped
directory below `$FASTLIO_RESULTS_ROOT/multilidar/<bag-name>/`. Use `--no-analyze`
only when the result bag should be saved without immediately generating the
PCD maps and trajectory CSVs.

To display the completed map and trajectory in RViz:

```bash
bash scripts/offline/visualize_multilidar_run.sh "$OUT"
```

Static mode publishes the saved preview map and trajectory. Dynamic mode also
replays the already-computed `/cloud_registered` and `/Odometry` outputs:

```bash
bash scripts/offline/visualize_multilidar_run.sh "$OUT" --dynamic --rate 2.0
```

Neither visualization mode runs FAST-LIO or the fusion node. See the
[offline result artifact workflow](docs/offline-result-artifacts.md) for the
full profile comparison procedure, artifact definitions, validation, and
troubleshooting.

## Attribution

FAST-LIO algorithm and many launch/build conventions follow upstream projects:

- `hku-mars/FAST_LIO`
- `Livox-SDK/livox_ros_driver2`
- `Unitree GO2-W description assets` in `frontier-fw-go2w`
