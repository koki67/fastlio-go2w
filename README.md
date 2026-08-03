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
│   ├── fastlio_go2w_bringup/      (launch + odometry adapter + configs)
│   ├── fastlio_go2w_livox/        (PointCloud2-to-CustomMsg replay adapter)
│   └── fastlio_go2w_fusion/       (experimental MID-360 + XT16 fusion)
├── scripts/
│   ├── setup_ws.sh
│   ├── build_ws.sh
│   ├── diagnostics/
│   │   └── check_tf.sh
│   ├── fastlio/
│   │   ├── replay.sh
│   │   └── live_rviz.sh
│   └── offline/
│       ├── run_fastlio_offline.sh
│       ├── run_multilidar_experiment.sh
│       ├── view_raw_scans.sh
│       ├── publish_offline_frame_alignment.py
│       ├── replay_fastlio_artifacts.py
│       ├── visualize_fastlio_run.sh
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

### View raw current scans without FAST-LIO

The current recorder bags store both LiDAR clouds as PointCloud2, so they can
be displayed directly without FAST-LIO, registration, or map accumulation:

```bash
BAG=/mnt/go2w-experiment-recorder/bags/experiment_corridor-zigzag-one_20260801_090747
bash scripts/offline/view_raw_scans.sh "$BAG" --loop
```

When running on the host rather than in the devcontainer, use the host path
under `/mnt/data1/experimental_data/go2w-experiment-recorder/bags/` instead.
The script sources an installed ROS 2 desktop environment when necessary and
does not require this colcon workspace to be built.

RViz starts with two independent displays enabled:

- `MID-360 current scan (cyan)` reads `/livox/lidar`.
- `XT16 current scan (orange)` reads `/points_raw`.

Use the checkboxes in the RViz Displays panel to show either sensor alone or
both together. Each display has `Decay Time: 0`; a new PointCloud2 message
replaces the previous scan instead of leaving a trail or building a map. The
script replays only those two topics and leaves the source bag unchanged.

The recorded frames are `livox_frame` and `hesai_lidar`. The viewer publishes
the calibrated `base_link -> livox_frame` mounting transform and uses
`base_link` as the RViz fixed frame. The Grid is therefore parallel to the
nominal robot XY plane instead of being tilted by the MID-360's approximately
18.3 degree mounting pitch. This is a static display frame, not gravity or
odometry compensation for robot roll/pitch while moving.

For the combined view, the script also publishes the branch's initial composed
XT16-to-MID-360 mounting transform. It is sufficient for a first visual
comparison, but it is not a dedicated dual-LiDAR calibration; double surfaces
or small offsets in the overlap must not be interpreted as a sensor range
error without calibrating the extrinsic first.

The viewer deliberately accepts only the current recorder contract where both
topics are `sensor_msgs/msg/PointCloud2`. It reports a clear error for a bag
with a missing topic, empty topic, or older MID-360 `CustomMsg` recording.
Useful playback options include `--rate`, `--start-offset`, `--loop`, and
`--domain-id`; run `bash scripts/offline/view_raw_scans.sh --help` for details.

For replaying a saved bag:

```bash
bash scripts/fastlio/replay.sh bags/raw_YYYYMMDD_HHMMSS
```

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

Replay reads `/livox/lidar` from `metadata.yaml` before starting ROS and prints
the resolved format. Two exact input types are supported:

| Recorded type | Resolved format | Processing path |
| --- | --- | --- |
| `livox_ros_driver2/msg/CustomMsg` | `custom-msg` | Existing FAST-LIO Livox callback, no adapter |
| `sensor_msgs/msg/PointCloud2` | `pointcloud2` | `/livox/lidar` -> Livox adapter -> `/livox/lidar_fastlio` CustomMsg |

The default `--lidar-format auto` is fail-closed for missing, empty,
unsupported, or multi-type metadata. An explicit value is useful for operator
verification, but it must agree with metadata:

```bash
bash scripts/fastlio/replay.sh "$BAG" --lidar-format pointcloud2
```

The adapter reads the Livox field schema and per-point absolute nanosecond
timestamp; it does not assume the current 26-byte stride. Epoch-scale FLOAT64
timestamps have a 256 ns ULP in the current bags, so a first-point negative
delta no larger than half an ULP is diagnosed and clamped to zero. Larger
negative deltas and all other schema, layout, range, or finite-value anomalies
drop the complete frame.

The PointCloud2 bag is not sent directly to FAST-LIO as `lidar_type=4`: that
handler does not consume this producer's `timestamp` contract and expects a
different reflectivity field. XT16 and multi-LiDAR integration remain isolated
behind the explicit experimental profiles described below; the default
`legacy` profile remains MID-360-only.

This branch additionally provides four controlled replay profiles:

| Profile | Processing input |
| --- | --- |
| `baseline` | MID-360 only |
| `fused-full` | MID-360 + Pandar XT16, stride 1 for both inputs |
| `fused-high` | MID-360 + Pandar XT16, higher retained density |
| `fused-matched` | MID-360 + Pandar XT16, density-matched sampling |

The fused profiles transform the XT16 points into the MID-360 frame and publish
one Livox-style cloud on `/livox/lidar_fused` before FAST-LIO. They do not
modify FAST-LIO into a native multi-sensor estimator. Run the same bag with
each profile for an interactive comparison:

```bash
BAG=/mnt/go2w-experiment-recorder/bags/experiment_long3_20260714_014823
bash scripts/fastlio/replay.sh "$BAG" --profile baseline
bash scripts/fastlio/replay.sh "$BAG" --profile fused-full
bash scripts/fastlio/replay.sh "$BAG" --profile fused-high
bash scripts/fastlio/replay.sh "$BAG" --profile fused-matched
```

The metadata-based `--lidar-format` selection applies to the MID-360 input in
all profiles. In a fused profile, a PointCloud2 MID-360 bag is adapted first,
then combined with `/points_raw` from the XT16. `--debug-cloud` enables the
source-labelled fused debug cloud.

After pulling this configuration change, use **Dev Containers: Rebuild and
Reopen in Container** once to apply the new mount.

To replay with a specific FAST-LIO parameter YAML, pass `--config`.
Without `--config`, replay uses `mid360_go2w.yaml`.

```bash
bash scripts/fastlio/replay.sh bags/raw_YYYYMMDD_HHMMSS --config mid360_go2w_accuracy.yaml
bash scripts/fastlio/replay.sh bags/raw_YYYYMMDD_HHMMSS --config humble_ws/src/fastlio_go2w_bringup/config/mid360_go2w_viz_dense.yaml
```

`--config` accepts an absolute path, a path relative to the current directory
or repository root, or a file name under
`humble_ws/src/fastlio_go2w_bringup/config/`.

RViz is enabled by default for replay. Add `--no-rviz` if you need headless replay.

## Headless offline processing and saved results

The offline workflow separates FAST-LIO computation from visualization. It
plays an existing MID-360 bag once, runs FAST-LIO without a GUI, records only
the registered clouds and odometry needed for final artifacts, and exits after
the bag and processing queue finish. The saved map and trajectory can be
visualized later without rerunning FAST-LIO.

Rebuild the workspace after pulling this feature. Then run the following from
the repository root in the ROS 2 Humble project container:

```bash
BAG=/mnt/go2w-experiment-recorder/bags/experiment_long3_20260714_014823
RESULTS_ROOT="${FASTLIO_RESULTS_ROOT:-$PWD/results}"
OUT="$RESULTS_ROOT/fastlio/long3/baseline"

bash scripts/offline/run_fastlio_offline.sh \
  "$BAG" --rate 1.0 --output "$OUT"
```

The runner reads only `/livox/lidar` and `/livox/imu`. It starts playback
paused, verifies all processing and recording endpoints, validates the live
FAST-LIO parameters, and then resumes the bag. The headless configuration
retains the accuracy tuning while disabling cumulative `/Laser_map`, `/path`,
the unused body-frame cloud, and FAST-LIO's built-in PCD writer.
It uses the same metadata detector and `--lidar-format` contract as interactive
replay. PointCloud2 runs additionally record adapter diagnostics and save the
final counters as `livox_adapter_diagnostics.json`; CustomMsg runs explicitly
record that the adapter was disabled in `manifest.json`.

A successful analyzed run contains:

- `rosbag/`: frozen `/odom`, `/Odometry`, and `/cloud_registered`
- `map_voxelized.pcd`: final accumulated registered-scan map
- `map_preview.pcd`: bounded-size RViz preview
- `trajectory.csv` and `trajectory_camera_init.csv`: frozen trajectories
- `summary.json`: map, trajectory, resource, and artifact metadata
- configuration snapshots, hashes, process metrics, and logs
- for PointCloud2 input, `livox_adapter_diagnostics.json` and its source/runtime hashes

The devcontainer sets
`FASTLIO_RESULTS_ROOT=/mnt/fastlio-go2w/results`, backed by the host data
disk. `docker/run.sh` uses the same external directory when it exists and
otherwise falls back to the repository's mounted `results/` directory.
Outside these containers, the default is `<repository>/results`. The output
directory must be empty.

Display the completed map and trajectory in RViz:

```bash
bash scripts/offline/visualize_fastlio_run.sh "$OUT"
```

Static mode publishes the frozen preview map and trajectory. Dynamic mode
starts with an empty map and path, replays the already-computed
`/cloud_registered` and `/odom`, and incrementally adds new 0.2 m map
voxels and trajectory poses:

```bash
bash scripts/offline/visualize_fastlio_run.sh "$OUT" --dynamic --rate 2.0
```

The current registered scan remains visible separately while the accumulated
map and traveled path grow. Dynamic replay does not publish the completed PCD
or trajectory CSV up front. Both modes reconstruct the saved `odom ->
camera_init` display transform from the MID-360 calibration, so the RViz Grid
is parallel to the initial robot `base_link` XY plane. Neither visualization
mode runs FAST-LIO. See the
[offline result artifact workflow](docs/offline-result-artifacts.md) for
artifact definitions, validation, comparison, and troubleshooting.

A completed run with finite clouds and odometry establishes software replay
operation only. It does not establish ground-truth trajectory accuracy,
Jetson performance, live-hardware behavior, or physical deployment readiness.

### Experimental MID-360 + XT16 offline comparison

The isolated fusion workflow remains available alongside the generic
MID-360-only workflow imported from `main`. It records the baseline and fused
profiles separately so their outputs and resource measurements remain
attributable:

```bash
BAG=/mnt/go2w-experiment-recorder/bags/experiment_long3_20260714_014823
RESULTS_ROOT="${FASTLIO_RESULTS_ROOT:-$PWD/results}"
OUT="$RESULTS_ROOT/multilidar/long3/fused-matched"

bash scripts/offline/run_multilidar_experiment.sh \
  "$BAG" --profile fused-matched --rate 1.0 --output "$OUT"
bash scripts/offline/visualize_multilidar_run.sh "$OUT"
```

When `--output` is omitted, the run directory is named
`<profile>-<UTC timestamp>` (for example,
`fused-matched-20260801T163403Z`) beneath the bag-specific results directory.

The batch multi-LiDAR runner currently retains its original CustomMsg input
contract. The interactive replay command above supports both recorded MID-360
formats through the imported adapter. See
[the Issue #7 experiment report](docs/experiments/issue-7-mid360-xt16.md) for
the profile definitions, evidence, and interpretation limits.

## Attribution

FAST-LIO algorithm and many launch/build conventions follow upstream projects:

- `hku-mars/FAST_LIO`
- `Livox-SDK/livox_ros_driver2`
- `Unitree GO2-W description assets` in `frontier-fw-go2w`
