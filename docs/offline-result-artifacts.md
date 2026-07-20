# Offline FAST-LIO result artifacts and visualization

This workflow processes a recorded Livox MID-360 bag once without RViz, saves
the computed FAST-LIO outputs, generates deterministic map and trajectory
artifacts, and visualizes those frozen results later. It is independent of the
interactive replay workflow and does not start the robot sensor stack.

The source bag is read-only. The processing run consumes only /livox/lidar and
/livox/imu; no secondary LiDAR topic is required.

## Data flow

~~~text
raw MID-360 bag
  /livox/lidar + /livox/imu
        |
        | scripts/offline/run_fastlio_offline.sh
        | FAST-LIO + odom adapter + result recording + resource sampling
        v
run directory
  rosbag/{/odom, /Odometry, /cloud_registered}
        |
        | deterministic post-processing
        v
  summary.json + trajectory CSVs + voxelized/preview PCD maps
        |
        | scripts/offline/visualize_fastlio_run.sh
        v
  static final-map view or dynamic replay of already-computed output
~~~

The runner starts source-bag playback paused, waits for all processing and
recording endpoints, validates the live FAST-LIO parameters, and only then
resumes playback. After the player exits, it waits until the result bag remains
unchanged for a stable interval, stops the measured processes, validates the
finalized bag, and runs the analyzer.

## Build and storage setup

Build the ROS 2 Humble workspace before the first run and after changes to the
launch file or offline configuration.

The desktop devcontainer maps the host result directory

~~~text
/mnt/data1/experimental_data/fastlio-go2w/results
~~~

to /mnt/fastlio-go2w/results and sets:

~~~bash
FASTLIO_RESULTS_ROOT=/mnt/fastlio-go2w/results
~~~

docker/run.sh uses the same data-disk directory when it exists. If it does not
exist, the Docker runner uses /external/results, which is retained through the
repository bind mount. When the offline script runs outside either container
and the variable is unset, it uses <repository>/results.

Rebuild and reopen the devcontainer once after the mount configuration changes.

## Run headless FAST-LIO

Run from the repository root inside the project container:

~~~bash
BAG=/mnt/go2w-experiment-recorder/bags/experiment_long3_20260714_014823
RESULTS_ROOT="${FASTLIO_RESULTS_ROOT:-$PWD/results}"
OUT="$RESULTS_ROOT/fastlio/long3/baseline"

bash scripts/offline/run_fastlio_offline.sh \
  "$BAG" \
  --rate 1.0 \
  --output "$OUT"
~~~

The output directory must not contain files from an earlier run. If --output
is omitted, the runner creates a UTC-timestamped directory below:

~~~text
$FASTLIO_RESULTS_ROOT/fastlio/<bag-name>/
~~~

Useful options are:

| Option | Purpose |
| --- | --- |
| --start-offset SEC | Start within the source bag |
| --duration SEC | Stop after an approximate bag duration; smoke tests only |
| --rate RATE | Source playback multiplier; default 1.0 |
| --domain-id ID | Isolated ROS domain; default 77 |
| --config YAML | Explicit compatible headless FAST-LIO configuration |
| --no-analyze | Keep the result bag without immediately generating PCD/CSV artifacts |
| --map-voxel-size M | Final map voxel edge; default 0.20 m |
| --preview-max-points N | RViz preview point cap; default 500000 |
| --plane-random-seed N | Deterministic local-plane sample seed; default 7 |

A --duration run is useful for endpoint and pipeline smoke testing, but it is
not a complete map and should not be compared with full-bag results.

## Headless FAST-LIO contract

The default mid360_go2w_accuracy_offline.yaml keeps the tuning from
mid360_go2w_accuracy_dense_false.yaml and changes only these publishers:

~~~yaml
publish:
  map_en: false
  path_en: false
  scan_bodyframe_pub_en: false
~~~

It keeps scan_publish_en enabled because /cloud_registered is the source of the
final map. The built-in PCD writer remains disabled. The runner validates the
live parameters before playback, including:

- common.lid_topic is /livox/lidar
- preprocess.scan_line is 4
- cumulative map, path, effect-map, and body-frame publishers are disabled
- registered-cloud output is enabled
- dense output, runtime position log, and built-in PCD saving are disabled

A custom configuration supplied with --config must preserve this input and
output contract. The selected YAML, live parameter dump, launch source hash,
runtime executable hashes, source-bag metadata hash, and Git revision are saved
with each run.

## Generated artifacts

Analysis is enabled by default. A successful run contains at least:

| Artifact | Purpose |
| --- | --- |
| manifest.json | Source bag, playback settings, hashes, process state, and artifact inventory |
| fastlio_config.yaml | Exact FAST-LIO YAML snapshot executed by the run |
| fastlio_mapping.yaml | Live FAST-LIO parameter dump |
| commands.log | Shell-escaped launch, record, play, and analysis commands |
| rosbag/ | Frozen /odom, /Odometry, and /cloud_registered |
| resource_metrics.csv | Per-process CPU-time and RSS samples |
| resource_summary.json | Resource-sampling summary |
| trajectory.csv | Primary trajectory; /odom is preferred |
| trajectory_camera_init.csv | Raw FAST-LIO /Odometry trajectory used by the viewer |
| map_voxelized.pcd | Voxelized accumulation of all finite registered points |
| map_preview.pcd | Deterministic bounded-size map used by RViz |
| summary.json | Trajectory, map, resource, provenance, and artifact hashes |
| analysis.log | Analyzer output or error details |

The analyzer preserves the frame IDs found in the recorded data. It rejects
mixed nonempty cloud frames. The viewer uses the /Odometry trajectory, verifies
that its frame matches the map frame, and passes that validated frame to RViz.

map_voxelized.pcd contains x, y, z, and count; count is the number of registered
points accumulated into each voxel. Voxel keys are sorted before writing,
preview points are selected at stable indexes, and local-plane sampling uses
the configured random seed.

## Visualize a completed result

Run the viewer in a graphical ROS 2 Humble environment with the workspace
built and sourced. Use an unused ROS_DOMAIN_ID if another ROS graph is active.

### Static final map

~~~bash
ROS_DOMAIN_ID=78 bash scripts/offline/visualize_fastlio_run.sh "$OUT"
~~~

Static mode publishes map_preview.pcd as /offline/map and the matching-frame
trajectory as /offline/path. It does not play a bag or run FAST-LIO. This is
the preferred mode for final-map inspection and screenshots.

### Dynamic saved-result replay

~~~bash
ROS_DOMAIN_ID=78 bash scripts/offline/visualize_fastlio_run.sh \
  "$OUT" --dynamic --rate 1.0
~~~

Dynamic mode keeps the frozen map and path visible and additionally replays
only /cloud_registered and /Odometry from the result bag. The rate changes
animation speed but cannot change the saved odometry.

To validate artifacts without starting ROS publishers or RViz:

~~~bash
python3 scripts/offline/publish_fastlio_artifacts.py \
  "$OUT" --validate-only
~~~

## Compare repeated runs

The analyzer can compare two or more completed runs produced from the same
source bag and compatible analysis settings:

~~~bash
python3 scripts/offline/analyze_fastlio_run.py compare \
  results/fastlio/run-a \
  results/fastlio/run-b \
  --labels run-a run-b \
  --output results/fastlio/comparison_summary.json
~~~

Comparison mode checks source-bag identity, playback settings, trajectory
thresholds, voxel size, preview cap, PCD format, and deterministic analysis
parameters before reporting trajectory differences and resource metrics.
Without ground truth, those differences are consistency diagnostics rather
than absolute trajectory error.

## Troubleshooting

### The runner reports a stale workspace

Rebuild the Humble workspace. The runner requires the installed launch file to
match the source file byte-for-byte and checks that the FAST-LIO and odom
adapter executables exist in the same overlay.

### The output directory is not empty

Choose a new directory. The runner will not overwrite a partial or completed
run because mixing artifacts would invalidate provenance and hashes.

### RViz opens but the map is blank

Validate the run first:

~~~bash
python3 scripts/offline/publish_fastlio_artifacts.py RUN_DIR --validate-only
~~~

The command checks PCD structure, hashes, point counts, trajectory data, and
map/trajectory frame compatibility.

### Dynamic mode has no animation

Confirm that RUN_DIR/rosbag exists and contains /cloud_registered and
/Odometry:

~~~bash
ros2 bag info RUN_DIR/rosbag
~~~

Static visualization only requires the analyzed PCD and trajectory artifacts.
