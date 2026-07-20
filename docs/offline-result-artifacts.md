# Offline FAST-LIO result artifacts and visualization

This workflow is for controlled, offline comparison of MID-360-only FAST-LIO
and MID-360 + Pandar XT16 fusion. It processes an existing raw rosbag once,
stores the computed odometry and registered clouds in a result bag, and then
visualizes or analyzes that frozen result. RViz is deliberately not part of the
measurement run.

The source bag is not modified, and none of these commands starts a robot
sensor stack.

## Data flow

```text
raw sensor bag
  /livox/lidar + /livox/imu + /points_raw
        |
        | scripts/offline/run_multilidar_experiment.sh
        | FAST-LIO, optional fusion, result recording, resource sampling
        v
run directory
  rosbag/{/odom, /Odometry, /cloud_registered, fusion diagnostics}
        |
        | deterministic post-processing
        v
  summary.json + trajectory CSVs + voxelized/preview PCD maps
        |
        | scripts/offline/visualize_multilidar_run.sh
        v
  static final-map view or dynamic replay of already-computed output
```

The headless runner starts the source bag paused, waits for all processing and
recording endpoints, validates the live FAST-LIO parameters, and only then
resumes playback. It records output topics rather than asking FAST-LIO to keep
publishing cumulative visualization messages. After playback and a short drain,
it stops all measured processes before running the analyzer.

## Run the baseline and fused-matched experiments

Build the ROS 2 Humble workspace first and run these commands from the
repository root in the project container. For the example dataset, the host
path is:

```text
/mnt/data1/experimental_data/go2w-experiment-recorder/bags/experiment_long3_20260714_014823
```

The devcontainer exposes the same directory as
`/mnt/go2w-experiment-recorder/bags`. The following commands use that container
path:

```bash
bash scripts/offline/run_multilidar_experiment.sh \
  /mnt/go2w-experiment-recorder/bags/experiment_long3_20260714_014823 \
  --profile baseline \
  --rate 1.0 \
  --output results/multilidar/long3/baseline

bash scripts/offline/run_multilidar_experiment.sh \
  /mnt/go2w-experiment-recorder/bags/experiment_long3_20260714_014823 \
  --profile fused-matched \
  --rate 1.0 \
  --output results/multilidar/long3/fused-matched
```

Run the profiles sequentially. Parallel runs compete for CPU, memory, and disk
bandwidth and therefore do not provide a fair resource or stability comparison.
The output directory must be empty. The devcontainer sets
`FASTLIO_RESULTS_ROOT=/mnt/fastlio-go2w/results`, backed by
`/mnt/data1/experimental_data/fastlio-go2w/results` on the host.
`docker/run.sh` uses that host directory when it exists and falls back to
`/external/results`; outside these containers, the default is
`<repository>/results`. If `--output` is omitted, the runner creates a
timestamped directory below
`$FASTLIO_RESULTS_ROOT/multilidar/<bag-name>/`.

`fused-high` remains available for the higher-density experiment:

```bash
bash scripts/offline/run_multilidar_experiment.sh \
  /mnt/go2w-experiment-recorder/bags/experiment_long3_20260714_014823 \
  --profile fused-high \
  --rate 1.0 \
  --output results/multilidar/long3/fused-high
```

The profile-specific front-end sampling is:

| Profile | FAST-LIO cloud | MID-360 selection | XT16 selection |
| --- | --- | ---: | ---: |
| `baseline` | `/livox/lidar` | no fusion-node selection | none |
| `fused-matched` | `/livox/lidar_fused` | every 6th valid point | every 22nd firing group |
| `fused-high` | `/livox/lidar_fused` | every 3rd valid point | every 3rd firing group |

An XT16 firing group retains all 16 rings. The two fused profiles differ only
in front-end density; they use the same fused FAST-LIO tuning by default.

## Default FAST-LIO configurations

The runner selects these headless defaults:

| Profile | Configuration |
| --- | --- |
| `baseline` | `mid360_go2w_accuracy_offline.yaml` |
| `fused-matched`, `fused-high` | `mid360_xt16_fused_accuracy_offline.yaml` |

The baseline file keeps the tuning from
`mid360_go2w_accuracy_dense_false.yaml`. The fused file keeps the same tuning,
apart from the input contract required by the combined cloud:

- baseline: `common.lid_topic: /livox/lidar` and `preprocess.scan_line: 4`
- fused: `common.lid_topic: /livox/lidar_fused` and
  `preprocess.scan_line: 20`

Both offline files use `point_filter_num: 1`, `max_iteration: 3`, 0.20 m surface
and map filters, and the same IMU noise, range, and extrinsic parameters. They
also use this output contract:

```yaml
publish:
  map_en: false
  path_en: false
  effect_map_en: false
  scan_publish_en: true
  dense_publish_en: false
  scan_bodyframe_pub_en: false
pcd_save:
  pcd_save_en: false
```

Thus FAST-LIO publishes the downsampled, registered scan
`/cloud_registered`, while cumulative `/Laser_map`, cumulative `/path`, and the
unused body-frame cloud are disabled. The runner reads the live node parameters
and fails before playback if the map, path, registered-scan, dense-scan, or
body-frame flags violate this contract. Keep effect-map and internal PCD saving
disabled as shown so they do not add unrelated output work.

### Override `--config`

`--config YAML` is still a complete FAST-LIO parameter override. It accepts an
existing path from the current directory, a repository-relative path, or a file
name under `humble_ws/src/fastlio_go2w_bringup/config/`:

```bash
bash scripts/offline/run_multilidar_experiment.sh \
  /mnt/go2w-experiment-recorder/bags/experiment_long3_20260714_014823 \
  --profile baseline \
  --config mid360_go2w_accuracy_offline.yaml \
  --output results/multilidar/long3/baseline-explicit
```

This example explicitly selects an existing compatible file. For custom tuning,
copy that offline YAML to a new file first, then pass the new path.

A baseline override must retain `/livox/lidar` and four scan lines. A fused
override must retain `/livox/lidar_fused` and 20 scan lines. Every override must
retain the headless output flags above. The runner copies the chosen file to
`fastlio_config.yaml` in the run directory, executes that snapshot, dumps the
live node parameters to `fastlio_mapping.yaml`, and records their hashes in the
manifest.

## Automatic analysis and artifacts

Analysis is enabled by default. A successful run contains at least:

| Artifact | Purpose |
| --- | --- |
| `manifest.json` | source bag, playback settings, profile, hashes, process state, and artifact inventory |
| `fastlio_config.yaml` | exact FAST-LIO YAML snapshot executed by the run |
| `fastlio_mapping.yaml` | live FAST-LIO parameter dump |
| `dual_lidar_fusion.yaml` | live fusion parameter dump for a fused profile |
| `commands.log` | shell-escaped launch, record, play, and analysis commands |
| `rosbag/` | frozen `/odom`, `/Odometry`, `/cloud_registered`, and fusion diagnostics |
| `resource_metrics.csv` | one-second process CPU/RSS samples |
| `resource_summary.json` | runner resource summary |
| `trajectory.csv` | primary trajectory; `/odom` is preferred, with `/Odometry` as fallback |
| `trajectory_camera_init.csv` | raw FAST-LIO `/Odometry` trajectory used by the viewer |
| `map_voxelized.pcd` | complete voxelized accumulation of all finite `/cloud_registered` points |
| `map_preview.pcd` | deterministic, bounded-size subset used by RViz |
| `summary.json` | trajectory, map, diagnostic, resource, provenance, and artifact-hash summary |
| `analysis.log` | analyzer output or error details |

The trajectory CSVs contain message and bag timestamps, elapsed time, position,
quaternion, `frame_id`, and `child_frame_id`. The analyzer preserves separate
records for `/odom` and `/Odometry` in `summary.json`.

FAST-LIO normally emits `/cloud_registered` and `/Odometry` in `camera_init`.
The analyzer records the actual cloud and trajectory frame IDs instead of
silently rewriting them. It rejects mixed nonempty cloud frames. The artifact
viewer selects the `/Odometry` trajectory, verifies that its single frame
matches the map frame, and passes that validated frame to RViz as its fixed
frame. The Orbit camera follows the fixed frame. These runs normally use
`camera_init`; the RViz file keeps that standalone default, but the wrapper
overrides it with the validated frame.

The PCD files contain `x`, `y`, `z`, and `count`, where `count` is the number of
raw registered points accumulated into that voxel. Binary PCD is the analyzer
default. Voxel keys are sorted before writing; preview points are selected at
stable, evenly spaced indexes; local-plane sampling uses a fixed random seed.
`summary.json` records SHA-256 hashes and sizes for the PCD and trajectory
artifacts as well as a hash of the analysis parameters.

The summary reports, among other values:

- odometry rate, gaps, non-finite samples, translation/orientation jumps, path
  length, and terminal displacement;
- map coverage, bounds, occupied voxels, local plane thickness, and planarity;
- per-process average CPU cores and peak RSS;
- fusion diagnostic level/message counts, numeric distributions, and final
  counter values.

These are relative stability and consistency diagnostics. The example bag has
no ground truth, so trajectory difference from the MID-only run is not ATE,
RPE, or absolute accuracy.

### Map-analysis options

The runner exposes the options most useful for consistent map comparisons:

- `--map-voxel-size M`: final PCD voxel edge, 0.20 m by default;
- `--preview-max-points N`: RViz preview cap, 500,000 by default;
- `--plane-random-seed N`: deterministic local-plane sample seed, 7 by
  default.

These options affect post-processing only; they do not change FAST-LIO
odometry. Use the same values for every compared profile. A lower preview cap
reduces RViz memory and graphics load without changing `map_voxelized.pcd` or
any computed odometry.

### Skip or rerun analysis

`--no-analyze` stops after producing the result bag and runner artifacts. Such
a directory cannot be opened by the artifact viewer until analysis has created
`summary.json`, the PCDs, and trajectory CSVs. To analyze it later:

```bash
python3 scripts/offline/analyze_multilidar_run.py analyze \
  results/multilidar/long3/baseline/rosbag \
  --output-dir results/multilidar/long3/baseline \
  --label baseline \
  --voxel-size 0.20 \
  --preview-max-points 500000 \
  --plane-random-seed 7
```

The analyzer also exposes lower-level thresholds, plane-neighborhood settings,
PCD format, and storage plugin through `--help`. Keep them identical across
runs used in one comparison.

## Visualize a completed result

The viewer validates artifact paths, point counts, hashes, PCD structure, and
map/trajectory frames before it opens RViz. It starts and cleans up all required
processes, so each view needs only one terminal. Run the GUI in the graphical
devcontainer/desktop environment with a built workspace overlay and a valid
`DISPLAY` or `WAYLAND_DISPLAY`.

The viewer inherits `ROS_DOMAIN_ID` and uses absolute ROS topic names. Select an
unused domain so a live robot or another replay cannot publish into the same
RViz session. The examples below use domain 78; any unused valid domain is fine.

### Static final-map view

Use static mode for the clearest side-by-side evaluation of final map shape and
trajectory:

```bash
ROS_DOMAIN_ID=78 bash scripts/offline/visualize_multilidar_run.sh \
  results/multilidar/long3/baseline

ROS_DOMAIN_ID=78 bash scripts/offline/visualize_multilidar_run.sh \
  results/multilidar/long3/fused-matched
```

Stop the first RViz window before starting the second command. Static mode
publishes the frozen preview PCD as `/offline/map` and the frozen
matching-frame trajectory as `/offline/path`. Both use reliable,
transient-local QoS, so RViz receives them even if its subscriptions start
after the one-shot publication. No bag, FAST-LIO node, or fusion node runs in
this mode.

This mode is best for final-map screenshots, inspecting double walls or warped
surfaces, and comparing the completed paths without animation.

### Dynamic result replay

Use dynamic mode to inspect when the trajectory begins to jump or the
registered cloud becomes inconsistent:

```bash
ROS_DOMAIN_ID=78 bash scripts/offline/visualize_multilidar_run.sh \
  results/multilidar/long3/baseline \
  --dynamic --rate 1.0

ROS_DOMAIN_ID=78 bash scripts/offline/visualize_multilidar_run.sh \
  results/multilidar/long3/fused-matched \
  --dynamic --rate 1.0
```

Dynamic mode keeps the frozen `/offline/map` and `/offline/path` displays and
additionally replays only `/cloud_registered` and `/Odometry` from the result
bag. It never reruns FAST-LIO or fusion. Consequently, changing the viewer's
`--rate` changes animation speed and GUI load but cannot change the saved
odometry.

Use 1.0x when recording a real-time video, or a lower viewer rate when a dense
cloud overwhelms the display. Use the static view for final-map captures: a
dynamic video shows temporal failure onset, whereas a static map makes global
consistency easier to compare.

`--no-rviz` keeps the ROS publishers running without a GUI. Static mode then
waits until Ctrl-C, while dynamic mode waits for result playback to finish; it
is not a terminating artifact check. For a one-shot headless check, use:

```bash
python3 scripts/offline/publish_multilidar_artifacts.py RUN_DIR --validate-only
```

## Compare runs fairly

Use the same source bag and these same runner settings for all profiles:

- `--start-offset` and `--duration`;
- `--rate`;
- map-analysis options and analyzer thresholds;
- FAST-LIO tuning except for the required fused input topic/scan-line change;
- built executables, fusion transform, and host load conditions.

Prefer a complete 1.0x playback for the final comparison. `--duration` uses an
approximate wall timer and is intended for smoke tests. A slower source
playback can help diagnose compute starvation, but it is a different experiment
because callback scheduling, queueing, and measured CPU behavior change. Never
compare one profile at 0.5x with another at 1.0x.

The comparison command requires successful completed manifests and enforces
the source bag path/hash, selected source topics, offset, duration, rate,
primary trajectory, jump thresholds, voxel size, PCD format, local-plane
radius, and local-plane minimum point count as invariants. It does not enforce
the preview cap, voxel chunk size, plane sample cap, or plane random seed; check
those analysis parameters yourself before comparing plane metrics:

```bash
python3 scripts/offline/analyze_multilidar_run.py compare \
  results/multilidar/long3/baseline \
  results/multilidar/long3/fused-matched \
  --labels baseline fused-matched \
  --output results/multilidar/long3/comparison_summary.json
```

It writes both `comparison_summary.json` and `comparison_summary.csv`. Put the
MID-only run first: it is treated as the reference trajectory, but it is not
treated as truth.

### Repeatability checks

One replay can be affected by thread scheduling or transient host load. Run at
least two full repetitions per profile, preferably three, with identical
options and distinct output directories. Do not run repetitions concurrently.
For example, repeat the two runner commands with outputs such as
`baseline-r1`, `baseline-r2`, `fused-matched-r1`, and `fused-matched-r2`, then
compare all four with the first baseline as the common reference:

```bash
python3 scripts/offline/analyze_multilidar_run.py compare \
  results/multilidar/long3/baseline-r1 \
  results/multilidar/long3/baseline-r2 \
  results/multilidar/long3/fused-matched-r1 \
  results/multilidar/long3/fused-matched-r2 \
  --labels baseline-r1 baseline-r2 fused-matched-r1 fused-matched-r2 \
  --output results/multilidar/long3/repeatability_summary.json
```

Compare failure time, gap/jump counts, maximum step, path length, map bounds,
plane metrics, CPU, and RSS across repetitions. The reference-divergence fields
measure positional disagreement with `baseline-r1`; they remain behavior
differences, not accuracy errors.

Analyzer determinism is a separate check from runtime repeatability. Reanalyzing
the same immutable result bag with the same options should reproduce the PCD
and trajectory hashes. Do not compare the hash of the whole `summary.json`,
because it contains a generation timestamp. Compare its `artifact_hashes` and
`analysis_parameters_sha256`, or run `sha256sum` on the PCD/CSV artifacts.

## Read fusion diagnostics

Fused result bags include `/fastlio_go2w_fusion/diagnostics`. The analyzer
aggregates numeric samples into count, mean, median, p95, maximum, and final
value under `summary.json`'s `diagnostics.numeric_values`; it does not retain
the raw sample series. Final string values are also available under
`diagnostics.last_values`. Keys have the form `TOPIC/STATUS/COUNTER`, so the
counter names below appear as key suffixes rather than direct fields:

- throughput: `mid_messages_received`, `hesai_messages_received`,
  `hesai_messages_accepted`, and `fused_messages_published`;
- source integrity: `mid_nonmonotonic_drops`, `mid_point_count_mismatches`,
  `hesai_nonmonotonic_drops`, `hesai_parser_errors`, `hesai_partial_clouds`, and
  `hesai_invalid_points`;
- fusion coverage: `mid_points_output`, `hesai_points_output`,
  `hesai_points_stale`, and `hesai_points_filtered`;
- buffering/fallback: `mid_only_fallbacks`, `pending_queue_overflows`,
  `idle_flush_frames`, `pending_mid_frames`, and `buffered_hesai_points`.

`last_processing_time_ms` records the most recent fusion callback duration.
Most named totals are cumulative counters; `pending_mid_frames` and
`buffered_hesai_points` are instantaneous gauges. Check both the final value and
the distribution captured by the analyzer.

For a healthy fused run, published-frame count should track accepted MID
windows, both source point-output totals should be nonzero, and parser errors,
unexpected non-monotonic drops, queue overflows, and MID-only fallbacks should
normally remain zero. Partial input clouds, stale/filtered points, or points
remaining buffered at the final boundary can be legitimate; interpret them
against timestamps and message counts rather than assuming every nonzero value
is a failure.

Use `--debug-cloud` only for a focused fusion/extrinsic investigation. It adds
the source-labelled `/livox/lidar_fused_debug` topic to publication and result
recording, which changes processing and I/O load. Therefore, do not compare a
debug-enabled fused run's CPU numbers with a baseline run that did not record
the extra cloud.

The standard offline result viewer deliberately does not replay or render the
debug topic; dynamic mode remains limited to `/cloud_registered` and
`/Odometry`. Debug-cloud inspection is a separate custom RViz task: replay
`/livox/lidar_fused_debug` from the result bag, add a PointCloud2 display for
that topic, and use the cloud header frame as RViz's fixed frame.
Alternatively, run interactive fused replay with `--debug-cloud` and add that
display manually.
Neither method is part of the one-terminal result viewer.

## What the offline map represents

`map_voxelized.pcd` is the complete voxelized accumulation of every finite
point recorded on `/cloud_registered`. `map_preview.pcd` is a deterministic,
bounded subset of those sorted voxel centroids. The artifact publisher prefers
the preview for `/offline/map` and falls back to the complete PCD only when the
summary has no preview path. Use the full PCD for complete offline analysis and
the preview topic for responsive RViz inspection.

It is **not** FAST-LIO's `/Laser_map`, and it is not a serialization of the
internal incremental k-d tree (iKD-Tree). `/Laser_map` is a cumulative
visualization topic whose repeated publication is deliberately disabled in
headless measurement.
The internal map also has update/deletion state that cannot be reconstructed
exactly from registered output scans. Accordingly, do not expect point-for-point
identity with an interactive main-branch `/Laser_map` display. The accumulated
registered-cloud map is the comparable artifact used by this workflow.

## Troubleshooting

### RViz opens but the fused map is blank

Validate the analyzed artifacts directly:

```bash
python3 scripts/offline/publish_multilidar_artifacts.py \
  results/multilidar/long3/fused-matched --validate-only
```

The command reports the selected preview PCD, point count, trajectory, pose
count, and frame. The visualizer runs the same validation automatically. A
missing `summary.json` usually means the run used `--no-analyze`; run the
analyzer before opening it. A hash, count, PCD, or frame mismatch means the run
directory is incomplete or an artifact was modified and should not be silently
ignored.

In RViz, inspect `/offline/map` and `/offline/path`, not `/Laser_map`. The
wrapper prints the validated artifact frame and sets it as RViz's fixed frame;
choose **Reset View** if the geometry is far outside the current camera. Dynamic
registered scans appear on `/cloud_registered` only after `--dynamic` starts
result playback.

### Dynamic mode has no animation

Confirm that `RUN_DIR/rosbag/metadata.yaml` exists and that the bag contains
`/cloud_registered` and `/Odometry`:

```bash
ros2 bag info results/multilidar/long3/fused-matched/rosbag
```

Dynamic mode does not read the original sensor bag and does not recreate
missing output. Re-run the headless experiment if the result bag lacks those
topics.

### The runner rejects a custom config

Check the profile's input topic and scan-line count, then ensure `map_en`,
`path_en`, and `scan_bodyframe_pub_en` are false, `scan_publish_en` is true, and
`dense_publish_en` is false. The rejection is intentional: changing those
outputs changes load and breaks the result-artifact contract.

### Comparison reports an invariant mismatch

Read the named field in the error. It usually indicates a different source
bag, playback rate/interval, map voxel size, PCD format, or analysis threshold.
Re-run or re-analyze with identical values instead of overriding the check.

### A fused map shows double walls or separated ground surfaces

Treat this as a possible extrinsic or timing problem, not an odometry
improvement. Make a focused fused run with `--debug-cloud`, inspect the
source-labelled points using the separate custom RViz procedure above, and
check the fusion counters for stale, filtered, partial, or dropped XT16 data.
Keep any revised transform and time offset fixed across every profile being
compared.

### The workspace is reported as stale or missing

Rebuild the Humble workspace in the project container. The runner intentionally
checks that the installed launch file matches the source and that the required
FAST-LIO, odometry-adapter, and fusion executables exist before starting a long
experiment.
