# Issue #7: offline MID-360 + Pandar XT16 experiment

## Status and question

This branch adds an offline-only front end that aligns recorded Pandar XT16
points to the MID-360 acquisition window, transforms them into `livox_frame`,
and emits one Livox-compatible cloud for FAST-LIO. It does not add a robot
sensor launch path and does not use `/go2w/imu`; the IMU input remains
`/livox/imu`.

The experiment asks whether adding the XT16 improves odometry continuity and
map consistency, and what it costs in CPU and memory. The bags have no
ground-truth trajectory, so the MID-only result is a comparison reference, not
truth. Path difference from that reference must not be reported as ATE or RPE.

The controlled full replay and analysis completed on 2026-07-16 JST. The
density-matched fused profile remained continuous for the full interval; the
high-density profile failed late, and the MID-only baseline diverged severely.
Because the bag has no ground truth, this is a stability result rather than an
absolute-accuracy result.

## Inputs, frames, and profiles

The primary dataset is:

```text
/mnt/data1/experimental_data/go2w-experiment-recorder/bags/experiment_long3_20260714_014823
```

Its metadata describes a 1,251.48 s sqlite3 bag with 11,954 MID clouds, 11,962
XT16 clouds, and 239,091 Livox IMU messages. The experiment runner replays only
`/livox/lidar`, `/livox/imu`, and `/points_raw` and uses the common sensor
interval rather than unrelated topics that extend the bag duration.

The approximate user-supplied `hesai_lidar` pose in `base_link` is a 90 degree
Z rotation and translation `[0.1634, 0, 0.116]` m. Combining the sensor poses
used by the related GO2-W configurations gives this initial transform from
`hesai_lidar` into `livox_frame`:

```yaml
translation: [-0.018602675, 0.0, -0.095450199]
rotation_xyzw: [-0.112310121, -0.112310121, 0.698130673, 0.698130673]
```

This is an initial composed extrinsic, not a dedicated dual-LiDAR calibration.
Before interpreting map metrics, inspect the source-labelled debug cloud for
double walls, displaced ground planes, and range-dependent separation. Record
any changed transform in the run manifest and repeat every profile with the
same transform.

The launch file fixes these three profiles:

| Profile | Input | MID selection | XT16 selection | Observed effective points/window |
| --- | --- | ---: | ---: | ---: |
| `baseline` | MID-360 only | Existing FAST-LIO stride 3 | none | about 5,245 |
| `fused-high` | fused cloud | every 3rd valid MID point | every 3rd XT firing group | 24,601 |
| `fused-matched` | fused cloud | every 6th valid MID point | every 22nd XT firing group | 5,256 |

The fusion counters give 5,245 MID + 19,356 XT points per high-density
window, and 2,626 MID + 2,630 XT points per matched window. The baseline uses
the same MID sampling phase as `fused-high`, so `fused-matched` is within 0.21%
of its approximately 5,245-point effective input density.

XT16 filtering is by firing-time group, retaining all 16 rings in each selected
group. MID lines stay at 0–3 and XT16 rings map to lines 4–19. The fused FAST-LIO
configuration uses `lidar_type: 1`, `scan_line: 20`, `point_filter_num: 1`,
`/livox/lidar_fused`, and `/livox/imu`.

Each fused output uses the MID `timebase` and acquisition end. XT points are
selected from the buffered absolute timestamps only when they fall within that
window, then consumed once. A zero-second XT time offset and 0.5 m source-frame
minimum range are the defaults. Pending MID frames fall back to MID-only at a
queue depth of 32.

### Known source-data anomaly

An initial timestamp inspection found seven XT16 cloud timestamp regressions of
approximately 7.2 s in `experiment_long3_20260714_014823`. The fusion front end
rejects those non-monotonic clouds. Both completed fused runs report exactly seven non-monotonic XT drops,
14 short-span partial clouds, no parser errors, no MID-only fallbacks, and no
queue overflows.

## Reproducible workflow

Build the workspace in the repository's ROS 2 Humble container, then run each
profile against the same source interval. The runner owns paused startup,
endpoint readiness, playback, result recording, resource sampling, and
controlled shutdown:

```bash
# From the repository root, after building/sourcing the Humble workspace.
BAG=/mnt/data1/experimental_data/go2w-experiment-recorder/bags/experiment_long3_20260714_014823

bash scripts/offline/run_multilidar_experiment.sh \
  "$BAG" --profile baseline \
  --output results/multilidar/long3/baseline

bash scripts/offline/run_multilidar_experiment.sh \
  "$BAG" --profile fused-high \
  --output results/multilidar/long3/fused-high

bash scripts/offline/run_multilidar_experiment.sh \
  "$BAG" --profile fused-matched \
  --output results/multilidar/long3/fused-matched
```

Use the runner's start-offset, duration, and rate options for a short smoke test
before the complete rate-1.0 replay. Enable the fusion debug cloud only for the
visual extrinsic check because it increases recording and processing load.

After each replay, analyze the recorded result bag in a sourced Humble shell.
Each run stores its result bag in the `rosbag/` directory created inside that
run directory:

```bash
python3 scripts/offline/analyze_multilidar_run.py analyze \
  results/multilidar/long3/baseline/rosbag \
  --output-dir results/multilidar/long3/baseline --label baseline

python3 scripts/offline/analyze_multilidar_run.py analyze \
  results/multilidar/long3/fused-high/rosbag \
  --output-dir results/multilidar/long3/fused-high --label fused-high

python3 scripts/offline/analyze_multilidar_run.py analyze \
  results/multilidar/long3/fused-matched/rosbag \
  --output-dir results/multilidar/long3/fused-matched --label fused-matched

python3 scripts/offline/analyze_multilidar_run.py compare \
  results/multilidar/long3/baseline \
  results/multilidar/long3/fused-high \
  results/multilidar/long3/fused-matched \
  --labels baseline fused-high fused-matched \
  --output results/multilidar/long3/comparison_summary.json
```

The analyzer prefers `/odom` (the `base_link` adapter output) and falls back to
raw FAST-LIO `/Odometry`. Its defaults are a 0.20 m output voxel, a 0.60 m local
covariance neighborhood, a 0.20 s odometry-gap threshold, a 1.0 m translation
jump threshold, and a 15 degree orientation-jump threshold. Keep these fixed
for the three-way comparison.

## Metrics and interpretation

`trajectory.csv` contains stamped position and quaternion samples. The summary
reports sample rate, non-finite poses, non-monotonic intervals, gaps, translation
and orientation steps/jumps, path length, and terminal displacement. The
comparison interpolates the MID-only positions at fused-run timestamps and
reports translation difference distributions. That divergence detects changed
behavior; it does not identify which trajectory is more accurate.

`map_voxelized.pcd` contains one centroid and raw observation count per occupied
voxel. Map coverage is reported as occupied 3D volume, occupied XY area, and
bounds. Local plane thickness is `sqrt(lambda_min)` and planarity is
`(lambda_mid - lambda_min) / lambda_max` from covariance neighborhoods. Repeated
scan observations and the approximate voxel-neighborhood calculation make
these comparative consistency indicators, not calibrated surface-error
measurements. Review the final PCDs side by side in RViz as well as comparing
their distributions.

`resource_metrics.csv` is sampled from `/proc`. The analyzer derives average
CPU cores as CPU-time delta divided by observed wall time for each named
process and reports peak RSS. Compare the FAST-LIO, fusion, player, and recorder
processes separately; also compare wall time and real-time factor from the run
manifest/runner summary.

### Completed result table

The full selected sensor stream was replayed at 1.0x for all three profiles on
2026-07-16 JST. Every manifest finished in `completed` state with exit code 0.
Each result contains 11,951 `/odom` samples over 1,195.000 s; the strict
comparison check confirms identical source-bag metadata, selected topics,
start/duration/rate, primary trajectory topic, jump thresholds, voxel size,
and local-plane settings.

| Metric | MID baseline | Fused high | Fused matched |
| --- | ---: | ---: | ---: |
| Odometry samples / maximum interval | 11,951 / 0.104 s | 11,951 / 0.104 s | 11,951 / 0.104 s |
| Non-finite / non-monotonic / gaps >0.20 s | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 |
| Translation >1 m / orientation >15 deg jumps | 5,182 / 237 | 424 / 0 | 0 / 0 |
| First translation / orientation jump | 676.9 s / 859.0 s | 1,152.7 s / none | none / none |
| Maximum translation / orientation step | 331.944 m / 83.512 deg | 11.996 m / 14.551 deg | 0.356 m / 13.735 deg |
| Path length / terminal displacement | 654,959.3 m / 642,552.4 m | 4,724.3 m / 3,258.0 m | 1,118.9 m / 189.5 m |
| Difference from MID reference, median / terminal | reference | 193.061 m / 640,132.4 m | 159.695 m / 642,560.5 m |
| Map input points / occupied voxels | 8,273,765 / 5,556,868 | 9,061,179 / 1,258,679 | 5,951,829 / 820,357 |
| Occupied XY area | 160,003.4 m² | 10,243.8 m² | 3,403.5 m² |
| Map bounds extent x/y/z | 186,842 / 101,408 / 606,597 m | 2,877 / 1,015 / 1,667 m | 358 / 141 / 137 m |
| Plane thickness median / p95 | 0.099 / 0.185 m | 0.145 / 0.221 m | 0.150 / 0.229 m |
| Planarity median / valid neighborhoods | 0.596 / 328 | 0.418 / 3,699 | 0.436 / 4,670 |
| FAST-LIO average CPU cores / peak RSS | 0.147 / 235.3 MiB | 0.159 / 267.9 MiB | 0.124 / 211.5 MiB |
| Fusion average CPU cores / peak RSS | n/a | 0.168 / 61.0 MiB | 0.132 / 59.0 MiB |
| Fusion source/failure counters | n/a | 7 non-monotonic, 14 partial, 0 parser/fallback/overflow | 7 non-monotonic, 14 partial, 0 parser/fallback/overflow |

The reference-difference row is not an error metric. The MID baseline itself
has catastrophic divergence, so the very large fused-to-reference terminal
differences cannot identify which trajectory is accurate.

### Findings

- `fused-matched` is the only profile that remains continuous for the complete
  1,195 s output interval. It has no translation or orientation jumps at the
  configured thresholds and a maximum 0.356 m translation step.
- `fused-high` delays the first >1 m translation jump until 1,152.7 s, but then
  diverges over the final approximately 42 s. The high-density configuration is therefore not robust through the end of
  this recording; one replay does not establish why it fails.
- The MID-only baseline diverges catastrophically: it records 5,182 translation
  jumps, reaches a 331.944 m single-step maximum, and ends 642.6 km from its
  initial position. This makes it unsuitable as an accuracy reference after
  divergence.
- There is no evidence of sustained CPU saturation in these runs. FAST-LIO plus fusion averages
  about 0.327 CPU cores for `fused-high` and 0.256 for `fused-matched`.
  `fused-matched` also uses less FAST-LIO CPU and peak RSS than the baseline in
  this replay.
- The bounded `fused-matched` map is the only full map suitable for visual
  consistency inspection. The apparently thinner baseline planes come from
  only 328 valid neighborhoods in a map stretched across hundreds of
  kilometers; the plane statistics cannot be ranked as if all three maps were
  geometrically valid.

These results support using density-matched fusion as the next experimental
configuration. They demonstrate improved continuity and boundedness on this
bag, not improved absolute accuracy. There is no ground truth, and the composed
Hesai-to-Livox extrinsic remains an initial estimate rather than a calibrated
transform. A source-labelled debug-cloud review for double surfaces is still
required before attributing the change specifically to the extra field of view.

### Observed source behavior and limitations

Both fused runs received 11,962 XT16 clouds, accepted 11,955, and rejected the
same seven timestamp regressions. Each classified 14 short-span clouds as
partial and reported zero parser errors, MID-only fallbacks, queue overflows,
or idle flushes. The 42,338 XT points still buffered after the final MID frame
are later than the final MID acquisition window and are intentionally not
published on their own.

All three full runs show noisy intentional teardown in `launch.log`: FAST-LIO
exits with `-11`, while Python nodes report `ExternalShutdownException` after
the result stream has drained and the runner sends SIGINT. This happened after
all 11,951 output frames and is separate from an in-replay processing failure.
The hardened runner now monitors required nodes, the recorder, and the resource
sampler during replay and before drain, but the existing FAST-LIO shutdown bug
remains visible in the logs.

The full runs were made from the branch's uncommitted working tree. Their
manifests record the Git status, source launch hash, config hash, and source-bag
metadata hash, and the source/installed launch and configs were rechecked after
the runs and matched byte-for-byte. They do not preserve the complete untracked
source contents, so the old artifacts alone are not an independently
reconstructible software snapshot. Future runs now refuse a stale installed
launch, execute the hashed config snapshot, dump and hash live fusion
parameters, and record hashes of the actual FAST-LIO, odom-adapter, and fusion
executables.

## Artifacts and repository policy

Generated artifacts live below `results/multilidar/` and are gitignored. Each
run should retain its source-bag reference and result bag, launch and recorder
logs, manifest and configuration hashes, `resource_metrics.csv`, `resource_summary.json`,
`trajectory.csv`, `map_voxelized.pcd`, and `summary.json`. The three-run parent
directory also contains `comparison_summary.json` and
`comparison_summary.csv`.

Only this methodology, completed result tables/conclusions, and small comparison
images belong in Git. Do not commit rosbag files, full PCD maps, raw logs, or
large generated CSV/JSON files.
