# XT16 point-filter comparison — 2026-07-29

## Verdict

The XT16 offline profile must not use FAST-LIO's index-stride point filter on
the timestamp-sorted adapter output. `point_filter_num: 4` aliases with the
XT16 firing/ring order and can reduce a nominal 16-ring scan to a strongly
biased effective subset. On the latest corrected-driver bag, disabling that
stride removed the catastrophic translation divergence observed in the first
700 seconds.

The selected accuracy-profile setting is:

- `point_filter_num: 1`
- `filter_size_surf: 0.2`
- `filter_size_map: 0.2`

This comparison has no ground truth. It establishes relative trajectory
continuity and internal map diagnostics, not absolute pose accuracy.

## Input and method

- Bag: `experiment_fixed-test_20260728_080342`
- Topics: `/points_raw` and `/go2w/imu`
- Interval: first 700 seconds
- Playback: rate 1.0, LiDAR time offset 0.0 seconds
- Branch: `feat/xt16-go2w-imu`
- Commit under test: `5929143`
- All three completed runs received and converted 7,002 XT16 frames with zero
  adapter warnings or drops.

The compared settings were:

1. `pf4-index`: `point_filter_num: 4`, `filter_size_surf: 0.2`
2. `pf1-all`: `point_filter_num: 1`, `filter_size_surf: 0.2`
3. `pf1-voxel040`: `point_filter_num: 1`, `filter_size_surf: 0.4`

The first attempt to start condition 3 stopped before playback because the
runner's ROS parameter dump transiently returned `Node not found`. Its failed
artifact was preserved. The `v2` run used a fresh output and ROS domain and
completed normally.

## Results

| Metric | pf4-index | pf1-all | pf1-voxel040 |
| --- | ---: | ---: | ---: |
| Manifest | completed/0 | completed/0 | completed/0 |
| Finite trajectory samples | 6,966 | 6,945 | 6,900 |
| Gaps over 0.2 s | 15 | 23 | 45 |
| Translation jumps over 1 m | 3,810 | 0 | 0 |
| Maximum translation step | 177.252 m | 0.366 m | 0.455 m |
| Orientation steps over 15 degrees | 69 | 1 | 6 |
| Maximum orientation step | 27.449 deg | 23.280 deg | 20.674 deg |
| Path length | 225,238.424 m | 471.321 m | 471.602 m |
| Terminal displacement | 221,743.554 m | 62.863 m | 64.213 m |
| `No Effective Points` | 409 | 0 | 0 |
| PCL voxel-index overflow | 17 | 0 | 0 |
| Map points | 9,227,602 | 11,942,141 | 5,376,986 |
| Map extent | 15,415 x 45,061 x 216,673 m | 295 x 190 x 58 m | 294 x 195 x 59 m |
| Local plane thickness p95 | 0.1785 m | 0.1986 m | 0.2061 m |
| FAST-LIO average CPU cores | 0.218 | 0.272 | 0.226 |
| FAST-LIO peak RSS | 303 MB | 408 MB | 348 MB |

The `pf4-index` plane statistic is not a quality advantage: it was computed on
an already divergent, enormous map and is therefore not comparable as a valid
localization result.

Every orientation-threshold crossing in the two non-divergent runs coincided
with an approximately 0.2-second output interval (one was 0.298 seconds).
They may include real accumulated rotation across a missing output sample, so
they are retained as continuity failures but are not treated as evidence of
the catastrophic estimator jump seen with `pf4-index`.

## Ring-alias evidence

Representative raw scans contained about 59,000 valid points across all 16
rings. At approximately 550 seconds, applying the same four-point index stride
retained 14,820 points, but 97.5% came from even rings and about 90% came from
rings 2, 6, 10, and 14; ring 15 contributed no points. This is caused by the
combination of timestamp sorting and deterministic modulo-index filtering.

## Artifacts

Base directory:

`/mnt/data1/experimental_data/fastlio-go2w/results/fastlio/experiment_fixed-test_20260728_080342/xt16-filter-ab/`

- Baseline: `pf4-index-700s/`
- Selected candidate: `pf1-all-700s/`
- Spatial-voxel candidate: `pf1-voxel040-700s-v2/`
- Preserved pre-playback startup failure: `pf1-voxel040-700s/`

Each completed directory contains the frozen config, manifest, result bag,
trajectory CSVs, map artifacts, adapter diagnostics, and resource metrics.

## Remaining verification

Run the complete corrected-driver bag with the selected `point_filter_num: 1`
profile. The 700-second result demonstrates a causal improvement over the
known failure interval, but it does not establish full-duration stability,
absolute accuracy, live-sensor readiness, or Jetson performance.
