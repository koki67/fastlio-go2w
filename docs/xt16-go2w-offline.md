# Offline FAST-LIO with Pandar XT16 and Go2W IMU

This path runs FAST-LIO from an existing ROS 2 bag containing only Pandar XT16
`/points_raw` and onboard Go2W `/go2w/imu`. It is offline-only: it neither
starts nor adds a live Hesai driver path, and it never rewrites the source bag.
The existing MID-360 launch and runner defaults remain unchanged.

## Sensor and time contract

The recorded XT16 PointCloud2 schema is:

- `x`, `y`, `z`, `intensity`: `FLOAT32`
- `timestamp`: `FLOAT64` absolute seconds
- `ring`: `UINT16`

FAST-LIO's Velodyne-style input requires `x/y/z/intensity/time/ring`, with
`time` relative to the cloud header. `hesai_pointcloud_adapter` validates the
source schema, endian representation, row padding, field bounds, finite point
times, finite point values, and ring range. It stable-sorts retained points by
absolute time, writes `time = timestamp - raw_header_stamp` as relative
seconds, and shifts only the output header by `lidar_time_offset_sec`.
The source subscription matches the recorded Reliable QoS and uses a depth-100
queue; this is required to keep the large-cloud frame count stable across the
five repeated sweep runs.

Whole frames are rejected when the header regresses, a point timestamp is not
finite, or any point timestamp differs from the raw header by more than 0.2 s.
Invalid XYZ/intensity/ring values reject individual points; a frame with fewer
than 10 retained points is rejected. The latter counters and the latest scan
width are published on `/fastlio_go2w_hesai/diagnostics` and frozen into
`adapter_diagnostics.json` and `manifest.json`.

FAST-LIO's `common.time_sync_en` remains false and
`common.time_offset_lidar_to_imu` remains 0.0. This prevents the adapter header
shift from being applied a second time. Go2W IMU timestamps are never changed.

## Calibration and FAST-LIO profile

`config/sensor/go2w_xt16_calibration.yaml` is the workspace reference. Values
are copied from the D-LIO Go2W mounting configuration:

- base_link to imu: translation `[-0.02557, 0, 0.04232]`, identity rotation
- base_link to hesai_lidar: translation `[0.1634, 0, 0.116]`, Z +90 degrees
- hesai_lidar to imu for FAST-LIO: translation `[0.18897, 0, 0.07368]`, Z +90 degrees

`xt16_go2w_accuracy_offline.yaml` uses 16 lines, 10 Hz, relative seconds,
`point_filter_num: 4`, 0.5 m blind range, 360 degree FOV, 100 m detection
range, and fixed extrinsics. IMU noise values are initial references copied
from the D-LIO sensor file, not measured FAST-LIO tuning values.

## Run one bag

Build the Humble workspace, source its install overlay, then run:

~~~bash
BAG=/mnt/go2w-experiment-recorder/bags/experiment_stair2_20260713_115313
OUT="${FASTLIO_RESULTS_ROOT:-$PWD/results}/fastlio/stair2/xt16-zero"

bash scripts/offline/run_fastlio_offline.sh \
  "$BAG" \
  --sensor xt16 \
  --lidar-time-offset-sec 0.0 \
  --start-offset 20 \
  --rate 1.0 \
  --output "$OUT"
~~~

The result contract is the same as MID-360 and additionally includes the
selected calibration and adapter diagnostics. A completed run has finite
trajectory CSVs, nonempty voxelized/preview maps, result rosbag, resource
metrics, logs, hashes, and `state: completed` / `exit_code: 0` in manifest.json.

## Reproduce fixed-offset selection

The only supported synchronization policy is a fixed, explicitly supplied
header offset. Runtime estimation is intentionally absent. Run all five
candidates against the same stair2 interval:

~~~bash
bash scripts/offline/sweep_xt16_time_offset.sh \
  /mnt/go2w-experiment-recorder/bags/experiment_stair2_20260713_115313 \
  --start-offset 20 \
  --rate 1.0 \
  --output "${FASTLIO_RESULTS_ROOT:-$PWD/results}/fastlio/stair2/xt16-offset-sweep"
~~~

The sweep retains each run and writes `offset_comparison.json` plus CSV. It
disqualifies incomplete runs, increased non-finite samples, more than 1%
finite-trajectory or map-frame coverage loss, and increased gap or jump counts.
A nonzero candidate is adopted only if it also improves local-plane thickness
p95 by strictly more than 5% without worsening median thickness or median
planarity. Ties use p95 thickness, absolute offset, then numeric offset. If no
nonzero candidate passes, 0 ms remains the default.

The valid 2026-07-22 stair2 sweep retained 0 ms: nonzero p95 improvements ranged
from 0.48% to 2.35%, below the strict 5% adoption gate. The selected value is
therefore still the default in calibration and launch configuration. The 0 ms
reference produced finite output but still contained one gap over 0.2 s and two
orientation steps over 15 degrees, so it did not pass the trajectory-continuity
acceptance criteria.

## Interpretation and remaining verification

Existing recordings contain a known XT16 driver defect in which seven long3
frames have timing displaced by roughly 7.2 seconds. The completed long3
validation observed all seven as `dropped_header_regression`; the adapter
rejected each complete frame and resumed at the next normal frame. This is not
counted as an adapter or FAST-LIO execution failure. Partial use of such frames
is prohibited because it would make the acquisition window ambiguous.

The complete 2026-07-22 validation is recorded in
[`validation/2026-07-22-xt16-go2w-imu.md`](validation/2026-07-22-xt16-go2w-imu.md).
The offline pipeline completed the original long3 bag and generated all
contract artifacts, but the estimated trajectory diverged late in the run and
failed the jump limits. The correct status is therefore **runnable, but the
trajectory-quality acceptance criteria failed**; the result is not evidence of
usable localization quality.

No ground truth exists in these bags. Local-plane thickness, map coverage, and
trajectory continuity are internal consistency diagnostics, not ATE or RPE.
Corrected-driver recordings, absolute accuracy, Jetson execution, live sensor
operation, and physical-robot operation remain **UNVERIFIED** and must not be
inferred from an offline completed run.
