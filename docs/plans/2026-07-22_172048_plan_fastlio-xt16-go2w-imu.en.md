# Plan

- Created: 2026-07-22T17:20:48+09:00
- Snapshot: 2026-07-22T17:20:48+09:00
- Status: final
- Language: en
- Translated from: `2026-07-22_172048_plan_fastlio-xt16-go2w-imu.md`
- Session: unavailable
- Branch: `main` (`a8180308f7a15dcb7412eb5b458b0792db6d6220`) → `feat/xt16-go2w-imu`
- Workspace: `/home/user/ws/fastlio-go2w`
- Scope: Run FAST-LIO offline using only the Pandar XT16 `/points_raw` and onboard Go2W IMU `/go2w/imu` from existing recordings, and preserve artifacts and a viability determination. Physical-robot and live-sensor startup are out of scope.

## Context and current state

- The current `main` supports only MID-360. The existing `feat/offline-dual-lidar-fusion` is a MID-360 + XT16 experiment and will not be merged into this branch.
- The XT16 point cloud contains `x/y/z/intensity: FLOAT32`, absolute-seconds `timestamp: FLOAT64`, and `ring: UINT16`. FAST-LIO's Velodyne input requires relative-seconds `time: FLOAT32`, so a dedicated conversion is necessary.
- Use the following transforms from `dlio-go2w`.
  - `base_link→imu`: translation `[-0.02557, 0, 0.04232]`, no rotation
  - `base_link→hesai_lidar`: translation `[0.1634, 0, 0.116]`, +90° about the Z axis
  - `hesai_lidar→imu` for FAST-LIO: translation `[0.18897, 0, 0.07368]`, rotation matrix `[0,-1,0, 1,0,0, 0,0,1]`
- Investigation of the real data indicates a high likelihood that execution is viable.
  - `stair2`: 940 XT16 frames, 46,990 IMU messages, normal point-time span about 100 ms, nearest-IMU difference median 0.577 ms / p95 1.143 ms
  - `long3`: 11,962 XT16 frames, 597,701 IMU messages, nearest-IMU difference median 0.486 ms / p95 0.991 ms
- Point clouds offset by approximately 7.2 seconds are a known XT16 driver defect. Detect and discard them so processing can continue on the current data, but do not treat them as a FAST-LIO implementation failure. Separate revalidation after acquiring new data with the corrected driver is mandatory.

## Decisions and constraints

- Use a fixed-offset time-synchronization method. Do not implement runtime automatic estimation.
- Define converted LiDAR time as `raw_lidar_time + lidar_time_offset_sec` and do not change Go2W IMU time. Disable/set to zero FAST-LIO's built-in `time_sync_en` and `time_offset_lidar_to_imu` to prevent double correction.
- Candidates are `-10/-5/0/+5/+10 ms`. Compare them over the same interval and adopt a nonzero candidate only if it improves surface-thickness p95 by more than 5% without worsening continuity. Otherwise retain 0 ms.
- Play the source bag directly without modifying it. Drop anomalous frames in their entirety in the conversion node and record them in diagnostic counters.
- Do not change or integrate the existing MID-360 default behavior, existing artifact format, `main`, or the experimental dual-LiDAR branch.
- Keep new recordings, the physical robot, Jetson, and absolute accuracy UNVERIFIED. The current bags have no ground truth.
- After implementation and verification, commit to the local branch. Do not push or merge until requested.

## Final plan

1. Before implementation, save this final version with identical content to `~/.codex/memories/rollout_plans/2026-07-22_172048_plan_fastlio-xt16-go2w-imu.md` and `docs/plans/2026-07-22_172048_plan_fastlio-xt16-go2w-imu.md`, then reread it in a fresh context. Recheck repository state and create `feat/xt16-go2w-imu` from `main`.
2. Add `config/sensor/go2w_xt16_calibration.yaml` to centralize the D-LIO-derived base, IMU, and XT16 transforms, the composed LiDAR-to-IMU transform, topics, and calibration provenance. Add a fixed `hesai_lidar` link to the Go2W URDF and keep it consistent with the existing `imu` link.
3. Implement `hesai_pointcloud_adapter` in a new ROS 2 C++ package, `fastlio_go2w_hesai`.
   - Input `/points_raw`, output `/points_raw_fastlio`
   - Output fields are `x/y/z/intensity/time/ring`. `time` is relative seconds calculated by subtracting the original header time from each original absolute point timestamp.
   - Stable-sort points by time and shift only the header by the fixed offset.
   - Validate endianness, row padding, field types, ring range, and finite values.
   - Drop an entire frame for a regressing header, a non-finite point timestamp, or a point timestamp more than 200 ms from the header. Remove invalid XYZ, intensity, or ring values point by point, and drop the frame if too few points remain.
   - Publish received count, converted count, counts by drop reason, invalid-point count, latest scan width, and applied offset on `/fastlio_go2w_hesai/diagnostics`.
4. Add a FAST-LIO configuration for XT16 + Go2W IMU.
   - `lidar_type: 2`, `scan_line: 16`, `scan_rate: 10`, `timestamp_unit: 0`
   - `lid_topic: /points_raw_fastlio`, `imu_topic: /go2w/imu`
   - `extrinsic_est_en: false` with the fixed LiDAR-to-IMU transform above
   - Initial values: `point_filter_num: 4`, `blind: 0.5 m`, `fov_degree: 360`, `det_range: 100 m`
   - Copy the initial Go2W IMU noise values from the D-LIO calibration reference and comment that they are not empirically tuned values.
5. Make the existing launch sensor-profile-aware.
   - Expose `offline_fastlio.launch.py sensor:=mid360|xt16`, retaining `mid360` as the default.
   - Start the conversion node only for `xt16` and switch the odom adapter's IMU frame to `imu`.
   - Expose `lidar_time_offset_sec` as a launch argument.
   - Do not add an XT16 path to the live sensor driver or `sensors.launch.py`.
6. Add `--sensor xt16` and `--lidar-time-offset-sec` to the existing offline runner.
   - For XT16, play only `/points_raw` and `/go2w/imu` from the source bag.
   - Monitor the conversion node, FAST-LIO, odom adapter, and recorder as required processes.
   - Save configuration, execution parameters, calibration, executable hashes, adapter diagnostics, and selected offset in the manifest.
   - Preserve MID-360 topics, default configuration, and artifact contract.
7. Add a fixed-offset comparison script and decision report.
   - Run the portion of `stair2` after `--start-offset 20` five times under identical conditions.
   - Disqualify candidates that are incomplete, increase non-finite values, reduce sample coverage by 1% or more, or increase gap/jump counts.
   - Among passing candidates, adopt only one that reduces local-plane thickness p95 by more than 5% relative to 0 ms without worsening median thickness or planarity.
   - If multiple candidates qualify, decide by p95 thickness, absolute offset, then numeric offset. If none qualify, use 0 ms.
   - Save comparison JSON/CSV and references to each run, then reflect the selected value in the XT16 default configuration and documentation.
8. Execute tests and real-bag verification. In the final report, distinguish among "runnable," "runnable but trajectory quality unverified," and "not runnable." Expected frame rejection caused by the known driver defect does not count as an implementation failure.
9. Confirm the diff, absence of generated-artifact contamination, and no existing MID-360 regression, then commit in coherent logical groups. Keep the branch local.

## Validation

- C++ unit tests:
  - Valid XT16 schema, absolute timestamp → relative `time`, time sorting, ring preservation, and signed fixed offset
  - Big/little endian, row padding, missing/duplicate/wrongly typed fields
  - NaN/Inf, out-of-range ring, header regression, scans over 200 ms, empty/tiny clouds
  - Anomalous frames produce no output and diagnostic counters increase correctly
- Configuration tests:
  - Numerical agreement among calibration YAML, URDF, and FAST-LIO `extrinsic_R/T`
  - XT16 topics, 16 lines, and seconds unit
  - Unchanged MID-360 defaults and existing offline configuration
- Static and build checks:
  - Run `bash -n` on changed shell scripts
  - Run Python tests with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`
  - Run `colcon build` and `colcon test` for the target packages in a ROS 2 Humble container
  - Run `git diff --check`
- Real data:
  1. Run a short smoke test from 20 seconds onward in `experiment_stair2_20260713_115313`.
  2. Run the five-candidate fixed-offset sweep over the same interval.
  3. Run the entire original `experiment_long3_20260714_014823` bag at rate 1.0 with the selected value.
  4. For `long3`, confirm that 7 known anomalous frames out of 11,962 inputs are rejected with diagnostics and the remainder are processed.
  5. Save `manifest.json`, `summary.json`, `trajectory.csv`, map PCD, resource metrics, and diagnostics.
  6. After acquiring a new bag with the corrected XT16 driver, rerun the same preflight, sweep, and long-duration verification.

## Acceptance criteria

- Existing recordings alone can feed XT16 point times, without losing them, together with Go2W IMU data into FAST-LIO.
- Over the normal `stair2` interval, the runner reaches completed state and generates finite `/odom` and `/cloud_registered` output and nonempty map artifacts.
- Over the normal interval, there are no non-finite poses, gaps over 0.2 seconds, translation jumps over 1 m, or orientation jumps over 15°.
- The original `long3` bag is processed to completion without derivative cleaning, safely isolating the 7 known anomalous frames.
- Offset selection is reproducible with the same data and criteria, and the adoption reason is recorded in JSON.
- Existing MID-360 offline tests and default execution do not regress.
- Even if the current data is viable, do not claim absolute accuracy or physical-robot operational readiness; leave verification with new normal data incomplete.

## Risks / cautions

- The approximately 7.2-second anomalies in the current bags are a known driver defect and must not be used to evaluate algorithm quality.
- There is no ground truth, so differences from D-LIO or trajectory length must not be treated as ATE/RPE.
- The fixed-offset sweep's selected value is provisional for the current data. Reevaluate it with a new recording.
- Reversing the LiDAR-to-IMU transform direction or the +90° Z rotation can produce a broken map even when FAST-LIO starts, so lock them down with automated tests.
- Partially using a frame makes its acquisition window ambiguous, so process timing anomalies fail-closed at frame granularity.
