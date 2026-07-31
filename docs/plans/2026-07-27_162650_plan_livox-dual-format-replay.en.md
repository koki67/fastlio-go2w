# Plan

- Created: 2026-07-27T16:26:50+09:00
- Snapshot: 2026-07-27T16:26:50+09:00
- Status: final
- Language: en
- Translated from: 2026-07-27_162650_plan_livox-dual-format-replay.md
- Session: 019fa23e-0621-7571-93e5-b63d55b77709
- Branch: main
- Workspace: /home/user/ws/fastlio-go2w
- Scope: Make MID-360 rosbag replay select both legacy `livox_ros_driver2/msg/CustomMsg` and new `sensor_msgs/msg/PointCloud2` inputs automatically and backward-compatibly. Apply the same input contract, diagnostics, and validation to interactive RViz replay and the headless offline runner. Merging the XT16 experimental branch, live robot or Jetson validation, and claims of absolute trajectory accuracy are out of scope.

## Context and current state

- The repository is `/home/user/ws/fastlio-go2w`; the current branch is `main`; HEAD is `a818030` and matches `origin/main`. The worktree contains the user's untracked `results/`; it must not be modified, deleted, or included in commits.
- `scripts/fastlio/replay.sh` launches `fastlio_go2w_bringup/replay.launch.py` and uses `mid360_go2w.yaml` by default. That configuration sets `common.lid_topic=/livox/lidar`, `common.imu_topic=/livox/imu`, and `preprocess.lidar_type=1`.
- FAST-LIO interprets `lidar_type=1` as a `livox_ros_driver2/msg/CustomMsg` subscription on `/livox/lidar`. The legacy bag `/mnt/data1/experimental_data/go2w-experiment-recorder/bags/experiment_long3_20260714_014823` uses that type and matches the current path.
- The new bag `/mnt/data1/experimental_data/go2w-experiment-recorder/bags/experiment_save-test_20260727_022320` records 14,183 `/livox/lidar` messages as `sensor_msgs/msg/PointCloud2`. Runtime reproduction showed `/livox/imu` at about 200 Hz, but the PointCloud2 publisher could not connect to the CustomMsg subscriber, so `/cloud_registered` and `/Odometry` remained silent. All three SQLite chunks returned `PRAGMA quick_check=ok`; the direct cause is a type-contract mismatch, not bag corruption.
- The new Livox PointCloud2 contains FLOAT32 `x/y/z/intensity`, UINT8 `tag/line`, and FLOAT64 `timestamp`. The current driver sets the header stamp from `pkg.base_time`, each point timestamp to absolute nanoseconds, and intensity from the original uint8 reflectivity. It can therefore be converted back almost exactly with `offset_time = round(point.timestamp) - header_stamp_ns`. Precision below the sub-microsecond quantization introduced by the double cannot be recovered, but the per-point time required for scan motion compensation remains available.
- Do not adopt the simple alternative of sending the new PointCloud2 through FAST-LIO with `lidar_type=4`. The current MID-360 PointCloud2 handler ignores the new `timestamp`, reconstructs point time from angle, and expects `reflectivity` instead of `intensity`; even if it displays a cloud, it does not preserve the legacy timing contract.
- The local `feat/xt16-go2w-imu` branch contains three commits implementing an XT16 adapter for `/points_raw` and `/go2w/imu`, but it is intentionally isolated from stable MID-360 `main`. This implementation must not merge or cherry-pick that branch and must address only backward compatibility between the two MID-360 message types on the same topic.
- ROS 2 Humble in the devcontainer is the authoritative runtime. The bag path inside the container is `/mnt/go2w-experiment-recorder/bags/...`, not the host `/mnt/data1/...`. Do not use the host ROS 2 Jazzy overlay for validation.

## Decisions and constraints

- Preserve the legacy CustomMsg path without changing its processing. Do not start a conversion adapter for legacy bags.
- Normalize only the new PointCloud2 input through a dedicated boundary adapter into CustomMsg and reuse FAST-LIO's existing Livox CustomMsg preprocessing. Do not add new-format-specific logic to the third-party FAST-LIO `preprocess.cpp`.
- Detect the bag format from `metadata.yaml` before playback begins. Do not create simultaneous subscribers of different types on the same topic and guess at runtime.
- Provide `--lidar-format auto|custom-msg|pointcloud2`; default to `auto`. Fail closed before playback on an explicit/metadata mismatch, unknown type, missing topic, multiple types for the same name, or malformed metadata.
- Detection must verify the exact ROS type of `/livox/lidar`, not only the topic name. Support only `livox_ros_driver2/msg/CustomMsg` and `sensor_msgs/msg/PointCloud2`.
- The PointCloud2 adapter must not assume a fixed 26-byte stride. Validate field metadata, datatype, offset, count, endianness, `point_step`, `row_step`, organized clouds, row padding, and data length. Accept both the current 26-byte layout and padded layouts that satisfy the same field contract.
- Require FLOAT32 `x/y/z/intensity`, UINT8 `tag/line`, and FLOAT64 `timestamp`. Validate the header, finite values for every point, integral intensity in `0..255`, timestamp range relative to the header, fit in a uint32 offset, and monotonic frame headers. Reject an entire frame containing a critical anomaly rather than partially consuming it, and publish a reason-specific counter.
- Interpret point timestamps as absolute nanoseconds, apply `llround`, and subtract the header timestamp in nanoseconds to form CustomPoint `offset_time`. Stable-sort by timestamp while preserving equal-timestamp order and count reordered frames in diagnostics.
- Populate output CustomMsg header and timebase from the input header and preserve x/y/z, reflectivity, tag, line, and offset time. Set the unavailable `lidar_id` and reserved fields to zero, and lock in with code inspection and unit tests that FAST-LIO does not use them.
- Publish adapter output on the separate `/livox/lidar_fastlio` topic so the graph never exposes the same topic name with two types. Keep PointCloud2 input on `/livox/lidar`. Switch only FAST-LIO's `common.lid_topic` through a launch parameter override.
- Publish received, converted, drop-reason, reordered, and latest-scan-width data as `diagnostic_msgs/msg/DiagnosticArray`. Persist final counters as a JSON artifact and in the offline manifest.
- Use the same detector, format names, adapter, and topic override in interactive replay and the headless offline runner. Do not fix only one entry point and let their contracts diverge.
- Preserve all tuning supplied through `--config`; format selection may change only the input topic override and whether the adapter runs.
- Treat software completion and publication of clouds/odometry separately from trajectory quality, live hardware, Jetson readiness, and absolute accuracy. Do not claim ATE/RPE or physical readiness without ground truth and relevant testing.
- In the fresh context, create `feat/livox-dual-format-replay` from `main`. Do not use a `codex/` prefix. Do not push, merge, or delete existing branches unless the user explicitly asks.

## Final plan

1. Recheck state in the fresh implementation context.
   - Change to `/home/user/ws/fastlio-go2w`, read every applicable `AGENTS.md`, and read this plan in full.
   - Check `git status --short --branch`, `git rev-parse HEAD`, and `git branch -a`. Confirm the relationship between `main` and `origin/main`, the user-owned `results/`, and any running FAST-LIO, rosbag, or RViz processes. Evaluate any drift from this plan before changing code.
   - Run `git switch -c feat/livox-dual-format-replay` from clean `main`. Do not stash, restore, or delete existing dirty files.

2. Add the rosbag metadata Livox format detector.
   - Add `scripts/fastlio/detect_livox_bag_format.py` and parse `rosbag2_bagfile_information.topics_with_message_count` with PyYAML.
   - Extract the unique exact type for `/livox/lidar`, returning one stdout token: `custom-msg` for `livox_ros_driver2/msg/CustomMsg` or `pointcloud2` for `sensor_msgs/msg/PointCloud2`.
   - Emit an actionable stderr diagnostic and exit nonzero for missing metadata, parse errors, a missing topic, duplicate types, an unsupported type, or a zero message count.
   - Separate the pure detector function from the CLI and add `scripts/fastlio/test_detect_livox_bag_format.py` tests for legacy, new, missing, zero-count, duplicate, unsupported, and malformed-YAML cases.

3. Add a dedicated C++ adapter package from PointCloud2 to Livox CustomMsg.
   - Create `humble_ws/src/fastlio_go2w_livox/` as an `ament_cmake` package depending on `rclcpp`, `sensor_msgs`, `livox_ros_driver2`, and `diagnostic_msgs`.
   - Separate the conversion library from the node so the library is unit-testable without a ROS graph. Node parameters must include `input_topic`, `output_topic`, `diagnostics_topic`, `max_point_header_delta_sec`, `minimum_points`, and diagnostics publication period.
   - Resolve offsets by field name; do not assume fixed stride. Safely read little- and big-endian data, row padding, and organized clouds, with overflow and pointer-range checks before access.
   - Safely convert headers to nanoseconds and count frame-level drops for header regression, invalid stamps, schema/layout errors, nonfinite coordinates/intensity/timestamp, invalid intensity range or integrality, negative or oversized offsets, and too few points.
   - Stable-sort valid points by absolute timestamp and generate CustomMsg header, timebase, point_num, x/y/z, reflectivity, tag, line, and offset time. Make the output publisher compatible with FAST-LIO's reliable depth-20 subscriber and use an input QoS compatible with the bag publisher.
   - Cover a driver-style 26-byte sample, padded/organized/big-endian layouts, value preservation, timestamp deltas, double-nanosecond quantization boundaries, stable sorting, every drop path, header regression, and uint32 boundaries in `test/test_pointcloud_adapter.cpp`. Use a fixture to prove the expected CustomMsg is reconstructed from driver-style PointCloud2.

4. Plumb format selection and topic override through the launch graph.
   - Add a runtime dependency on the new adapter package to `fastlio_go2w_bringup/package.xml`.
   - Add an optional `lid_topic_override` launch argument to `fastlio.launch.py`. Preserve the YAML unchanged when empty; only add a `common.lid_topic` parameter override when nonempty.
   - Forward `lid_topic_override` through `bringup.launch.py`.
   - Add a resolved `lidar_format` argument to `replay.launch.py`. For `custom-msg`, start no adapter and use the legacy topic. For `pointcloud2`, start `fastlio_go2w_livox` and pass `/livox/lidar_fastlio` to FAST-LIO. Reject unknown values before processing starts.
   - Apply the same argument and node composition to `offline_fastlio.launch.py`. Put shared graph construction in a bringup helper, or enforce equivalent minimal logic with synchronization tests so interactive and offline processing remain identical.
   - Do not change the RViz `/cloud_registered` topic, odometry adapter, robot description, or existing tuning files.

5. Update `scripts/fastlio/replay.sh` as a backward-compatible CLI.
   - Add `--lidar-format auto|custom-msg|pointcloud2` to usage, default to `auto`, and invoke the detector.
   - Even for explicit formats, inspect metadata and stop before playback if the explicit value conflicts. Display the resolved format and pass `lidar_format:=...` to `replay.launch.py`.
   - Harden ROS preflight: if `/opt/ros/humble/setup.bash` exists, source Humble correctly even when the current `ROS_DISTRO` is Jazzy or another distribution. If Humble or a current matching overlay is unavailable, stop with devcontainer and container-path guidance.
   - Preserve the meaning and defaults of `--rviz/--no-rviz`, `--rate`, and `--config`.
   - Cover unknown options, format mismatch, and host/container path errors with `bash -n` and CLI error tests.

6. Integrate the same format contract and provenance into the headless offline runner.
   - Add `--lidar-format` to `scripts/offline/run_fastlio_offline.sh` and resolve it through the same detector. Continue limiting player topics to `/livox/lidar` and `/livox/imu`.
   - Pass the resolved format to the launch command. Preserve legacy endpoint readiness for CustomMsg. For PointCloud2, type-check the `/livox/lidar` PointCloud2 publisher-to-adapter subscription and `/livox/lidar_fastlio` CustomMsg adapter publisher-to-FAST-LIO subscription. Do not rely only on publisher/subscriber counts.
   - Switch the expected live `common.lid_topic` by format. Add adapter-node readiness, diagnostics topic, and process-metrics targets only for PointCloud2.
   - For PointCloud2, persist adapter diagnostics in the result bag or with a dedicated collector, extract final counters into `livox_adapter_diagnostics.json`, and add an extraction script plus unit tests. Include received, converted, dropped, reordered, and scan-width data in the manifest.
   - Extend the manifest with detected metadata type, resolved format, explicit override, adapter enabled/topic/config, adapter source/runtime hashes, and diagnostics artifact/hash. For CustomMsg, state explicitly that the adapter was disabled without breaking the existing artifact contract.
   - Add the adapter executable/hash to runtime overlay preflight only for PointCloud2, rejecting stale builds. Verify cleanup, drain, and error handling terminate the adapter with the launch process group.

7. Update documentation and operator guidance.
   - Document auto detection, both supported types, container paths, resolved-format output, and explicit override examples in the README replay section.
   - Document format provenance, adapter diagnostics artifacts, and adapter absence on legacy runs in `docs/offline-result-artifacts.md`.
   - Explain why direct `lidar_type=4` is not used, why software output and trajectory quality are separate, and why XT16 is outside this change.

8. Run static, unit, and build tests.
   - Run `bash -n scripts/fastlio/replay.sh scripts/offline/run_fastlio_offline.sh` on the host.
   - Run ROS-independent Python tests, including `python3 -m pytest -q scripts/fastlio/test_detect_livox_bag_format.py` and the diagnostics extractor tests.
   - Rebuild in the Humble devcontainer and run `colcon test` and `colcon test-result --verbose` for the new package, bringup, and FAST-LIO. Verify source/runtime launch hashes and the adapter executable pass preflight.
   - Run all existing bringup Python and offline script tests and confirm no regression in the old CLI, configuration, or result-artifact contract.

9. Validate incrementally with legacy and new real bags.
   - For legacy `/mnt/go2w-experiment-recorder/bags/experiment_long3_20260714_014823`, verify detection returns `custom-msg`, no adapter node exists, and the single-type `/livox/lidar` connects to FAST-LIO.
   - For new `/mnt/go2w-experiment-recorder/bags/experiment_save-test_20260727_022320`, verify detection returns `pointcloud2`, input and canonical output use separate topics and types, and adapter received/converted counters increase with zero critical drops.
   - Run a 30-second isolated-domain headless smoke for each bag in a new empty output directory. Verify nonzero finite `/cloud_registered`, `/Odometry`, and `/odom`, no processes remain after cleanup, and each manifest records correct format/provenance.
   - Run a longer interval for the new bag if needed and analyze coverage, timestamp regressions, trajectory jumps, and nonfinite poses. Do not treat artifact generation or exit status alone as trajectory-quality acceptance.
   - Finally run interactive `replay.sh` once for each bag and inspect `/cloud_registered`, the RViz fixed frame, and TF errors. Record GUI observations separately from automated tests.

10. Hand off the exact change scope and evidence.
   - Run `git diff --check`, `git status --short`, and inspect the branch diff, confirming the user's `results/` and bags are not included.
   - Report the implemented format matrix, test results, runtime evidence for each bag, and unverified boundaries.
   - Do not commit, push, open a PR, or merge unless the user explicitly asks.

## Validation

At minimum, run the following during implementation. Adjust only as required by final package/test paths and record why.

```bash
cd /home/user/ws/fastlio-go2w
git status --short --branch
bash -n scripts/fastlio/replay.sh
bash -n scripts/offline/run_fastlio_offline.sh
python3 -m pytest -q scripts/fastlio/test_detect_livox_bag_format.py
```

Inside the Humble devcontainer:

```bash
cd /workspaces/fastlio-go2w
bash .devcontainer/postCreate.sh
source /opt/ros/humble/setup.bash
source .devcontainer/desktop_ws/install/setup.bash
colcon test --base-paths humble_ws/src --packages-select fastlio_go2w_livox fastlio_go2w_bringup
colcon test-result --verbose
```

Format detection:

```bash
python3 scripts/fastlio/detect_livox_bag_format.py \
  /mnt/go2w-experiment-recorder/bags/experiment_long3_20260714_014823/metadata.yaml
python3 scripts/fastlio/detect_livox_bag_format.py \
  /mnt/go2w-experiment-recorder/bags/experiment_save-test_20260727_022320/metadata.yaml
```

Expected outputs are `custom-msg` and `pointcloud2`, respectively. An explicit override mismatch must exit nonzero.

Use new output directories for headless smoke tests; never overwrite existing results.

```bash
OLD_OUT="$(mktemp -d /tmp/fastlio-old-custom-XXXXXX)/run"
NEW_OUT="$(mktemp -d /tmp/fastlio-new-pointcloud2-XXXXXX)/run"

bash scripts/offline/run_fastlio_offline.sh \
  /mnt/go2w-experiment-recorder/bags/experiment_long3_20260714_014823 \
  --duration 30 --output "$OLD_OUT" --no-analyze

bash scripts/offline/run_fastlio_offline.sh \
  /mnt/go2w-experiment-recorder/bags/experiment_save-test_20260727_022320 \
  --duration 30 --output "$NEW_OUT" --no-analyze
```

For each run, verify manifest state `completed` and nonzero `/cloud_registered`, `/Odometry`, and `/odom` counts in the result bag. The legacy run must report adapter disabled. The new run must report adapter enabled, received/converted greater than zero, and zero critical drops. These smoke tests establish runnable software, not trajectory-quality acceptance.

## Acceptance criteria

- `replay.sh BAG` with no format option correctly auto-detects legacy CustomMsg and new PointCloud2 from metadata.
- A legacy CustomMsg bag enters the existing FAST-LIO Livox callback without an adapter and publishes `/cloud_registered` and odometry.
- A new PointCloud2 bag creates no same-name/multiple-type endpoint, is converted through the dedicated adapter, enters FAST-LIO as CustomMsg, and publishes `/cloud_registered` and odometry.
- Tests cover explicit `--lidar-format`, auto detection, mismatch rejection, and unknown-type rejection.
- The adapter does not depend on a fixed 26-byte layout and handles field/schema/layout/time/value anomalies fail-closed with diagnostics.
- The new-bag smoke reports zero critical adapter drops; the legacy-bag smoke starts no adapter.
- The offline manifest and artifacts make the input ROS type, resolved format, adapter enablement/hashes/counters, runtime config, and bag metadata hash reproducibly traceable.
- Existing `--config`, rate, RViz, legacy offline artifacts, and MID-360 tuning do not regress.
- The user's `results/`, source bags, and `feat/xt16-go2w-imu` remain unchanged.
- Reports and documentation preserve the boundary between runnable software and trajectory quality, Jetson, live hardware, and absolute accuracy.

## Risks / cautions

- Absolute nanoseconds stored in FLOAT64 cannot represent every integer nanosecond at epoch-scale magnitudes. The adapter must `llround` before subtracting the header and test quantization and allowed range, but lost low bits cannot be reconstructed.
- Intensity-to-uint8 reflectivity is reversible for the current Livox driver output. Do not silently round fractional or out-of-range intensity from another producer; reject it as outside the contract rather than weakening validation.
- Publisher/subscriber counts alone can miss a type mismatch; readiness must be type-aware.
- Incorrect launch-parameter ordering could overwrite user configuration unintentionally. Tests must prove only `lid_topic_override` changes and all other tuning remains intact.
- The adapter adds serialization and copy load. It is expected to be acceptable for desktop offline replay, but measure CPU and memory. Consider a native FAST-LIO PointCloud2 handler only as a separate design if measured performance is insufficient.
- The legacy and new bags are not the same run, so their trajectories cannot directly prove adapter accuracy. Keep unit-level reconstruction tests separate from per-bag sanity checks.
- Legacy `long3` and past XT16 validation may contain known trajectory-quality failures. A successful 30-second output smoke does not qualify a full run.
- Devcontainer mount paths differ from host paths. Do not accidentally run validation in the host Jazzy environment.
- This plan is limited to MID-360 format compatibility. Integrating the XT16 branch or adding `--sensor xt16` to `main` requires a separate review of branch boundaries and existing validation.
