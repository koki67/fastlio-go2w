# fastlio_go2w_fusion

The package preserves the offline replay contract and also supports a guarded
online path through `fastlio_go2w_bringup/online_multilidar.launch.py`.

Online profiles are deliberately limited to:

- `fused-matched`: MID stride 6, XT16 firing-group stride 22 (default)
- `fused-full`: MID stride 1, XT16 firing-group stride 1 (high load)

`fused-high` remains available only in the existing offline experiment. Online
`strict` mode stops output when either source is stale. The optional
`mid360-fallback` policy emits MID-only data only while MID is fresh and XT16 is
stale. `/livox/imu` remains the sole FAST-LIO IMU input.

The same fusion core supports recorded Livox MID-360 and Hesai Pandar XT16
clouds in offline experiments. Sensor drivers are owned only by the online
launch; offline launch files continue to own processing nodes only.

The `dual_lidar_fusion_node` subscribes to `/livox/lidar` and `/points_raw` and
publishes a time-sorted Livox `CustomMsg` on `/livox/lidar_fused`. Every output
window uses the MID-360 `timebase` and maximum point offset. Hesai points are
cropped to that window and consumed at most once.

Important parameters:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `mid_point_stride` | `3` | Keep every Nth valid MID point |
| `hesai_firing_stride` | `3` | Keep every Nth XT16 firing group |
| `min_range_m` | `0.5` | Minimum source-frame point range |
| `hesai_time_offset_sec` | `0.0` | Fixed offset added to XT16 timestamps |
| `max_pending_mid_frames` | `32` | Queue limit before MID-only fallback |
| `pending_flush_wall_timeout_sec` | `0.5` | Wall-idle delay before flushing final MID-only frames (`0` disables) |
| `online_watchdog_enabled` | `false` | Enable online source gating; keeps legacy offline behavior by default |
| `fusion_profile` | `offline-custom` | Online profile identity reported in diagnostics |
| `missing_sensor_policy` | `strict` | `strict` or explicit `mid360-fallback` online behavior |
| `source_stale_timeout_sec` | `0.5` | Online maximum source age |
| `startup_grace_sec` | `2.0` | Initial interval before missing-source state is asserted |
| `partial_hesai_min_points` | `0` | Optional low-count partial-cloud diagnostic (`0` disables) |
| `partial_hesai_min_span_sec` | `0.08` | Mark shorter XT16 acquisition spans as partial |
| `publish_debug_cloud` | `false` | Publish a source-labelled debug cloud |
| `hesai_to_livox.translation` | `[-0.018602675, 0, -0.095450199]` | XT16 origin in `livox_frame` |
| `hesai_to_livox.rotation_xyzw` | `[-0.112310121, -0.112310121, 0.698130673, 0.698130673]` | XT16 orientation in `livox_frame` |

Diagnostics are published as `diagnostic_msgs/DiagnosticArray` on
`/fastlio_go2w_fusion/diagnostics`. The optional debug cloud is published on
`/livox/lidar_fused_debug` and includes `source` (`0` MID, `1` XT16), `line`,
and `offset_time` fields.
