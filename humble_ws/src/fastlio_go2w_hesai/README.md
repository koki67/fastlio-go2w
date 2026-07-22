# fastlio_go2w_hesai

Offline-only adapter from the recorded Pandar XT16 `sensor_msgs/PointCloud2`
schema on `/points_raw` to FAST-LIO's Velodyne-style schema on
`/points_raw_fastlio`.

The adapter preserves `x`, `y`, `z`, `intensity`, and `ring`; converts each
absolute `timestamp` (`FLOAT64` seconds) to a relative `time` (`FLOAT32`
seconds from the original cloud header); stable-sorts by point time; and shifts
only the output header by `lidar_time_offset_sec`.

Malformed schemas, regressing headers, non-finite point timestamps, and point
timestamps more than `max_point_header_delta_sec` from the raw header reject
the complete frame. Invalid coordinate/intensity/ring values reject individual
points, with the frame rejected if fewer than `minimum_points` remain.

This package does not launch or depend on a live Hesai driver.
The recorded `/points_raw` contract is Reliable; the adapter uses a Reliable
depth-100 input queue so repeated offline runs do not lose large cloud samples
through a best-effort depth-5 subscription.
