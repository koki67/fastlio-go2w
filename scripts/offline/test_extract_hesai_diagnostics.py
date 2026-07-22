from types import SimpleNamespace

import pytest

from extract_hesai_diagnostics import diagnostic_document


def _message(**overrides):
    values = {
        "received_frames": "12",
        "converted_frames": "10",
        "dropped_schema": "0",
        "dropped_layout": "0",
        "dropped_header_regression": "0",
        "dropped_invalid_header": "0",
        "dropped_nonfinite_timestamp": "0",
        "dropped_timestamp_out_of_range": "2",
        "dropped_too_few_points": "0",
        "invalid_points": "3",
        "latest_scan_width_sec": "0.099",
        "lidar_time_offset_sec": "-0.005",
        "latest_drop_reason": "timestamp_out_of_range",
    }
    values.update(overrides)
    status = SimpleNamespace(
        name="fastlio_go2w_hesai/pointcloud_adapter",
        level=0,
        message="frame_dropped",
        hardware_id="recorded_pandar_xt16",
        values=[SimpleNamespace(key=key, value=value) for key, value in values.items()],
    )
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=100, nanosec=25)),
        status=[status],
    )


def test_diagnostic_document_types_and_validates_accounting():
    message = _message()
    message.status[0].level = b"\x00"
    document = diagnostic_document(
        message, topic="/diagnostics", message_count=12, bag_timestamp_ns=99
    )

    assert document["message_count"] == 12
    assert document["values"]["received_frames"] == 12
    assert document["values"]["dropped_timestamp_out_of_range"] == 2
    assert document["values"]["lidar_time_offset_sec"] == -0.005
    assert document["last_header_stamp_ns"] == 100_000_000_025
    assert document["status"]["level"] == 0


def test_diagnostic_document_rejects_counter_mismatch():
    with pytest.raises(ValueError, match="accounting mismatch"):
        diagnostic_document(
            _message(converted_frames="9"),
            topic="/diagnostics",
            message_count=12,
            bag_timestamp_ns=99,
        )


def test_diagnostic_document_rejects_missing_key():
    message = _message()
    message.status[0].values = [
        item for item in message.status[0].values if item.key != "invalid_points"
    ]
    with pytest.raises(ValueError, match="missing keys"):
        diagnostic_document(
            message, topic="/diagnostics", message_count=12, bag_timestamp_ns=99
        )
