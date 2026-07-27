from types import SimpleNamespace

import pytest

from extract_livox_adapter_diagnostics import diagnostic_document


def message(**overrides):
    values = {
        "received_frames": "12",
        "converted_frames": "10",
        "critical_drops": "2",
        "reordered_frames": "1",
        "quantization_clamped_frames": "4",
        "quantization_clamped_points": "4",
        "dropped_invalid_header": "0",
        "dropped_header_regression": "0",
        "dropped_schema": "0",
        "dropped_layout": "0",
        "dropped_nonfinite_coordinate": "0",
        "dropped_nonfinite_intensity": "0",
        "dropped_nonfinite_timestamp": "0",
        "dropped_intensity_out_of_range": "0",
        "dropped_intensity_nonintegral": "0",
        "dropped_negative_offset": "0",
        "dropped_offset_too_large": "2",
        "dropped_too_few_points": "0",
        "latest_scan_width_sec": "0.099",
        "latest_drop_reason": "offset_too_large",
    }
    values.update(overrides)
    status = SimpleNamespace(
        name="fastlio_go2w_livox/pointcloud_adapter",
        level=0,
        message="frame_dropped",
        hardware_id="recorded_livox_mid360",
        values=[SimpleNamespace(key=key, value=value) for key, value in values.items()],
    )
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=100, nanosec=25)),
        status=[status],
    )


def document(value):
    return diagnostic_document(
        value, topic="/diagnostics", message_count=12, bag_timestamp_ns=99
    )


def test_types_values_and_validates_accounting():
    value = message()
    value.status[0].level = b"\x00"
    result = document(value)
    assert result["message_count"] == 12
    assert result["values"]["received_frames"] == 12
    assert result["values"]["critical_drops"] == 2
    assert result["values"]["latest_scan_width_sec"] == 0.099
    assert result["last_header_stamp_ns"] == 100_000_000_025
    assert result["status"]["level"] == 0


def test_rejects_counter_mismatch():
    with pytest.raises(ValueError, match="accounting mismatch"):
        document(message(converted_frames="9"))


def test_rejects_critical_drop_total_mismatch():
    with pytest.raises(ValueError, match="critical drop total mismatch"):
        document(message(critical_drops="1"))


def test_rejects_reordered_count_above_converted():
    with pytest.raises(ValueError, match="cannot exceed"):
        document(message(reordered_frames="11"))


def test_rejects_missing_key():
    value = message()
    value.status[0].values = [
        item for item in value.status[0].values if item.key != "received_frames"
    ]
    with pytest.raises(ValueError, match="missing keys"):
        document(value)


def test_rejects_duplicate_key():
    value = message()
    value.status[0].values.append(value.status[0].values[0])
    with pytest.raises(ValueError, match="duplicate diagnostic key"):
        document(value)
