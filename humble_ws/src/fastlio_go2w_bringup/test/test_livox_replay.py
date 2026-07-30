import pytest

from fastlio_go2w_bringup.livox_replay import resolve_livox_replay_graph


def test_custom_msg_graph_keeps_legacy_topic_without_adapter():
    graph = resolve_livox_replay_graph("custom-msg")
    assert not graph.adapter_enabled
    assert graph.input_topic == "/livox/lidar"
    assert graph.fastlio_topic == "/livox/lidar"
    assert graph.diagnostics_topic is None


def test_pointcloud2_graph_uses_separate_custom_message_topic():
    graph = resolve_livox_replay_graph("pointcloud2")
    assert graph.adapter_enabled
    assert graph.input_topic == "/livox/lidar"
    assert graph.fastlio_topic == "/livox/lidar_fastlio"
    assert graph.diagnostics_topic == "/fastlio_go2w_livox/diagnostics"


def test_unknown_format_is_rejected():
    with pytest.raises(ValueError, match="unsupported resolved lidar_format"):
        resolve_livox_replay_graph("auto")
