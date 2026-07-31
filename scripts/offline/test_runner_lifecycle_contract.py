from pathlib import Path


RUNNER = Path(__file__).with_name("run_fastlio_offline.sh")
MULTILIDAR_RUNNER = Path(__file__).with_name("run_multilidar_experiment.sh")


def _runner_source():
    return RUNNER.read_text(encoding="utf-8")


def test_parameter_snapshot_dump_retries_transient_discovery_failures():
    source = _runner_source()

    assert "dump_node_parameters()" in source
    assert "local deadline=$((SECONDS + 30))" in source
    assert 'dump_node_parameters /fastlio_mapping "$FASTLIO_PARAMETERS_SNAPSHOT"' in source


def test_playback_health_uses_launch_process_group_not_ros_graph_lookup():
    source = _runner_source()

    assert "for command in ros2 setsid pgrep" in source
    assert "required_processing_processes_alive()" in source
    assert 'pgrep -g "$LAUNCH_PID" -f -- "$process_name"' in source
    assert "required_processing_nodes_alive" not in source


def test_multilidar_runner_uses_the_same_lifecycle_contract():
    source = MULTILIDAR_RUNNER.read_text(encoding="utf-8")

    assert "for command in ros2 setsid pgrep" in source
    assert "local deadline=$((SECONDS + 30))" in source
    assert 'dump_node_parameters /dual_lidar_fusion "$FUSION_PARAMETERS_SNAPSHOT"' in source
    assert "required_processing_processes_alive()" in source
    assert 'pgrep -g "$LAUNCH_PID" -f -- "$process_name"' in source
    assert "required_processing_nodes_alive" not in source
