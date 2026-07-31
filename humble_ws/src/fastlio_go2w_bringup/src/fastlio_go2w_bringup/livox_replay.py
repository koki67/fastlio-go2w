"""Shared input-graph contract for interactive and offline Livox replay."""

from dataclasses import dataclass


CUSTOM_MSG_FORMAT = "custom-msg"
POINTCLOUD2_FORMAT = "pointcloud2"
SUPPORTED_LIDAR_FORMATS = (CUSTOM_MSG_FORMAT, POINTCLOUD2_FORMAT)


@dataclass(frozen=True)
class LivoxReplayGraph:
    lidar_format: str
    adapter_enabled: bool
    input_topic: str
    fastlio_topic: str
    diagnostics_topic: str | None


def resolve_livox_replay_graph(lidar_format: str) -> LivoxReplayGraph:
    if lidar_format == CUSTOM_MSG_FORMAT:
        return LivoxReplayGraph(
            lidar_format=lidar_format,
            adapter_enabled=False,
            input_topic="/livox/lidar",
            fastlio_topic="/livox/lidar",
            diagnostics_topic=None,
        )
    if lidar_format == POINTCLOUD2_FORMAT:
        return LivoxReplayGraph(
            lidar_format=lidar_format,
            adapter_enabled=True,
            input_topic="/livox/lidar",
            fastlio_topic="/livox/lidar_fastlio",
            diagnostics_topic="/fastlio_go2w_livox/diagnostics",
        )
    supported = ", ".join(SUPPORTED_LIDAR_FORMATS)
    raise ValueError(
        f"unsupported resolved lidar_format {lidar_format!r}; expected one of: {supported}"
    )
