"""Public profile contract for the online MID-360 + XT16 fusion graph."""

ONLINE_FUSION_PROFILES = {
    "fused-matched": {"mid_point_stride": 6, "hesai_firing_stride": 22},
    "fused-full": {"mid_point_stride": 1, "hesai_firing_stride": 1},
}

MISSING_SENSOR_POLICIES = ("strict", "mid360-fallback")


def resolve_online_fusion_profile(name):
    try:
        return dict(ONLINE_FUSION_PROFILES[name])
    except KeyError as error:
        choices = ", ".join(ONLINE_FUSION_PROFILES)
        raise ValueError(
            f"Unknown online fusion profile '{name}'. Expected one of: {choices}"
        ) from error


def validate_missing_sensor_policy(name):
    if name not in MISSING_SENSOR_POLICIES:
        choices = ", ".join(MISSING_SENSOR_POLICIES)
        raise ValueError(
            f"Unknown missing-sensor policy '{name}'. Expected one of: {choices}"
        )
    return name
