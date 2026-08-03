import pytest

from fastlio_go2w_bringup.fusion_profiles import (
    resolve_online_fusion_profile,
    validate_missing_sensor_policy,
)


def test_online_profiles_resolve_exact_strides():
    assert resolve_online_fusion_profile("fused-matched") == {
        "mid_point_stride": 6,
        "hesai_firing_stride": 22,
    }
    assert resolve_online_fusion_profile("fused-full") == {
        "mid_point_stride": 1,
        "hesai_firing_stride": 1,
    }


@pytest.mark.parametrize("profile", ["fused-high", "baseline", "unknown"])
def test_online_profiles_reject_offline_only_values(profile):
    with pytest.raises(ValueError):
        resolve_online_fusion_profile(profile)


@pytest.mark.parametrize("policy", ["strict", "mid360-fallback"])
def test_missing_sensor_policy_accepts_public_values(policy):
    assert validate_missing_sensor_policy(policy) == policy


def test_missing_sensor_policy_rejects_implicit_fallback():
    with pytest.raises(ValueError):
        validate_missing_sensor_policy("mid-only")
