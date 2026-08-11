from __future__ import annotations

from src.freedrive_configuration import (
    CARTESIAN_MASK,
    CARTESIAN_MASK_PARAM,
    DISABLE_ELBOW_MOTION,
    ELBOW_PARAM,
    EXPECTED_ROBOT_MODEL,
    ConfigurationError,
    FreedriveConfiguration,
    normalize_robot_model,
    software_version_is_supported,
)


def test_freedrive_is_fixed_six_axis_cartesian_with_elbow_disabled() -> None:
    cfg = FreedriveConfiguration()
    params = cfg.primitive_params()
    assert cfg.primitive_name() == "FloatingCartesian"
    assert params == {
        CARTESIAN_MASK_PARAM: list(CARTESIAN_MASK),
        ELBOW_PARAM: DISABLE_ELBOW_MOTION,
    }


def test_no_unverified_parameters_are_sent() -> None:
    assert set(FreedriveConfiguration().primitive_params()) == {
        CARTESIAN_MASK_PARAM,
        ELBOW_PARAM,
    }


def test_invalid_rate_configuration_is_rejected() -> None:
    try:
        FreedriveConfiguration(sample_period_s=0.0).validate()
        raise AssertionError("expected sample_period_s validation failure")
    except ConfigurationError:
        pass


def test_print_mode_reports_locked_startup_without_connection() -> None:
    cfg = FreedriveConfiguration(print_command_only=True)
    report = cfg.command_report()
    assert report["execute_primitive"]["name"] == "FloatingCartesian"
    assert report["effective_configuration"]["floatingAxis"] == [1, 1, 1, 1, 1, 1]
    assert report["effective_configuration"]["enableElbowMotion"] == 0
    assert report["startup_sequence"]["starts_locked"] is True
    assert report["startup_sequence"]["home_before_freedrive"] is False
    assert report["startup_sequence"]["zero_ft_before_freedrive"] is True


def test_rizon4s_identity_and_rdk_19_software_series() -> None:
    assert EXPECTED_ROBOT_MODEL == "Rizon4s"
    assert normalize_robot_model("Rizon 4S") == "rizon4s"
    assert software_version_is_supported("v3.11.0")
    assert software_version_is_supported("3.11")
    assert not software_version_is_supported("3.9.3")
