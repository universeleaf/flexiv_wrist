import pytest

from scripts.diagnose_wrist_encoder_direction import (
    _shortest_periodic_delta,
    _sign,
)


def test_shortest_periodic_delta_unwraps_both_directions() -> None:
    assert _shortest_periodic_delta(0.02, 0.98) == pytest.approx(0.04)
    assert _shortest_periodic_delta(0.98, 0.02) == pytest.approx(-0.04)


def test_sign_has_deadband() -> None:
    assert _sign(0.5, minimum=1.0) == 0
    assert _sign(2.0, minimum=1.0) == 1
    assert _sign(-2.0, minimum=1.0) == -1
