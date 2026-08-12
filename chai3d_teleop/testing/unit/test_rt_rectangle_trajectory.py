import math

import pytest

from tools.osc_launcher import _rounded_rectangle_xy


WIDTH = 0.060
HEIGHT = 0.040
RADIUS = 0.010
PERIMETER = (
    2.0 * (WIDTH - 2.0 * RADIUS)
    + 2.0 * (HEIGHT - 2.0 * RADIUS)
    + 2.0 * math.pi * RADIUS
)


def test_rounded_rectangle_is_closed_and_has_requested_extent() -> None:
    start = _rounded_rectangle_xy(0.0, WIDTH, HEIGHT, RADIUS)
    end = _rounded_rectangle_xy(PERIMETER, WIDTH, HEIGHT, RADIUS)
    assert start[:2] == pytest.approx((0.0, 0.0))
    assert end[:2] == pytest.approx(start[:2])

    samples = [
        _rounded_rectangle_xy(PERIMETER * index / 2000, WIDTH, HEIGHT, RADIUS)
        for index in range(2001)
    ]
    xs = [sample[0] for sample in samples]
    ys = [sample[1] for sample in samples]
    assert max(xs) - min(xs) == pytest.approx(WIDTH, abs=1e-6)
    assert max(ys) - min(ys) == pytest.approx(HEIGHT, abs=1e-6)


def test_straight_edges_have_expected_tangent_headings() -> None:
    straight_x = WIDTH - 2.0 * RADIUS
    straight_y = HEIGHT - 2.0 * RADIUS
    arc = 0.5 * math.pi * RADIUS
    distances = (
        0.5 * straight_x,
        straight_x + arc + 0.5 * straight_y,
        2.0 * straight_x + 2.0 * arc + 0.5 * straight_x,
        2.0 * straight_x + straight_y + 3.0 * arc + 0.5 * straight_y,
    )
    headings = [
        _rounded_rectangle_xy(value, WIDTH, HEIGHT, RADIUS)[2]
        for value in distances
    ]
    assert headings == pytest.approx((0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi))
