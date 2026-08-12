import numpy as np

from tools.dynamic_inertia import _csv_header, _target


def test_dynamic_inertia_target_starts_and_ends_at_center() -> None:
    center = np.array([0.2, -0.4])
    amplitude = np.deg2rad([3.0, 5.0])
    assert np.allclose(_target(center, amplitude, 0.0, 30.0, 12.0), center)
    assert np.allclose(_target(center, amplitude, 30.0, 30.0, 12.0), center)


def test_dynamic_inertia_target_never_exceeds_requested_amplitude() -> None:
    center = np.array([-0.1, 0.3])
    amplitude = np.deg2rad([3.0, 5.0])
    for elapsed in np.linspace(0.0, 30.0, 1001):
        target = _target(center, amplitude, elapsed, 30.0, 12.0)
        assert np.all(np.abs(target - center) <= amplitude + 1e-12)


def test_dynamic_inertia_csv_contains_full_matrices_and_eigenvalues() -> None:
    header = _csv_header()
    assert len(header) == 55
    assert all(f"M_wrist_{row}{column}" in header for row in range(2) for column in range(2))
    assert all(
        f"I_spatial_flange_{row}{column}" in header
        for row in range(6)
        for column in range(6)
    )
