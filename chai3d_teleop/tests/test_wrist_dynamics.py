import numpy as np

from src.nine_dof_core import WristGeometry
from src.wrist_dynamics import WristDynamics, WristInertialParameters


def _model() -> WristDynamics:
    geometry = WristGeometry(
        joint1_origin_flange_m=np.array([0.0, 0.0, 0.112]),
        joint1_axis_flange=np.array([0.0, 1.0, 0.0]),
        joint2_offset_after_joint1_m=np.zeros(3),
        joint2_axis_after_joint1=np.array([0.0, 0.0, -1.0]),
        tip_offset_after_joint2_m=np.array([0.0079, -0.0311, 0.2306]),
        probe_rotation_at_zero=np.eye(3),
        joint_min_rad=np.deg2rad([-90.0, -180.0]),
        joint_max_rad=np.deg2rad([90.0, 180.0]),
    )
    params = WristInertialParameters(
        link1_mass_kg=0.5,
        link1_com_after_joint1_m=np.array([0.0, 0.0, -0.03]),
        link1_inertia_com_kg_m2=np.diag([0.001, 0.001, 0.001]),
        link2_mass_kg=0.341,
        link2_com_after_joint2_m=np.array([0.00395, -0.01555, 0.1153]),
        link2_inertia_com_kg_m2=np.diag([0.002, 0.002, 0.0005]),
        reflected_joint_inertia_kg_m2=np.array([0.008, 0.003]),
        viscous_friction_nm_s_rad=np.array([0.02, 0.02]),
        coulomb_friction_nm=np.array([0.05, 0.03]),
        torque_bias_nm=np.zeros(2),
    )
    return WristDynamics(geometry, params)


def test_mass_matrix_is_symmetric_positive_definite_across_workspace() -> None:
    model = _model()
    matrices = []
    for q8 in np.deg2rad([-70.0, 0.0, 70.0]):
        for q9 in np.deg2rad([-150.0, 0.0, 150.0]):
            matrix = model.mass_matrix(np.array([q8, q9]))
            assert np.allclose(matrix, matrix.T, atol=1e-12)
            assert np.min(np.linalg.eigvalsh(matrix)) > 0.0
            matrices.append(matrix)
    # The distal body changes orientation, so M(q) is evaluated rather than
    # treated as one fixed calibration matrix.
    assert any(not np.allclose(matrices[0], item) for item in matrices[1:])


def test_coriolis_is_zero_at_zero_velocity() -> None:
    model = _model()
    assert np.allclose(
        model.coriolis_torque(np.array([0.3, -0.4]), np.zeros(2)), np.zeros(2)
    )


def test_gravity_compensation_reverses_with_flange_upside_down() -> None:
    model = _model()
    q = np.array([0.2, -0.3])
    normal = model.gravity_compensation(q, np.eye(3))
    upside_down = np.diag([1.0, -1.0, -1.0])
    inverted = model.gravity_compensation(q, upside_down)
    assert np.allclose(inverted, -normal, atol=1e-10)
