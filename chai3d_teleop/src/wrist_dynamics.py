"""Configuration-dependent rigid-body dynamics for the articulated 2R wrist.

The inertial *parameters* are identified off-line.  The matrices are then
evaluated from the live q8/q9 state; this is the physically meaningful form of
"dynamic inertia" needed by a torque/operational-space controller.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .nine_dof_core import WristGeometry, axis_angle_rotation


def _vector(value: np.ndarray, length: int, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (length,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite {length}-vector")
    return result


def _inertia(value: np.ndarray, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape == (9,):
        result = result.reshape(3, 3)
    if result.shape != (3, 3) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite 3x3 matrix")
    result = 0.5 * (result + result.T)
    if np.min(np.linalg.eigvalsh(result)) < -1e-10:
        raise ValueError(f"{name} must be positive semidefinite")
    return result


@dataclass(frozen=True)
class WristInertialParameters:
    link1_mass_kg: float
    link1_com_after_joint1_m: np.ndarray
    link1_inertia_com_kg_m2: np.ndarray
    link2_mass_kg: float
    link2_com_after_joint2_m: np.ndarray
    link2_inertia_com_kg_m2: np.ndarray
    reflected_joint_inertia_kg_m2: np.ndarray
    viscous_friction_nm_s_rad: np.ndarray
    coulomb_friction_nm: np.ndarray
    torque_bias_nm: np.ndarray
    rigid_body_scale: float = 1.0

    def __post_init__(self) -> None:
        if (
            not math.isfinite(self.link1_mass_kg)
            or not math.isfinite(self.link2_mass_kg)
            or self.link1_mass_kg < 0.0
            or self.link2_mass_kg < 0.0
        ):
            raise ValueError("link masses must be finite and non-negative")
        if not math.isfinite(self.rigid_body_scale) or self.rigid_body_scale <= 0.0:
            raise ValueError("rigid_body_scale must be finite and positive")
        object.__setattr__(
            self,
            "link1_com_after_joint1_m",
            _vector(self.link1_com_after_joint1_m, 3, "link1_com"),
        )
        object.__setattr__(
            self,
            "link2_com_after_joint2_m",
            _vector(self.link2_com_after_joint2_m, 3, "link2_com"),
        )
        object.__setattr__(
            self,
            "link1_inertia_com_kg_m2",
            _inertia(self.link1_inertia_com_kg_m2, "link1_inertia"),
        )
        object.__setattr__(
            self,
            "link2_inertia_com_kg_m2",
            _inertia(self.link2_inertia_com_kg_m2, "link2_inertia"),
        )
        for field_name in (
            "reflected_joint_inertia_kg_m2",
            "viscous_friction_nm_s_rad",
            "coulomb_friction_nm",
            "torque_bias_nm",
        ):
            values = _vector(getattr(self, field_name), 2, field_name)
            if field_name != "torque_bias_nm" and np.any(values < 0.0):
                raise ValueError(f"{field_name} cannot contain negative values")
            object.__setattr__(self, field_name, values)


class WristDynamics:
    """Numerical 2R dynamics with configuration-dependent M, C and g."""

    def __init__(self, geometry: WristGeometry, parameters: WristInertialParameters):
        self.geometry = geometry
        self.parameters = parameters

    def _body_jacobians(
        self, q_rad: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        q = _vector(q_rad, 2, "q_rad")
        p = self.parameters
        r1 = axis_angle_rotation(self.geometry.joint1_axis_flange, float(q[0]))
        r2 = axis_angle_rotation(
            self.geometry.joint2_axis_after_joint1, float(q[1])
        )
        p1 = self.geometry.joint1_origin_flange_m
        p2 = p1 + r1 @ self.geometry.joint2_offset_after_joint1_m
        axis1 = self.geometry.joint1_axis_flange
        axis2 = r1 @ self.geometry.joint2_axis_after_joint1

        c1 = p1 + r1 @ p.link1_com_after_joint1_m
        c2 = p2 + r1 @ r2 @ p.link2_com_after_joint2_m
        jv1 = np.column_stack((np.cross(axis1, c1 - p1), np.zeros(3)))
        jw1 = np.column_stack((axis1, np.zeros(3)))
        jv2 = np.column_stack(
            (np.cross(axis1, c2 - p1), np.cross(axis2, c2 - p2))
        )
        jw2 = np.column_stack((axis1, axis2))
        return jv1, jw1, jv2, jw2, r1, r1 @ r2

    def rigid_body_mass_matrix(self, q_rad: np.ndarray) -> np.ndarray:
        p = self.parameters
        jv1, jw1, jv2, jw2, r1, r12 = self._body_jacobians(q_rad)
        i1 = r1 @ p.link1_inertia_com_kg_m2 @ r1.T
        i2 = r12 @ p.link2_inertia_com_kg_m2 @ r12.T
        matrix = (
            p.link1_mass_kg * (jv1.T @ jv1)
            + jw1.T @ i1 @ jw1
            + p.link2_mass_kg * (jv2.T @ jv2)
            + jw2.T @ i2 @ jw2
        )
        return 0.5 * (matrix + matrix.T)

    def mass_matrix(self, q_rad: np.ndarray) -> np.ndarray:
        p = self.parameters
        result = (
            p.rigid_body_scale * self.rigid_body_mass_matrix(q_rad)
            + np.diag(p.reflected_joint_inertia_kg_m2)
        )
        # A calibrated physical model must be SPD.  Raise instead of silently
        # passing an invalid matrix into an inverse-dynamics/OSC calculation.
        if np.min(np.linalg.eigvalsh(result)) <= 1e-8:
            raise ValueError("calibrated wrist mass matrix is not positive definite")
        return result

    def gravity_compensation(
        self,
        q_rad: np.ndarray,
        flange_rotation_world: np.ndarray,
        gravity_world_m_s2: np.ndarray = np.array([0.0, 0.0, -9.80665]),
    ) -> np.ndarray:
        rotation = np.asarray(flange_rotation_world, dtype=float)
        if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
            raise ValueError("flange_rotation_world must be a finite 3x3 matrix")
        p = self.parameters
        jv1, _, jv2, _, _, _ = self._body_jacobians(q_rad)
        gravity_flange = rotation.T @ _vector(
            gravity_world_m_s2, 3, "gravity_world_m_s2"
        )
        generalized_gravity = (
            jv1.T @ (p.link1_mass_kg * gravity_flange)
            + jv2.T @ (p.link2_mass_kg * gravity_flange)
        )
        return -p.rigid_body_scale * generalized_gravity

    def coriolis_torque(self, q_rad: np.ndarray, dq_rad_s: np.ndarray) -> np.ndarray:
        q = _vector(q_rad, 2, "q_rad")
        dq = _vector(dq_rad_s, 2, "dq_rad_s")
        epsilon = 1e-5
        derivatives = []
        for coordinate in range(2):
            step = np.zeros(2)
            step[coordinate] = epsilon
            derivatives.append(
                (self.mass_matrix(q + step) - self.mass_matrix(q - step))
                / (2.0 * epsilon)
            )
        result = np.zeros(2)
        for i in range(2):
            for j in range(2):
                for k in range(2):
                    christoffel = 0.5 * (
                        derivatives[k][i, j]
                        + derivatives[j][i, k]
                        - derivatives[i][j, k]
                    )
                    result[i] += christoffel * dq[j] * dq[k]
        return result

    def friction_torque(self, dq_rad_s: np.ndarray, smoothing_rad_s: float = 0.02) -> np.ndarray:
        dq = _vector(dq_rad_s, 2, "dq_rad_s")
        if smoothing_rad_s <= 0.0:
            raise ValueError("smoothing_rad_s must be positive")
        p = self.parameters
        return (
            p.viscous_friction_nm_s_rad * dq
            + p.coulomb_friction_nm * np.tanh(dq / smoothing_rad_s)
            + p.torque_bias_nm
        )

    def inverse_dynamics(
        self,
        q_rad: np.ndarray,
        dq_rad_s: np.ndarray,
        ddq_rad_s2: np.ndarray,
        flange_rotation_world: np.ndarray,
    ) -> np.ndarray:
        ddq = _vector(ddq_rad_s2, 2, "ddq_rad_s2")
        return (
            self.mass_matrix(q_rad) @ ddq
            + self.coriolis_torque(q_rad, dq_rad_s)
            + self.gravity_compensation(q_rad, flange_rotation_world)
            + self.friction_torque(dq_rad_s)
        )
