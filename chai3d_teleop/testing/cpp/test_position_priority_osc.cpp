#include "rt_osc_common.hpp"

#include <iostream>

using teleop::osc::PositionPriorityOperationalSpaceTorque;

int main()
{
    constexpr int dof = 7;
    Eigen::Matrix<double, dof, dof> seed;
    seed << 1.0, 0.2, 0.1, 0.0, 0.1, 0.0, 0.0,
            0.1, 1.2, 0.0, 0.2, 0.0, 0.1, 0.0,
            0.0, 0.2, 1.1, 0.1, 0.0, 0.0, 0.2,
            0.1, 0.0, 0.2, 1.0, 0.1, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.2, 1.3, 0.1, 0.0,
            0.2, 0.0, 0.1, 0.0, 0.2, 1.1, 0.1,
            0.0, 0.2, 0.0, 0.1, 0.0, 0.1, 1.0;
    const Eigen::Matrix<double, dof, dof> mass
        = seed.transpose() * seed
          + 0.5 * Eigen::Matrix<double, dof, dof>::Identity();
    Eigen::Matrix<double, 3, dof> position_jacobian;
    position_jacobian
        << 0.8, 0.1, -0.2, 0.3, 0.0, 0.1, -0.1,
           0.0, 0.7, 0.2, -0.1, 0.4, 0.0, 0.2,
           0.1, -0.2, 0.9, 0.0, 0.1, 0.3, 0.0;
    Eigen::Matrix<double, 3, dof> orientation_jacobian;
    orientation_jacobian
        << 0.2, 0.5, 0.1, -0.3, 0.4, 0.2, 0.0,
           -0.1, 0.0, 0.3, 0.6, 0.1, -0.2, 0.4,
           0.4, -0.2, 0.0, 0.1, 0.5, 0.3, -0.1;

    // Ask only for a large secondary orientation correction. A strict
    // hierarchy must produce essentially zero primary Cartesian acceleration.
    const auto result = PositionPriorityOperationalSpaceTorque(mass,
        position_jacobian, orientation_jacobian, Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(), Eigen::Vector3d(0.7, -0.6, 0.5),
        Eigen::Vector3d::Zero(), Eigen::VectorXd::Zero(dof),
        Eigen::VectorXd::Zero(dof), 160.0, 32.0, 18.0, 8.0, 5.0, 2.0,
        0.03, 1.0);
    const Eigen::Vector3d leaked_primary_acceleration
        = position_jacobian * mass.ldlt().solve(result.torque);
    const double leakage = leaked_primary_acceleration.norm();
    std::cout << "position_acceleration_leakage=" << leakage << '\n';
    if (!std::isfinite(leakage) || leakage > 1e-6) return 1;

    return 0;
}
