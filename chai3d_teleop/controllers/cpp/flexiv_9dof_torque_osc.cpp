#include "rt_osc_common.hpp"
#include "teleop_shared_memory.hpp"
#include "wrist_shared_memory.hpp"

#include <flexiv/rdk/model.hpp>
#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/scheduler.hpp>

#include <limits>
#include <map>
#include <memory>
#include <thread>

using namespace flexiv;
using namespace teleop::osc;

namespace {

struct WristModel {
    Eigen::Vector3d joint1_origin {0.0, 0.0, 0.112};
    Eigen::Vector3d joint1_axis {0.0, 1.0, 0.0};
    Eigen::Vector3d joint2_offset {0.0, 0.0, 0.0};
    Eigen::Vector3d joint2_axis {0.0, 0.0, -1.0};
    Eigen::Vector3d tip_offset {0.0079, -0.0311, 0.2306};
    Eigen::Matrix3d probe_rotation_zero {Eigen::Matrix3d::Identity()};
    double mass1 {0.5};
    double mass2 {0.341};
    Eigen::Vector3d com1 {0.0, 0.0, -0.03};
    Eigen::Vector3d com2 {0.00395, -0.01555, 0.1153};
    Eigen::Matrix3d inertia1 {Eigen::Vector3d(0.001, 0.001, 0.0006).asDiagonal()};
    Eigen::Matrix3d inertia2 {Eigen::Vector3d(0.002, 0.002, 0.0005).asDiagonal()};
    double active_tool_mass {0.0};
    Eigen::Vector3d active_tool_com {Eigen::Vector3d::Zero()};
    Eigen::Matrix3d active_tool_inertia {Eigen::Matrix3d::Zero()};
    Eigen::Vector2d reflected_inertia {0.008, 0.003};
    Eigen::Vector2d viscous_friction {0.02, 0.02};
    Eigen::Vector2d coulomb_friction {0.05, 0.03};
    Eigen::Vector2d torque_bias {0.0, 0.0};
    double rigid_body_scale {1.0};
    double priority_inertia_scale {1.0};
    double wrist_priority_gain {1.0};
    Eigen::Vector2d wrist_tracking_kp {3.0, 1.2};
    Eigen::Vector2d wrist_tracking_kd {0.8, 0.35};
    double wrist_target_filter_hz {1.5};
    Eigen::Vector2d wrist_target_max_velocity_rad_s {
        30.0 * kPi / 180.0, 60.0 * kPi / 180.0};
    Eigen::Vector2d wrist_target_max_acceleration_rad_s2 {
        120.0 * kPi / 180.0, 240.0 * kPi / 180.0};
    double wrist_state_velocity_filter_hz {5.0};
    double torque_startup_ramp_s {2.0};
    double stale_recovery_ramp_s {1.0};
    Eigen::Vector2d wrist_rectangle_excursion_rad {
        30.0 * kPi / 180.0, 120.0 * kPi / 180.0};
    double ik_damping {0.03};
    double ik_max_step_rad {4.0 * kPi / 180.0};
    int ik_max_iterations {30};
    double joint_limit_margin_rad {2.0 * kPi / 180.0};
    Eigen::Vector2d joint_min {-0.5 * kPi, -2.0 * kPi};
    Eigen::Vector2d joint_max {0.5 * kPi, 2.0 * kPi};
    double task_translation_kp {80.0};
    double task_translation_kd {22.0};
    double task_rotation_kp {45.0};
    double task_rotation_kd {14.0};
    double nullspace_kp {5.0};
    double nullspace_kd {2.0};
    Eigen::Matrix<double, 7, 1> arm_torque_limit {
        (Eigen::Matrix<double, 7, 1>() << 20.0, 20.0, 15.0, 15.0, 8.0, 8.0, 5.0).finished()};
    Eigen::Matrix<double, 7, 1> arm_torque_slew {
        (Eigen::Matrix<double, 7, 1>() << 200.0, 200.0, 150.0, 150.0, 80.0, 80.0, 50.0).finished()};
    Eigen::Vector2d wrist_torque_slew {15.0, 8.0};
    double teleop_target_filter_hz {8.0};
    double teleop_max_linear_velocity_m_s {0.12};
    double teleop_max_linear_acceleration_m_s2 {0.6};
    double teleop_max_angular_velocity_rad_s {0.35};
    double teleop_max_angular_acceleration_rad_s2 {1.2};
    double teleop_arm_translation_kp {55.0};
    double teleop_arm_translation_kd {20.0};
    double teleop_arm_rotation_kp {18.0};
    double teleop_arm_rotation_kd {9.0};
    double mode2_arm_rotation_kp {18.0};
    double mode2_arm_rotation_kd {8.0};
    double mode2_arm_max_angular_velocity_rad_s {0.20};
    double mode2_arm_max_angular_acceleration_rad_s2 {0.60};
    double mode2_posture_reference_rate_per_s {0.0};
    double teleop_wrist_target_filter_hz {4.0};
    Eigen::Vector2d teleop_wrist_target_max_velocity_rad_s {
        90.0 * kPi / 180.0, 150.0 * kPi / 180.0};
    Eigen::Vector2d teleop_wrist_target_max_acceleration_rad_s2 {
        360.0 * kPi / 180.0, 600.0 * kPi / 180.0};
    double teleop_max_operational_damping {0.5};
    double teleop_singularity_characteristic_length_m {0.25};
    double teleop_singularity_slow_sigma {0.05};
    double teleop_singularity_critical_sigma {0.015};
    double teleop_singularity_min_motion_scale {0.15};
    double teleop_posture_reference_rate_per_s {0.1};
    double clutch_hold_natural_frequency_hz {1.0};
    double clutch_hold_damping_ratio {1.25};
    bool dynamic_wrist_gravity_compensation {true};
    double dynamic_gravity_filter_hz {5.0};
    double mode3_position_kp {160.0};
    double mode3_position_kd {32.0};
    double mode3_arm_rotation_kp {18.0};
    double mode3_arm_rotation_kd {8.0};
    double mode3_arm_max_angular_velocity_rad_s {0.20};
    double mode3_arm_max_angular_acceleration_rad_s2 {0.60};
    double mode3_contact_enable_threshold_n {2.0};
    double mode3_contact_release_threshold_n {1.0};
    double mode3_contact_release_delay_s {0.05};
    double mode3_force_tolerance_n {1.0};
    double mode3_force_full_position_error_m {0.001};
    double mode3_force_disable_position_error_m {0.003};
    double mode3_force_kp {0.25};
    double mode3_force_ki_per_s {0.35};
    double mode3_force_damping_n_s_m {25.0};
    double mode3_force_integral_limit_n {8.0};
    double mode3_force_command_limit_n {30.0};
};

struct WristKinematics {
    Eigen::Vector3d position_flange;
    Eigen::Matrix3d rotation_flange;
    Eigen::Matrix<double, 6, 2> jacobian_flange;
    Eigen::Matrix<double, 3, 2> link1_linear_jacobian;
    Eigen::Matrix<double, 3, 2> link2_linear_jacobian;
    Eigen::Matrix<double, 3, 2> link1_angular_jacobian;
    Eigen::Matrix<double, 3, 2> link2_angular_jacobian;
    Eigen::Vector3d link1_com_position_flange;
    Eigen::Vector3d link2_com_position_flange;
    Eigen::Matrix3d link1_rotation_flange;
    Eigen::Matrix3d link2_rotation_flange;
    Eigen::Matrix2d mass;
};

WristKinematics EvaluateWrist(const WristModel& model, const Eigen::Vector2d& q)
{
    const Eigen::Matrix3d r1
        = Eigen::AngleAxisd(q[0], model.joint1_axis.normalized()).toRotationMatrix();
    const Eigen::Matrix3d r2
        = Eigen::AngleAxisd(q[1], model.joint2_axis.normalized()).toRotationMatrix();
    const Eigen::Vector3d p1 = model.joint1_origin;
    const Eigen::Vector3d p2 = p1 + r1 * model.joint2_offset;
    const Eigen::Vector3d axis1 = model.joint1_axis.normalized();
    const Eigen::Vector3d axis2 = r1 * model.joint2_axis.normalized();
    const Eigen::Matrix3d r12 = r1 * r2;
    const Eigen::Vector3d tip = p2 + r12 * model.tip_offset;
    Eigen::Matrix<double, 6, 2> jacobian;
    jacobian.col(0).head<3>() = axis1.cross(tip - p1);
    jacobian.col(1).head<3>() = axis2.cross(tip - p2);
    jacobian.col(0).tail<3>() = axis1;
    jacobian.col(1).tail<3>() = axis2;

    const Eigen::Vector3d c1 = p1 + r1 * model.com1;
    const Eigen::Vector3d c2 = p2 + r12 * model.com2;
    Eigen::Matrix<double, 3, 2> jv1, jw1, jv2, jw2;
    jv1.col(0) = axis1.cross(c1 - p1);
    jv1.col(1).setZero();
    jw1.col(0) = axis1;
    jw1.col(1).setZero();
    jv2.col(0) = axis1.cross(c2 - p1);
    jv2.col(1) = axis2.cross(c2 - p2);
    jw2.col(0) = axis1;
    jw2.col(1) = axis2;
    Eigen::Matrix2d mass
        = model.rigid_body_scale
          * (model.mass1 * jv1.transpose() * jv1
              + jw1.transpose() * r1 * model.inertia1 * r1.transpose() * jw1
              + model.mass2 * jv2.transpose() * jv2
              + jw2.transpose() * r12 * model.inertia2 * r12.transpose() * jw2);
    mass.diagonal() += model.reflected_inertia;
    mass = 0.5 * (mass + mass.transpose());
    Eigen::LDLT<Eigen::Matrix2d> mass_solver(mass);
    if (mass_solver.info() != Eigen::Success || !mass_solver.isPositive()) {
        throw std::runtime_error("wrist M(q) is not positive definite");
    }
    return {tip, r12 * model.probe_rotation_zero, jacobian, jv1, jv2, jw1,
        jw2, c1, c2, r1, r12, mass};
}

Eigen::Matrix<double, 6, 6> SpatialInertiaAtCom(double mass,
    const Eigen::Matrix3d& rotation_world,
    const Eigen::Matrix3d& inertia_com_body)
{
    Eigen::Matrix<double, 6, 6> result
        = Eigen::Matrix<double, 6, 6>::Zero();
    result.topLeftCorner<3, 3>() = mass * Eigen::Matrix3d::Identity();
    result.bottomRightCorner<3, 3>()
        = rotation_world * inertia_com_body * rotation_world.transpose();
    return result;
}

Eigen::Matrix<double, 6, 6> SpatialInertiaAtOrigin(double mass,
    const Eigen::Vector3d& com, const Eigen::Matrix3d& inertia_com)
{
    Eigen::Matrix<double, 6, 6> result
        = Eigen::Matrix<double, 6, 6>::Zero();
    const Eigen::Matrix3d cross = Skew(com);
    result.topLeftCorner<3, 3>() = mass * Eigen::Matrix3d::Identity();
    result.topRightCorner<3, 3>() = -mass * cross;
    result.bottomLeftCorner<3, 3>() = mass * cross;
    result.bottomRightCorner<3, 3>()
        = inertia_com + mass * cross.transpose() * cross;
    return result;
}

Eigen::Matrix<double, 6, 9> BodyComJacobian(const Eigen::MatrixXd& flange_jacobian,
    const Eigen::Matrix3d& flange_rotation,
    const Eigen::Vector3d& com_position_flange,
    const Eigen::Matrix<double, 3, 2>& wrist_linear_jacobian,
    const Eigen::Matrix<double, 3, 2>& wrist_angular_jacobian)
{
    Eigen::Matrix<double, 6, 9> result
        = Eigen::Matrix<double, 6, 9>::Zero();
    const Eigen::Vector3d offset_world
        = flange_rotation * com_position_flange;
    result.leftCols<7>().topRows<3>()
        = flange_jacobian.topRows<3>()
          - Skew(offset_world) * flange_jacobian.bottomRows<3>();
    result.leftCols<7>().bottomRows<3>()
        = flange_jacobian.bottomRows<3>();
    result.rightCols<2>().topRows<3>()
        = flange_rotation * wrist_linear_jacobian;
    result.rightCols<2>().bottomRows<3>()
        = flange_rotation * wrist_angular_jacobian;
    return result;
}

struct WristGravityResult {
    Eigen::Matrix<double, 7, 1> arm_delta {Eigen::Matrix<double, 7, 1>::Zero()};
    Eigen::Vector2d wrist {Eigen::Vector2d::Zero()};
};

WristGravityResult DynamicWristGravity(const WristModel& model,
    const Eigen::Vector2d& q_wrist, const Eigen::MatrixXd& flange_jacobian,
    const Eigen::Matrix3d& flange_rotation)
{
    // Flexiv's internal gravity model already contains the Elements Tool as a
    // rigid body at q8=q9=0.  Compute the full generalized gravity compensation
    // of the articulated two-body wrist, then send only live-minus-zero to the
    // seven Flexiv joints.  The two external joints are absent from the Flexiv
    // model, so they receive their complete live gravity vector.
    const auto generalized = [&](const Eigen::Vector2d& q)
        -> Eigen::Matrix<double, 9, 1> {
        const WristKinematics wrist = EvaluateWrist(model, q);
        const auto j1 = BodyComJacobian(flange_jacobian, flange_rotation,
            wrist.link1_com_position_flange, wrist.link1_linear_jacobian,
            wrist.link1_angular_jacobian);
        const auto j2 = BodyComJacobian(flange_jacobian, flange_rotation,
            wrist.link2_com_position_flange, wrist.link2_linear_jacobian,
            wrist.link2_angular_jacobian);
        const Eigen::Vector3d gravity_world(0.0, 0.0, -9.80665);
        return (-model.rigid_body_scale
                * (j1.topRows<3>().transpose()
                       * (model.mass1 * gravity_world)
                    + j2.topRows<3>().transpose()
                          * (model.mass2 * gravity_world)))
            .eval();
    };
    const Eigen::Matrix<double, 9, 1> live = generalized(q_wrist);
    const Eigen::Matrix<double, 9, 1> zero
        = generalized(Eigen::Vector2d::Zero());
    WristGravityResult result;
    result.arm_delta = live.head<7>() - zero.head<7>();
    result.wrist = live.tail<2>();
    return result;
}

Eigen::Matrix<double, 9, 9> MovingWristRigidBodyMass(const WristModel& model,
    const Eigen::Vector2d& q, const Eigen::MatrixXd& flange_jacobian,
    const Eigen::Matrix3d& flange_rotation)
{
    const WristKinematics wrist = EvaluateWrist(model, q);
    const auto j1 = BodyComJacobian(flange_jacobian, flange_rotation,
        wrist.link1_com_position_flange, wrist.link1_linear_jacobian,
        wrist.link1_angular_jacobian);
    const auto j2 = BodyComJacobian(flange_jacobian, flange_rotation,
        wrist.link2_com_position_flange, wrist.link2_linear_jacobian,
        wrist.link2_angular_jacobian);
    const auto spatial1 = SpatialInertiaAtCom(model.mass1,
        flange_rotation * wrist.link1_rotation_flange, model.inertia1);
    const auto spatial2 = SpatialInertiaAtCom(model.mass2,
        flange_rotation * wrist.link2_rotation_flange, model.inertia2);
    Eigen::Matrix<double, 9, 9> result
        = model.rigid_body_scale
          * (j1.transpose() * spatial1 * j1 + j2.transpose() * spatial2 * j2);
    return 0.5 * (result + result.transpose());
}

Eigen::Matrix<double, 9, 9> AutoCompositeMass(const WristModel& model,
    const Eigen::MatrixXd& arm_mass, const Eigen::MatrixXd& flange_jacobian,
    const Eigen::Matrix3d& flange_rotation, const Eigen::Vector2d& q_wrist)
{
    // Flexiv Model::M() already contains the active Elements Tool as a rigid
    // body fixed to the flange.  Preserve that calibrated q8=q9=0 baseline,
    // then replace only the wrist-configuration-dependent part and add the
    // missing arm<->wrist coupling blocks from 6x6 body spatial inertias.
    const auto live = MovingWristRigidBodyMass(
        model, q_wrist, flange_jacobian, flange_rotation);
    const auto zero = MovingWristRigidBodyMass(model, Eigen::Vector2d::Zero(),
        flange_jacobian, flange_rotation);
    Eigen::Matrix<double, 9, 9> result
        = Eigen::Matrix<double, 9, 9>::Zero();
    result.topLeftCorner<7, 7>()
        = arm_mass + live.topLeftCorner<7, 7>()
          - zero.topLeftCorner<7, 7>();
    result.topRightCorner<7, 2>() = live.topRightCorner<7, 2>();
    result.bottomLeftCorner<2, 7>() = live.bottomLeftCorner<2, 7>();
    result.bottomRightCorner<2, 2>() = live.bottomRightCorner<2, 2>();
    result.bottomRightCorner<2, 2>().diagonal() += model.reflected_inertia;
    result = 0.5 * (result + result.transpose());
    Eigen::LDLT<Eigen::Matrix<double, 9, 9>> solver(result);
    if (solver.info() != Eigen::Success || !solver.isPositive()) {
        throw std::runtime_error("auto coupled 9x9 M(q) is not positive definite");
    }
    return result;
}

double SymmetricConditionNumber(const Eigen::MatrixXd& matrix)
{
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(
        0.5 * (matrix + matrix.transpose()), Eigen::EigenvaluesOnly);
    if (solver.info() != Eigen::Success || solver.eigenvalues().minCoeff() <= 0.0) {
        return std::numeric_limits<double>::infinity();
    }
    return solver.eigenvalues().maxCoeff() / solver.eigenvalues().minCoeff();
}

double AutomaticOperationalDamping(const Eigen::MatrixXd& mass,
    const Eigen::MatrixXd& jacobian, double maximum_damping = 0.20)
{
    if (maximum_damping < 0.01) {
        throw std::invalid_argument("maximum operational damping is too small");
    }
    Eigen::LDLT<Eigen::MatrixXd> mass_solver(mass);
    if (mass_solver.info() != Eigen::Success) {
        throw std::runtime_error("cannot factor auto coupled M(q)");
    }
    Eigen::MatrixXd inverse_lambda
        = jacobian * mass_solver.solve(jacobian.transpose());
    inverse_lambda = 0.5 * (inverse_lambda + inverse_lambda.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(
        inverse_lambda, Eigen::EigenvaluesOnly);
    if (solver.info() != Eigen::Success) {
        throw std::runtime_error("cannot inspect operational inertia");
    }
    const double minimum = std::max(0.0, solver.eigenvalues().minCoeff());
    const double maximum = solver.eigenvalues().maxCoeff();
    constexpr double target_condition = 200.0;
    const double required_squared
        = std::max(0.0,
            (maximum - target_condition * minimum) / (target_condition - 1.0));
    return std::clamp(std::sqrt(required_squared), 0.01, maximum_damping);
}

struct ArmSingularityStatus {
    double sigma_min {0.0};
    double condition {std::numeric_limits<double>::infinity()};
    double motion_scale {1.0};
};

ArmSingularityStatus EvaluateArmSingularity(
    const Eigen::Matrix<double, 6, 7>& jacobian,
    double characteristic_length_m, double slow_sigma,
    double critical_sigma, double minimum_motion_scale)
{
    // Translation has units of metres/radian whereas orientation has units
    // of radian/radian. Scale the angular rows by one physical arm length so
    // the singular values are meaningful and independent of unit choice.
    Eigen::Matrix<double, 6, 7> normalized = jacobian;
    normalized.bottomRows<3>() *= characteristic_length_m;
    const Eigen::Matrix<double, 6, 6> gram
        = normalized * normalized.transpose();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(
        0.5 * (gram + gram.transpose()), Eigen::EigenvaluesOnly);
    if (solver.info() != Eigen::Success) {
        return {0.0, std::numeric_limits<double>::infinity(),
            minimum_motion_scale};
    }
    const double minimum_eigenvalue
        = std::max(0.0, solver.eigenvalues().minCoeff());
    const double maximum_eigenvalue
        = std::max(0.0, solver.eigenvalues().maxCoeff());
    const double sigma_min = std::sqrt(minimum_eigenvalue);
    const double sigma_max = std::sqrt(maximum_eigenvalue);
    const double condition = sigma_max / std::max(sigma_min, 1e-9);
    const double normalized_margin = std::clamp(
        (sigma_min - critical_sigma) / (slow_sigma - critical_sigma),
        0.0, 1.0);
    // C1-continuous slowdown: it never terminates the mission and retains a
    // small command authority so the operator can reclutch and move away.
    const double smooth
        = normalized_margin * normalized_margin
          * (3.0 - 2.0 * normalized_margin);
    const double motion_scale
        = minimum_motion_scale + (1.0 - minimum_motion_scale) * smooth;
    return {sigma_min, condition, motion_scale};
}

Eigen::Vector2d SolveWristOrientationTarget(const WristModel& model,
    const Eigen::Vector2d& current_q, const Eigen::Matrix3d& desired_rotation_flange)
{
    Eigen::Vector2d target = current_q;
    const Eigen::Vector2d lower
        = model.joint_min.array() + model.joint_limit_margin_rad;
    const Eigen::Vector2d upper
        = model.joint_max.array() - model.joint_limit_margin_rad;
    for (int iteration = 0; iteration < model.ik_max_iterations; ++iteration) {
        const auto kinematics = EvaluateWrist(model, target);
        const Eigen::Vector3d error = RotationError(
            kinematics.rotation_flange, desired_rotation_flange);
        const Eigen::Matrix<double, 3, 2> jacobian
            = kinematics.jacobian_flange.bottomRows<3>();
        Eigen::Matrix2d normal = jacobian.transpose() * jacobian;
        normal.diagonal().array() += model.ik_damping * model.ik_damping;
        Eigen::Vector2d step = normal.ldlt().solve(jacobian.transpose() * error);
        if (step.norm() > model.ik_max_step_rad) {
            step *= model.ik_max_step_rad / step.norm();
        }
        target += step;
        target = target.cwiseMax(lower).cwiseMin(upper);
        if (step.norm() < 1e-5) break;
    }
    return target;
}

Eigen::Vector2d WristCoriolis(const WristModel& model,
    const Eigen::Vector2d& q, const Eigen::Vector2d& dq)
{
    constexpr double epsilon = 1e-5;
    std::array<Eigen::Matrix2d, 2> derivative;
    for (int coordinate = 0; coordinate < 2; ++coordinate) {
        Eigen::Vector2d step = Eigen::Vector2d::Zero();
        step[coordinate] = epsilon;
        derivative[coordinate]
            = (EvaluateWrist(model, q + step).mass
                  - EvaluateWrist(model, q - step).mass)
              / (2.0 * epsilon);
    }
    Eigen::Vector2d result = Eigen::Vector2d::Zero();
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            for (int k = 0; k < 2; ++k) {
                const double christoffel
                    = 0.5
                      * (derivative[k](i, j) + derivative[j](i, k)
                          - derivative[i](j, k));
                result[i] += christoffel * dq[j] * dq[k];
            }
        }
    }
    return result;
}

struct Options {
    std::string robot_sn {"Rizon4s-062232"};
    std::string shared_memory;
    std::string teleop_shared_memory;
    std::string model_config;
    std::string dry_run_csv {"/tmp/flexiv_9dof_osc_loop.csv"};
    std::string trajectory {"orientation"};
    std::string endpoint {"pivot"};
    double duration_s {20.0};
    double radius_m {0.030};
    double orientation_deg {3.0};
    double rectangle_width_m {0.060};
    double rectangle_height_m {0.040};
    double rectangle_corner_radius_m {0.010};
    int tangent_axis {0};
    std::string inertia_mode {"auto"};
    std::string wrist_execution_mode {"hybrid_position_ff"};
    double wrist_state_timeout_ms {250.0};
    double wrist_hard_timeout_ms {2000.0};
    double teleop_command_timeout_ms {500.0};
    bool real {false};
    int cpu_affinity {2};
};

Options Parse(int argc, char** argv)
{
    Options result;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto value = [&]() -> std::string {
            if (++i >= argc) {
                throw std::invalid_argument("missing value after " + arg);
            }
            return argv[i];
        };
        if (arg == "--robot-sn") result.robot_sn = value();
        else if (arg == "--wrist-shm") result.shared_memory = value();
        else if (arg == "--teleop-shm") result.teleop_shared_memory = value();
        else if (arg == "--wrist-model") result.model_config = value();
        else if (arg == "--duration-s") result.duration_s = std::stod(value());
        else if (arg == "--radius-m") result.radius_m = std::stod(value());
        else if (arg == "--orientation-deg") result.orientation_deg = std::stod(value());
        else if (arg == "--trajectory") result.trajectory = value();
        else if (arg == "--endpoint") result.endpoint = value();
        else if (arg == "--rectangle-width-m") {
            result.rectangle_width_m = std::stod(value());
        }
        else if (arg == "--rectangle-height-m") {
            result.rectangle_height_m = std::stod(value());
        }
        else if (arg == "--rectangle-corner-radius-m") {
            result.rectangle_corner_radius_m = std::stod(value());
        }
        else if (arg == "--tangent-axis") {
            const std::string axis = value();
            if (axis == "x") result.tangent_axis = 0;
            else if (axis == "z") result.tangent_axis = 2;
            else throw std::invalid_argument("--tangent-axis must be x or z");
        }
        else if (arg == "--inertia-mode") result.inertia_mode = value();
        else if (arg == "--wrist-execution-mode") {
            result.wrist_execution_mode = value();
        }
        else if (arg == "--wrist-state-timeout-ms") {
            result.wrist_state_timeout_ms = std::stod(value());
        }
        else if (arg == "--wrist-hard-timeout-ms") {
            result.wrist_hard_timeout_ms = std::stod(value());
        }
        else if (arg == "--teleop-command-timeout-ms") {
            result.teleop_command_timeout_ms = std::stod(value());
        }
        else if (arg == "--cpu-affinity") result.cpu_affinity = std::stoi(value());
        else if (arg == "--dry-run-csv") result.dry_run_csv = value();
        else if (arg == "--real-confirm") {
            result.real = value() == "RUN_9DOF_TORQUE_OSC";
            if (!result.real) throw std::invalid_argument(
                "real confirmation must equal RUN_9DOF_TORQUE_OSC");
        } else if (arg == "-h" || arg == "--help") {
            std::cout << "9-DoF periodic multi-rate torque OSC. --duration-s is one "
                         "loop period; real motion continues until Ctrl-C.\n"
                      << "Real requires --wrist-shm, --wrist-model and "
                         "--real-confirm RUN_9DOF_TORQUE_OSC\n";
            std::exit(0);
        } else throw std::invalid_argument("unknown argument: " + arg);
    }
    const double maximum_orientation_deg
        = result.trajectory == "spin" ? 60.0 : 45.0;
    if (result.duration_s < 8.0 || result.radius_m <= 0.0 || result.radius_m > 0.05
        || result.orientation_deg <= 0.0
        || result.orientation_deg > maximum_orientation_deg
        || result.wrist_state_timeout_ms < 100.0
        || result.wrist_state_timeout_ms > 1000.0
        || result.wrist_hard_timeout_ms <= result.wrist_state_timeout_ms
        || result.wrist_hard_timeout_ms > 5000.0
        || result.teleop_command_timeout_ms < 20.0
        || result.teleop_command_timeout_ms > 1000.0) {
        throw std::invalid_argument("invalid duration/radius/orientation");
    }
    if (result.trajectory != "orientation" && result.trajectory != "spin"
        && result.trajectory != "rectangle" && result.trajectory != "loop") {
        throw std::invalid_argument(
            "--trajectory must be orientation, spin, rectangle or loop");
    }
    if (result.endpoint != "tip" && result.endpoint != "pivot") {
        throw std::invalid_argument("--endpoint must be tip or pivot");
    }
    if (result.inertia_mode != "auto" && result.inertia_mode != "legacy-block") {
        throw std::invalid_argument("--inertia-mode must be auto or legacy-block");
    }
    if (result.wrist_execution_mode != "hybrid_position_ff"
        && result.wrist_execution_mode != "pure_torque") {
        throw std::invalid_argument(
            "--wrist-execution-mode must be hybrid_position_ff or pure_torque");
    }
    if (result.rectangle_width_m <= 0.0 || result.rectangle_width_m > 0.15
        || result.rectangle_height_m <= 0.0 || result.rectangle_height_m > 0.15
        || result.rectangle_corner_radius_m <= 0.0
        || 2.0 * result.rectangle_corner_radius_m
               >= std::min(result.rectangle_width_m, result.rectangle_height_m)) {
        throw std::invalid_argument("invalid rounded rectangle dimensions");
    }
    if (result.real && (result.shared_memory.empty() || result.model_config.empty())) {
        throw std::invalid_argument("real 9-DoF needs wrist shared memory and model files");
    }
    return result;
}

void LoadVector(std::istream& stream, Eigen::Vector3d& value)
{
    stream >> value.x() >> value.y() >> value.z();
}

WristModel LoadWristModel(const std::string& path)
{
    WristModel model;
    std::ifstream input(path);
    if (!input) throw std::runtime_error("cannot read wrist model file: " + path);
    std::string key;
    while (input >> key) {
        if (key == "joint1_origin") LoadVector(input, model.joint1_origin);
        else if (key == "joint1_axis") LoadVector(input, model.joint1_axis);
        else if (key == "joint2_offset") LoadVector(input, model.joint2_offset);
        else if (key == "joint2_axis") LoadVector(input, model.joint2_axis);
        else if (key == "tip_offset") LoadVector(input, model.tip_offset);
        else if (key == "probe_rotation_zero") {
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) input >> model.probe_rotation_zero(r, c);
            }
        }
        else if (key == "active_tool_mass") input >> model.active_tool_mass;
        else if (key == "active_tool_com") LoadVector(input, model.active_tool_com);
        else if (key == "active_tool_inertia") {
            double ixx, iyy, izz, ixy, ixz, iyz;
            input >> ixx >> iyy >> izz >> ixy >> ixz >> iyz;
            model.active_tool_inertia << ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz,
                izz;
        }
        else if (key == "mass1") input >> model.mass1;
        else if (key == "mass2") input >> model.mass2;
        else if (key == "com1") LoadVector(input, model.com1);
        else if (key == "com2") LoadVector(input, model.com2);
        else if (key == "inertia1") {
            for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) input >> model.inertia1(r, c);
        } else if (key == "inertia2") {
            for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) input >> model.inertia2(r, c);
        } else if (key == "reflected_inertia") {
            input >> model.reflected_inertia[0] >> model.reflected_inertia[1];
        } else if (key == "viscous_friction") {
            input >> model.viscous_friction[0] >> model.viscous_friction[1];
        } else if (key == "coulomb_friction") {
            input >> model.coulomb_friction[0] >> model.coulomb_friction[1];
        } else if (key == "torque_bias") {
            input >> model.torque_bias[0] >> model.torque_bias[1];
        }
        else if (key == "rigid_body_scale") input >> model.rigid_body_scale;
        else if (key == "priority_inertia_scale") input >> model.priority_inertia_scale;
        else if (key == "wrist_priority_gain") input >> model.wrist_priority_gain;
        else if (key == "wrist_tracking_kp") {
            input >> model.wrist_tracking_kp[0] >> model.wrist_tracking_kp[1];
        }
        else if (key == "wrist_tracking_kd") {
            input >> model.wrist_tracking_kd[0] >> model.wrist_tracking_kd[1];
        }
        else if (key == "wrist_target_filter_hz") {
            input >> model.wrist_target_filter_hz;
        }
        else if (key == "wrist_target_max_velocity_rad_s") {
            input >> model.wrist_target_max_velocity_rad_s[0]
                  >> model.wrist_target_max_velocity_rad_s[1];
        }
        else if (key == "wrist_target_max_acceleration_rad_s2") {
            input >> model.wrist_target_max_acceleration_rad_s2[0]
                  >> model.wrist_target_max_acceleration_rad_s2[1];
        }
        else if (key == "wrist_state_velocity_filter_hz") {
            input >> model.wrist_state_velocity_filter_hz;
        }
        else if (key == "torque_startup_ramp_s") {
            input >> model.torque_startup_ramp_s;
        }
        else if (key == "stale_recovery_ramp_s") {
            input >> model.stale_recovery_ramp_s;
        }
        else if (key == "wrist_rectangle_excursion_rad") {
            input >> model.wrist_rectangle_excursion_rad[0]
                  >> model.wrist_rectangle_excursion_rad[1];
        }
        else if (key == "ik_damping") input >> model.ik_damping;
        else if (key == "ik_max_step_rad") input >> model.ik_max_step_rad;
        else if (key == "ik_max_iterations") input >> model.ik_max_iterations;
        else if (key == "joint_limit_margin_rad") {
            input >> model.joint_limit_margin_rad;
        }
        else if (key == "joint_min") {
            input >> model.joint_min[0] >> model.joint_min[1];
        }
        else if (key == "joint_max") {
            input >> model.joint_max[0] >> model.joint_max[1];
        }
        else if (key == "task_translation_kp") input >> model.task_translation_kp;
        else if (key == "task_translation_kd") input >> model.task_translation_kd;
        else if (key == "task_rotation_kp") input >> model.task_rotation_kp;
        else if (key == "task_rotation_kd") input >> model.task_rotation_kd;
        else if (key == "nullspace_kp") input >> model.nullspace_kp;
        else if (key == "nullspace_kd") input >> model.nullspace_kd;
        else if (key == "arm_torque_limit") {
            for (int index = 0; index < 7; ++index) input >> model.arm_torque_limit[index];
        }
        else if (key == "arm_torque_slew") {
            for (int index = 0; index < 7; ++index) input >> model.arm_torque_slew[index];
        }
        else if (key == "wrist_torque_slew") {
            input >> model.wrist_torque_slew[0] >> model.wrist_torque_slew[1];
        }
        else if (key == "teleop_target_filter_hz") {
            input >> model.teleop_target_filter_hz;
        }
        else if (key == "teleop_max_linear_velocity_m_s") {
            input >> model.teleop_max_linear_velocity_m_s;
        }
        else if (key == "teleop_max_linear_acceleration_m_s2") {
            input >> model.teleop_max_linear_acceleration_m_s2;
        }
        else if (key == "teleop_max_angular_velocity_rad_s") {
            input >> model.teleop_max_angular_velocity_rad_s;
        }
        else if (key == "teleop_max_angular_acceleration_rad_s2") {
            input >> model.teleop_max_angular_acceleration_rad_s2;
        }
        else if (key == "teleop_arm_translation_kp") {
            input >> model.teleop_arm_translation_kp;
        }
        else if (key == "teleop_arm_translation_kd") {
            input >> model.teleop_arm_translation_kd;
        }
        else if (key == "teleop_arm_rotation_kp") {
            input >> model.teleop_arm_rotation_kp;
        }
        else if (key == "teleop_arm_rotation_kd") {
            input >> model.teleop_arm_rotation_kd;
        }
        else if (key == "mode2_arm_rotation_kp") {
            input >> model.mode2_arm_rotation_kp;
        }
        else if (key == "mode2_arm_rotation_kd") {
            input >> model.mode2_arm_rotation_kd;
        }
        else if (key == "mode2_arm_max_angular_velocity_rad_s") {
            input >> model.mode2_arm_max_angular_velocity_rad_s;
        }
        else if (key == "mode2_arm_max_angular_acceleration_rad_s2") {
            input >> model.mode2_arm_max_angular_acceleration_rad_s2;
        }
        else if (key == "mode2_posture_reference_rate_per_s") {
            input >> model.mode2_posture_reference_rate_per_s;
        }
        else if (key == "teleop_wrist_target_filter_hz") {
            input >> model.teleop_wrist_target_filter_hz;
        }
        else if (key == "teleop_wrist_target_max_velocity_rad_s") {
            input >> model.teleop_wrist_target_max_velocity_rad_s[0]
                  >> model.teleop_wrist_target_max_velocity_rad_s[1];
        }
        else if (key == "teleop_wrist_target_max_acceleration_rad_s2") {
            input >> model.teleop_wrist_target_max_acceleration_rad_s2[0]
                  >> model.teleop_wrist_target_max_acceleration_rad_s2[1];
        }
        else if (key == "teleop_max_operational_damping") {
            input >> model.teleop_max_operational_damping;
        }
        else if (key == "teleop_singularity_characteristic_length_m") {
            input >> model.teleop_singularity_characteristic_length_m;
        }
        else if (key == "teleop_singularity_slow_sigma") {
            input >> model.teleop_singularity_slow_sigma;
        }
        else if (key == "teleop_singularity_critical_sigma") {
            input >> model.teleop_singularity_critical_sigma;
        }
        else if (key == "teleop_singularity_min_motion_scale") {
            input >> model.teleop_singularity_min_motion_scale;
        }
        else if (key == "teleop_posture_reference_rate_per_s") {
            input >> model.teleop_posture_reference_rate_per_s;
        }
        else if (key == "clutch_hold_natural_frequency_hz") {
            input >> model.clutch_hold_natural_frequency_hz;
        }
        else if (key == "clutch_hold_damping_ratio") {
            input >> model.clutch_hold_damping_ratio;
        }
        else if (key == "dynamic_wrist_gravity_compensation") {
            int enabled = 0;
            input >> enabled;
            if (enabled != 0 && enabled != 1) {
                throw std::runtime_error(
                    "dynamic_wrist_gravity_compensation must be 0 or 1");
            }
            model.dynamic_wrist_gravity_compensation = enabled != 0;
        }
        else if (key == "dynamic_gravity_filter_hz") {
            input >> model.dynamic_gravity_filter_hz;
        }
        else if (key == "mode3_position_kp") input >> model.mode3_position_kp;
        else if (key == "mode3_position_kd") input >> model.mode3_position_kd;
        else if (key == "mode3_arm_rotation_kp") {
            input >> model.mode3_arm_rotation_kp;
        }
        else if (key == "mode3_arm_rotation_kd") {
            input >> model.mode3_arm_rotation_kd;
        }
        else if (key == "mode3_arm_max_angular_velocity_rad_s") {
            input >> model.mode3_arm_max_angular_velocity_rad_s;
        }
        else if (key == "mode3_arm_max_angular_acceleration_rad_s2") {
            input >> model.mode3_arm_max_angular_acceleration_rad_s2;
        }
        else if (key == "mode3_contact_enable_threshold_n") {
            input >> model.mode3_contact_enable_threshold_n;
        }
        else if (key == "mode3_contact_release_threshold_n") {
            input >> model.mode3_contact_release_threshold_n;
        }
        else if (key == "mode3_contact_release_delay_s") {
            input >> model.mode3_contact_release_delay_s;
        }
        else if (key == "mode3_force_tolerance_n") {
            input >> model.mode3_force_tolerance_n;
        }
        else if (key == "mode3_force_full_position_error_m") {
            input >> model.mode3_force_full_position_error_m;
        }
        else if (key == "mode3_force_disable_position_error_m") {
            input >> model.mode3_force_disable_position_error_m;
        }
        else if (key == "mode3_force_kp") input >> model.mode3_force_kp;
        else if (key == "mode3_force_ki_per_s") input >> model.mode3_force_ki_per_s;
        else if (key == "mode3_force_damping_n_s_m") {
            input >> model.mode3_force_damping_n_s_m;
        }
        else if (key == "mode3_force_integral_limit_n") {
            input >> model.mode3_force_integral_limit_n;
        }
        else if (key == "mode3_force_command_limit_n") {
            input >> model.mode3_force_command_limit_n;
        }
        else throw std::runtime_error("unknown wrist model key: " + key);
    }
    const auto tool_spatial = SpatialInertiaAtOrigin(model.active_tool_mass,
        model.active_tool_com, model.active_tool_inertia);
    Eigen::LDLT<Eigen::Matrix<double, 6, 6>> tool_solver(tool_spatial);
    if (model.active_tool_mass <= 0.0 || tool_solver.info() != Eigen::Success
        || !tool_solver.isPositive()
        || std::abs(model.wrist_priority_gain - 1.0) > 1e-9
        || (model.wrist_tracking_kp.array() <= 0.0).any()
        || (model.wrist_tracking_kd.array() <= 0.0).any()
        || model.wrist_target_filter_hz <= 0.0
        || model.wrist_state_velocity_filter_hz <= 0.0
        || (model.wrist_target_max_velocity_rad_s.array() <= 0.0).any()
        || (model.wrist_target_max_acceleration_rad_s2.array() <= 0.0).any()
        || model.torque_startup_ramp_s <= 0.0
        || model.stale_recovery_ramp_s <= 0.0
        || (model.wrist_rectangle_excursion_rad.array() <= 0.0).any()
        || (model.wrist_rectangle_excursion_rad
                   - (model.joint_max - model.joint_min) * 0.5)
                   .maxCoeff()
               >= 0.0
        || model.ik_damping <= 0.0 || model.ik_max_step_rad <= 0.0
        || model.ik_max_iterations < 1 || model.ik_max_iterations > 100
        || model.joint_limit_margin_rad < 0.0
        || model.task_translation_kp <= 0.0 || model.task_translation_kd <= 0.0
        || model.task_rotation_kp <= 0.0 || model.task_rotation_kd <= 0.0
        || model.nullspace_kp <= 0.0 || model.nullspace_kd <= 0.0
        || (model.arm_torque_limit.array() <= 0.0).any()
        || (model.arm_torque_slew.array() <= 0.0).any()
        || (model.wrist_torque_slew.array() <= 0.0).any()
        || model.teleop_target_filter_hz <= 0.0
        || model.teleop_max_linear_velocity_m_s <= 0.0
        || model.teleop_max_linear_acceleration_m_s2 <= 0.0
        || model.teleop_max_angular_velocity_rad_s <= 0.0
        || model.teleop_max_angular_acceleration_rad_s2 <= 0.0
        || model.teleop_arm_translation_kp <= 0.0
        || model.teleop_arm_translation_kd <= 0.0
        || model.teleop_arm_rotation_kp <= 0.0
        || model.teleop_arm_rotation_kd <= 0.0
        || model.mode2_arm_rotation_kp <= 0.0
        || model.mode2_arm_rotation_kd <= 0.0
        || model.mode2_arm_max_angular_velocity_rad_s <= 0.0
        || model.mode2_arm_max_angular_acceleration_rad_s2 <= 0.0
        || model.mode2_posture_reference_rate_per_s < 0.0
        || model.mode2_posture_reference_rate_per_s > 10.0
        || model.teleop_wrist_target_filter_hz <= 0.0
        || (model.teleop_wrist_target_max_velocity_rad_s.array() <= 0.0).any()
        || (model.teleop_wrist_target_max_acceleration_rad_s2.array() <= 0.0).any()
        || model.teleop_max_operational_damping < 0.01
        || model.teleop_max_operational_damping > 2.0
        || model.teleop_singularity_characteristic_length_m <= 0.0
        || model.teleop_singularity_critical_sigma <= 0.0
        || model.teleop_singularity_slow_sigma
               <= model.teleop_singularity_critical_sigma
        || model.teleop_singularity_min_motion_scale <= 0.0
        || model.teleop_singularity_min_motion_scale > 1.0
        || model.teleop_posture_reference_rate_per_s <= 0.0
        || model.teleop_posture_reference_rate_per_s > 10.0
        || model.clutch_hold_natural_frequency_hz <= 0.0
        || model.clutch_hold_damping_ratio < 1.0
        || model.dynamic_gravity_filter_hz <= 0.0
        || model.mode3_position_kp <= 0.0
        || model.mode3_position_kd <= 0.0
        || model.mode3_arm_rotation_kp <= 0.0
        || model.mode3_arm_rotation_kd <= 0.0
        || model.mode3_arm_max_angular_velocity_rad_s <= 0.0
        || model.mode3_arm_max_angular_acceleration_rad_s2 <= 0.0
        || model.mode3_contact_enable_threshold_n <= 0.0
        || model.mode3_contact_release_threshold_n <= 0.0
        || model.mode3_contact_release_threshold_n
               >= model.mode3_contact_enable_threshold_n
        || model.mode3_contact_release_delay_s <= 0.0
        || model.mode3_force_tolerance_n <= 0.0
        || model.mode3_force_full_position_error_m < 0.0
        || model.mode3_force_disable_position_error_m
               <= model.mode3_force_full_position_error_m
        || model.mode3_force_kp < 0.0 || model.mode3_force_ki_per_s < 0.0
        || model.mode3_force_damping_n_s_m < 0.0
        || model.mode3_force_integral_limit_n <= 0.0
        || model.mode3_force_command_limit_n <= 0.0
        || (model.joint_max - model.joint_min).minCoeff()
               <= 2.0 * model.joint_limit_margin_rad) {
        throw std::runtime_error("invalid wrist IK configuration");
    }
    return model;
}

Eigen::VectorXd InertiaShapedJointHold(const Eigen::MatrixXd& mass,
    const Eigen::VectorXd& position_error,
    const Eigen::VectorXd& joint_velocity, double natural_frequency_hz,
    double damping_ratio)
{
    if (mass.rows() != mass.cols() || mass.rows() != position_error.size()
        || position_error.size() != joint_velocity.size()
        || natural_frequency_hz <= 0.0 || damping_ratio < 1.0) {
        throw std::invalid_argument("invalid inertia-shaped hold parameters");
    }
    // Express the hold as a desired joint acceleration and then map it
    // through the live Flexiv mass matrix. This gives every joint the same
    // closed-loop return speed even though their physical inertias differ.
    const double omega = 2.0 * kPi * natural_frequency_hz;
    const Eigen::VectorXd desired_acceleration
        = omega * omega * position_error
          - 2.0 * damping_ratio * omega * joint_velocity;
    return mass * desired_acceleration;
}

std::array<double, 7> PoseArray(
    const Eigen::Vector3d& position, const Eigen::Matrix3d& rotation)
{
    const Eigen::Quaterniond quaternion(rotation);
    return {position.x(), position.y(), position.z(), quaternion.w(),
        quaternion.x(), quaternion.y(), quaternion.z()};
}

struct Controller {
    rdk::Robot& robot;
    rdk::Model& arm_model;
    WristSharedMemory& wrist_io;
    WristModel wrist_model;
    Options options;
    Eigen::VectorXd q_reference;
    Eigen::VectorXd previous_torque;
    Eigen::Vector3d position_origin;
    Eigen::Matrix3d rotation_origin;
    Eigen::Matrix3d flange_rotation_reference;
    std::chrono::steady_clock::time_point started;
    std::ofstream log;
    std::uint64_t cycle_count {0};
    bool first_cycle {true};
    bool stale_hold_active {false};
    Eigen::VectorXd stale_hold_arm_q {Eigen::VectorXd::Zero(7)};
    std::chrono::steady_clock::time_point stale_hold_started {};
    Eigen::Vector2d shaped_wrist_target {Eigen::Vector2d::Zero()};
    Eigen::Vector2d shaped_wrist_velocity {Eigen::Vector2d::Zero()};
    bool wrist_target_initialized {false};
    Eigen::Vector2d filtered_wrist_velocity {Eigen::Vector2d::Zero()};
    bool wrist_velocity_initialized {false};
    Eigen::Vector2d previous_wrist_actuator_torque {Eigen::Vector2d::Zero()};
    Eigen::Matrix<double, 7, 1> last_arm_wrist_gravity_delta {
        Eigen::Matrix<double, 7, 1>::Zero()};
    WristGravityResult filtered_dynamic_gravity {};
    bool dynamic_gravity_filter_initialized {false};
    std::chrono::steady_clock::time_point torque_ramp_started {};
    double torque_ramp_duration_s {0.0};
    TeleopSharedMemory* teleop_io {nullptr};
    TeleopCommand teleop_command {};
    TeleopControlMode teleop_mode {TeleopControlMode::Hold};
    bool teleop_enabled {false};
    bool teleop_command_fresh {false};
    bool mode3_force_active {false};
    Eigen::Vector3d teleop_position {Eigen::Vector3d::Zero()};
    Eigen::Vector3d teleop_linear_velocity {Eigen::Vector3d::Zero()};
    Eigen::Matrix3d teleop_rotation {Eigen::Matrix3d::Identity()};
    Eigen::Vector3d teleop_angular_velocity {Eigen::Vector3d::Zero()};
    Eigen::Matrix3d teleop_flange_rotation {Eigen::Matrix3d::Identity()};
    Eigen::Vector3d teleop_flange_angular_velocity {Eigen::Vector3d::Zero()};
    Eigen::Vector3d teleop_anchor_position {Eigen::Vector3d::Zero()};
    Eigen::Matrix3d teleop_probe_anchor_rotation {Eigen::Matrix3d::Identity()};
    Eigen::Matrix3d teleop_flange_anchor_rotation {Eigen::Matrix3d::Identity()};
    Eigen::Matrix3d teleop_wrist_anchor_rotation {Eigen::Matrix3d::Identity()};
    Eigen::Vector2d teleop_wrist_hold_q {Eigen::Vector2d::Zero()};
    Eigen::Vector3d force_bias_world {Eigen::Vector3d::Zero()};
    double mode3_force_integral_n {0.0};
    std::uint32_t mode3_contact_loss_cycles {0};
    double mode3_force_position_scale {0.0};
    bool mode3_position_recovery_active {false};
    double last_probe_force_z_n {0.0};
    double last_force_error_n {0.0};
    double last_position_error_m {0.0};
    double last_orientation_error_rad {0.0};
    Eigen::Vector3d last_arm_orientation_error_world {Eigen::Vector3d::Zero()};
    double last_arm_singularity_sigma_min {0.0};
    double last_arm_motion_scale {1.0};
    Eigen::VectorXd teleop_safe_arm_q {Eigen::VectorXd::Zero(7)};

    static double SmoothRamp(double value)
    {
        const double u = std::clamp(value, 0.0, 1.0);
        return u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
    }

    WristGravityResult FilterDynamicGravity(
        const WristGravityResult& unfiltered)
    {
        if (!dynamic_gravity_filter_initialized) {
            filtered_dynamic_gravity = unfiltered;
            dynamic_gravity_filter_initialized = true;
            return filtered_dynamic_gravity;
        }
        constexpr double dt = 0.001;
        const double alpha = 1.0 - std::exp(
            -2.0 * kPi * wrist_model.dynamic_gravity_filter_hz * dt);
        filtered_dynamic_gravity.arm_delta += alpha
            * (unfiltered.arm_delta - filtered_dynamic_gravity.arm_delta);
        filtered_dynamic_gravity.wrist += alpha
            * (unfiltered.wrist - filtered_dynamic_gravity.wrist);
        return filtered_dynamic_gravity;
    }

    void ResetTeleopReference(const Eigen::Vector3d& task_position,
        const Eigen::Matrix3d& probe_rotation,
        const Eigen::Matrix3d& flange_rotation,
        const Eigen::Matrix3d& wrist_rotation,
        const Eigen::Vector2d& q_wrist, const Eigen::VectorXd& q_arm,
        TeleopControlMode mode, bool enabled,
        const std::chrono::steady_clock::time_point& now)
    {
        teleop_mode = mode;
        teleop_enabled = enabled;
        teleop_position = task_position;
        teleop_linear_velocity.setZero();
        teleop_rotation = probe_rotation;
        teleop_angular_velocity.setZero();
        teleop_flange_rotation = flange_rotation;
        teleop_flange_angular_velocity.setZero();
        teleop_anchor_position = task_position;
        teleop_probe_anchor_rotation = probe_rotation;
        teleop_flange_anchor_rotation = flange_rotation;
        teleop_wrist_anchor_rotation = wrist_rotation;
        teleop_wrist_hold_q = q_wrist;
        q_reference.head<7>() = q_arm;
        q_reference.tail<2>() = q_wrist;
        // Every teleoperation edge defines a new stationary operating point.
        // Capture the redundant seven-axis posture together with the Cartesian
        // anchors. Reusing a startup/previous-mode null-space reference made
        // the arm reconfigure itself even when the haptic delta was zero.
        teleop_safe_arm_q = q_arm;
        wrist_target_initialized = false;
        mode3_force_integral_n = 0.0;
        mode3_contact_loss_cycles = 0;
        mode3_force_position_scale = 0.0;
        mode3_position_recovery_active = false;
        mode3_force_active = false;
        torque_ramp_duration_s = wrist_model.stale_recovery_ramp_s;
        // The measured Cartesian and redundant-joint references above make a
        // clutch edge torque-continuous. Do not fade OSC support from zero on
        // engagement: that old one-second gap removed Cartesian stiffness and
        // allowed any payload/gravity residual to make the arm sag. The final
        // actuator torque-rate limiter still prevents a command step. Startup
        // and stale-state recovery retain their separate ramps.
        torque_ramp_started
            = now - std::chrono::duration_cast<
                        std::chrono::steady_clock::duration>(
                        std::chrono::duration<double>(torque_ramp_duration_s));
    }

    void ShapeTeleopTarget(const Eigen::Vector3d& requested_position,
        const Eigen::Matrix3d& requested_rotation,
        const Eigen::Vector3d& requested_linear_velocity,
        const Eigen::Vector3d& requested_angular_velocity,
        double arm_motion_scale)
    {
        constexpr double dt = 0.001;
        const double omega = 2.0 * kPi * wrist_model.teleop_target_filter_hz;
        const Eigen::Vector3d position_error
            = requested_position - teleop_position;
        // A second-order shaper normally needs time to brake. For direct
        // teleoperation that makes a quick right command continue left because
        // velocity from the previous target survives. Cancel only genuinely
        // opposing stored velocity before computing the next acceleration.
        if (teleop_linear_velocity.dot(position_error) < 0.0) {
            teleop_linear_velocity.setZero();
        }
        Eigen::Vector3d linear_acceleration
            = omega * omega * position_error
              + 2.0 * omega
                    * (requested_linear_velocity - teleop_linear_velocity);
        const double maximum_linear_acceleration
            = arm_motion_scale
              * wrist_model.teleop_max_linear_acceleration_m_s2;
        if (linear_acceleration.norm() > maximum_linear_acceleration) {
            linear_acceleration *= maximum_linear_acceleration
                                   / linear_acceleration.norm();
        }
        teleop_linear_velocity += dt * linear_acceleration;
        const double maximum_linear_velocity
            = arm_motion_scale * wrist_model.teleop_max_linear_velocity_m_s;
        if (teleop_linear_velocity.norm() > maximum_linear_velocity) {
            teleop_linear_velocity *= maximum_linear_velocity
                                      / teleop_linear_velocity.norm();
        }
        teleop_position += dt * teleop_linear_velocity;

        const Eigen::Vector3d rotation_error
            = RotationError(teleop_rotation, requested_rotation);
        if (teleop_angular_velocity.dot(rotation_error) < 0.0) {
            teleop_angular_velocity.setZero();
        }
        Eigen::Vector3d angular_acceleration
            = omega * omega * rotation_error
              + 2.0 * omega
                    * (requested_angular_velocity - teleop_angular_velocity);
        const double maximum_angular_acceleration
            = arm_motion_scale
              * wrist_model.teleop_max_angular_acceleration_rad_s2;
        if (angular_acceleration.norm() > maximum_angular_acceleration) {
            angular_acceleration *= maximum_angular_acceleration
                                    / angular_acceleration.norm();
        }
        teleop_angular_velocity += dt * angular_acceleration;
        const double maximum_angular_velocity
            = arm_motion_scale * wrist_model.teleop_max_angular_velocity_rad_s;
        if (teleop_angular_velocity.norm() > maximum_angular_velocity) {
            teleop_angular_velocity *= maximum_angular_velocity
                                       / teleop_angular_velocity.norm();
        }
        const Eigen::Vector3d rotation_step = dt * teleop_angular_velocity;
        const double angle = rotation_step.norm();
        if (angle > 1e-12) {
            teleop_rotation
                = Eigen::AngleAxisd(angle, rotation_step / angle).toRotationMatrix()
                  * teleop_rotation;
        }
    }

    const Eigen::Matrix3d& ShapeArmFlangeTarget(
        const Eigen::Matrix3d& requested_flange_rotation,
        double arm_motion_scale)
    {
        constexpr double dt = 0.001;
        const double omega = 2.0 * kPi * wrist_model.teleop_target_filter_hz;
        const Eigen::Vector3d error = RotationError(
            teleop_flange_rotation, requested_flange_rotation);
        if (teleop_flange_angular_velocity.dot(error) < 0.0) {
            teleop_flange_angular_velocity.setZero();
        }
        Eigen::Vector3d acceleration
            = omega * omega * error
              - 2.0 * omega * teleop_flange_angular_velocity;
        double configured_maximum_acceleration
            = wrist_model.teleop_max_angular_acceleration_rad_s2;
        if (teleop_mode == TeleopControlMode::ArmWrist9Dof) {
            configured_maximum_acceleration
                = wrist_model.mode2_arm_max_angular_acceleration_rad_s2;
        } else if (teleop_mode == TeleopControlMode::OrientationForce) {
            configured_maximum_acceleration
                = wrist_model.mode3_arm_max_angular_acceleration_rad_s2;
        }
        const double maximum_acceleration
            = arm_motion_scale * configured_maximum_acceleration;
        if (acceleration.norm() > maximum_acceleration) {
            acceleration *= maximum_acceleration / acceleration.norm();
        }
        teleop_flange_angular_velocity += dt * acceleration;
        double configured_maximum_velocity
            = wrist_model.teleop_max_angular_velocity_rad_s;
        if (teleop_mode == TeleopControlMode::ArmWrist9Dof) {
            configured_maximum_velocity
                = wrist_model.mode2_arm_max_angular_velocity_rad_s;
        } else if (teleop_mode == TeleopControlMode::OrientationForce) {
            configured_maximum_velocity
                = wrist_model.mode3_arm_max_angular_velocity_rad_s;
        }
        const double maximum_velocity
            = arm_motion_scale * configured_maximum_velocity;
        if (teleop_flange_angular_velocity.norm() > maximum_velocity) {
            teleop_flange_angular_velocity
                *= maximum_velocity / teleop_flange_angular_velocity.norm();
        }
        const Eigen::Vector3d step = dt * teleop_flange_angular_velocity;
        const double angle = step.norm();
        if (angle > 1e-12) {
            teleop_flange_rotation
                = Eigen::AngleAxisd(angle, step / angle).toRotationMatrix()
                  * teleop_flange_rotation;
        }
        return teleop_flange_rotation;
    }

    Eigen::Vector2d ShapeWristTarget(
        const Eigen::Vector2d& raw_target, const Eigen::Vector2d& measured_q)
    {
        if (!wrist_target_initialized) {
            shaped_wrist_target = measured_q;
            shaped_wrist_velocity.setZero();
            wrist_target_initialized = true;
        }
        constexpr double dt = 0.001;
        const bool teleoperation = teleop_io != nullptr;
        const double filter_hz
            = teleoperation ? wrist_model.teleop_wrist_target_filter_hz
                            : wrist_model.wrist_target_filter_hz;
        const Eigen::Vector2d& maximum_velocity
            = teleoperation
                  ? wrist_model.teleop_wrist_target_max_velocity_rad_s
                  : wrist_model.wrist_target_max_velocity_rad_s;
        const Eigen::Vector2d& maximum_acceleration
            = teleoperation
                  ? wrist_model.teleop_wrist_target_max_acceleration_rad_s2
                  : wrist_model.wrist_target_max_acceleration_rad_s2;
        const double omega = 2.0 * kPi * filter_hz;
        const Eigen::Vector2d target_error
            = raw_target - shaped_wrist_target;
        for (int axis = 0; axis < 2; ++axis) {
            if (shaped_wrist_velocity[axis] * target_error[axis] < 0.0) {
                shaped_wrist_velocity[axis] = 0.0;
            }
        }
        Eigen::Vector2d acceleration
            = omega * omega * target_error
              - 2.0 * omega * shaped_wrist_velocity;
        acceleration = acceleration
                           .cwiseMin(maximum_acceleration)
                           .cwiseMax(-maximum_acceleration);
        shaped_wrist_velocity += dt * acceleration;
        shaped_wrist_velocity
            = shaped_wrist_velocity
                  .cwiseMin(maximum_velocity)
                  .cwiseMax(-maximum_velocity);
        shaped_wrist_target += dt * shaped_wrist_velocity;
        const Eigen::Vector2d lower
            = wrist_model.joint_min.array() + wrist_model.joint_limit_margin_rad;
        const Eigen::Vector2d upper
            = wrist_model.joint_max.array() - wrist_model.joint_limit_margin_rad;
        shaped_wrist_target = shaped_wrist_target.cwiseMax(lower).cwiseMin(upper);
        return shaped_wrist_target;
    }

    void Step()
    {
        try {
            if (robot.fault() || !robot.operational()) throw std::runtime_error(
                "robot fault/non-operational during RT loop");
            const auto now = std::chrono::steady_clock::now();
            const auto& states = robot.states();
            const Eigen::Map<const Eigen::VectorXd> qa(states.q.data(), states.q.size());
            const Eigen::Map<const Eigen::VectorXd> dqa(states.dtheta.data(), states.dtheta.size());
            arm_model.Update(states.q, states.dtheta);
            const Eigen::VectorXd arm_coriolis = arm_model.c();
            std::array<double, 2> qw {}, dqw {}, measured_torque {};
            std::uint64_t wrist_age_ns = 0;
            const bool wrist_state_valid
                = wrist_io.ReadState(qw, dqw, measured_torque, wrist_age_ns);
            const double wrist_age_ms = wrist_state_valid
                                            ? static_cast<double>(wrist_age_ns) / 1e6
                                            : options.wrist_state_timeout_ms + 1.0;
            if (!wrist_state_valid
                || wrist_age_ms > options.wrist_state_timeout_ms) {
                if (!stale_hold_active) {
                    stale_hold_active = true;
                    stale_hold_started = now;
                    stale_hold_arm_q = qa;
                    std::cerr << "RT_WARNING wrist state transiently stale; "
                                 "holding Flexiv and commanding wrist 0 Nm"
                              << std::endl;
                }
                const double stale_duration_ms
                    = std::chrono::duration<double, std::milli>(
                          now - stale_hold_started)
                          .count();
                if ((wrist_state_valid
                        && wrist_age_ms > options.wrist_hard_timeout_ms)
                    || stale_duration_ms > options.wrist_hard_timeout_ms) {
                    throw std::runtime_error(
                        "wrist state continuously stale beyond hard timeout");
                }
                const Eigen::VectorXd hold_request
                    = arm_coriolis + last_arm_wrist_gravity_delta
                      + InertiaShapedJointHold(arm_model.M(),
                            stale_hold_arm_q - qa, dqa,
                            wrist_model.clutch_hold_natural_frequency_hz,
                            wrist_model.clutch_hold_damping_ratio);
                Eigen::VectorXd arm_absolute_limit(7), arm_rate_limit(7);
                arm_absolute_limit << 20.0, 20.0, 15.0, 15.0, 8.0, 8.0, 5.0;
                arm_rate_limit << 200.0, 200.0, 150.0, 150.0, 80.0, 80.0, 50.0;
                const Eigen::VectorXd arm_torque = RateAndMagnitudeLimit(
                    hold_request, previous_torque.head(7), arm_absolute_limit,
                    arm_rate_limit);
                previous_torque.head(7) = arm_torque;
                previous_torque.tail(2).setZero();
                previous_wrist_actuator_torque.setZero();
                robot.StreamJointTorque(std::vector<double>(
                    arm_torque.data(), arm_torque.data() + arm_torque.size()),
                    true, true);
                wrist_io.WriteTorque({0.0, 0.0});
                return;
            }
            if (stale_hold_active) {
                stale_hold_active = false;
                wrist_target_initialized = false;
                wrist_velocity_initialized = false;
                torque_ramp_started = now;
                torque_ramp_duration_s = wrist_model.stale_recovery_ramp_s;
                std::cerr << "RT_INFO wrist state recovered; resuming 9-DoF OSC"
                          << std::endl;
            }
            const Eigen::Vector2d q_wrist(qw[0], qw[1]);
            const Eigen::Vector2d raw_wrist_velocity(dqw[0], dqw[1]);
            if (!wrist_velocity_initialized) {
                filtered_wrist_velocity = raw_wrist_velocity;
                wrist_velocity_initialized = true;
            } else {
                constexpr double dt = 0.001;
                const double alpha = 1.0 - std::exp(
                    -2.0 * kPi * wrist_model.wrist_state_velocity_filter_hz * dt);
                filtered_wrist_velocity += alpha
                    * (raw_wrist_velocity - filtered_wrist_velocity);
            }
            const Eigen::Vector2d dq_wrist = filtered_wrist_velocity;
            const Eigen::MatrixXd arm_mass = arm_model.M();
            const Eigen::MatrixXd flange_jacobian = arm_model.J("flange");
            const WristKinematics wrist = EvaluateWrist(wrist_model, q_wrist);
            const Eigen::Vector3d flange_position = PosePosition(states.flange_pose);
            const Eigen::Matrix3d flange_rotation = PoseRotation(states.flange_pose);
            WristGravityResult unfiltered_dynamic_gravity = DynamicWristGravity(
                wrist_model, q_wrist, flange_jacobian, flange_rotation);
            if (!wrist_model.dynamic_wrist_gravity_compensation) {
                // Retain complete wrist-axis compensation even when the
                // arm-side live-minus-zero correction is disabled for an
                // explicit diagnostic comparison.
                unfiltered_dynamic_gravity.arm_delta.setZero();
            }
            const WristGravityResult dynamic_gravity
                = FilterDynamicGravity(unfiltered_dynamic_gravity);
            last_arm_wrist_gravity_delta = dynamic_gravity.arm_delta;
            // For the comparison demo, control the intersecting wrist-axis
            // pivot as the positional task and the probe as the orientation
            // task. Wrist rotation then does not create a fictitious 50--120
            // mm translation error that the seven arm joints must cancel.
            // "tip" remains available for experiments that truly require the
            // physical probe point to follow the Cartesian path.
            const bool pivot_task
                = teleop_io == nullptr && options.endpoint == "pivot";
            const Eigen::Vector3d task_offset_flange
                = pivot_task ? wrist_model.joint1_origin
                             : wrist.position_flange;
            const Eigen::Vector3d task_offset_world
                = flange_rotation * task_offset_flange;
            const Eigen::Vector3d task_position
                = flange_position + task_offset_world;
            const Eigen::Matrix3d probe_rotation
                = flange_rotation * wrist.rotation_flange;

            Eigen::Matrix<double, 6, 9> jacobian;
            jacobian.leftCols<7>().topRows<3>()
                = flange_jacobian.topRows<3>()
                  - Skew(task_offset_world) * flange_jacobian.bottomRows<3>();
            jacobian.leftCols<7>().bottomRows<3>()
                = flange_jacobian.bottomRows<3>();
            if (pivot_task) {
                jacobian.rightCols<2>().topRows<3>().setZero();
            } else {
                jacobian.rightCols<2>().topRows<3>()
                    = flange_rotation * wrist.jacobian_flange.topRows<3>();
            }
            jacobian.rightCols<2>().bottomRows<3>()
                = flange_rotation * wrist.jacobian_flange.bottomRows<3>();

            Eigen::Matrix<double, 9, 9> mass;
            if (options.inertia_mode == "auto") {
                mass = AutoCompositeMass(wrist_model, arm_mass,
                    flange_jacobian, flange_rotation, q_wrist);
            } else {
                // Diagnostic compatibility with the original implementation.
                // It omits arm/wrist inertial coupling and should not be used
                // for the real 9-DoF demo unless comparing logs.
                mass.setZero();
                mass.topLeftCorner<7, 7>() = arm_mass;
                mass.bottomRightCorner<2, 2>() = wrist.mass;
            }
            Eigen::VectorXd q(9), dq(9);
            q << qa, q_wrist;
            dq << dqa, dq_wrist;

            if (first_cycle) {
                started = now;
                first_cycle = false;
            }
            const double elapsed
                = std::chrono::duration<double>(now - started).count();
            Eigen::VectorXd error = Eigen::VectorXd::Zero(6);
            Eigen::VectorXd osc_torque = Eigen::VectorXd::Zero(9);
            Eigen::MatrixXd task_lambda = Eigen::MatrixXd::Identity(6, 6);
            Eigen::Vector2d raw_wrist_target = q_reference.tail<2>();
            Eigen::Vector2d hybrid_force_feedforward = Eigen::Vector2d::Zero();
            double operational_damping = 0.03;
            const Eigen::VectorXd full_twist = jacobian * dq;
            last_probe_force_z_n = 0.0;
            last_force_error_n = 0.0;
            mode3_position_recovery_active = false;

            if (teleop_io != nullptr) {
                TeleopCommand command;
                std::uint64_t command_age_ns = 0;
                const bool received = teleop_io->ReadCommand(
                    command, command_age_ns);
                teleop_command_fresh
                    = received
                      && command_age_ns
                             <= static_cast<std::uint64_t>(
                                 options.teleop_command_timeout_ms * 1e6);
                if (teleop_command_fresh) teleop_command = command;
                const TeleopControlMode requested_mode
                    = teleop_command_fresh ? teleop_command.mode : teleop_mode;
                const bool requested_enabled
                    = teleop_command_fresh && teleop_command.enabled;
                if (requested_mode != teleop_mode
                    || requested_enabled != teleop_enabled) {
                    ResetTeleopReference(task_position, probe_rotation,
                        flange_rotation, wrist.rotation_flange, q_wrist, qa,
                        requested_mode, requested_enabled, now);
                    std::cout << "RT_TELEOP mode="
                              << static_cast<std::uint32_t>(teleop_mode)
                              << " enabled=" << static_cast<int>(teleop_enabled)
                              << std::endl;
                }

                Eigen::Vector3d requested_position = teleop_anchor_position;
                Eigen::Matrix3d requested_rotation
                    = teleop_probe_anchor_rotation;
                Eigen::Vector3d requested_linear_velocity
                    = Eigen::Vector3d::Zero();
                Eigen::Vector3d requested_angular_velocity
                    = Eigen::Vector3d::Zero();
                if (teleop_enabled && teleop_command_fresh) {
                    requested_position = PosePosition(teleop_command.target_pose);
                    requested_rotation = PoseRotation(teleop_command.target_pose);
                    requested_linear_velocity = Eigen::Map<const Eigen::Vector3d>(
                        teleop_command.target_linear_velocity.data());
                    requested_angular_velocity = Eigen::Map<const Eigen::Vector3d>(
                        teleop_command.target_angular_velocity.data());
                    if (teleop_mode == TeleopControlMode::OrientationForce) {
                        // Mode 3 explicitly discards haptic translation.
                        requested_position = teleop_anchor_position;
                        requested_linear_velocity.setZero();
                    }
                }
                Eigen::Matrix<double, 6, 7> teleop_arm_jacobian
                    = jacobian.leftCols<7>();
                if (teleop_mode == TeleopControlMode::ArmWrist9Dof
                    || teleop_mode == TeleopControlMode::OrientationForce) {
                    // Translation is measured at the physical probe point;
                    // orientation allocation is measured at the flange so
                    // Flexiv does not cancel useful q8/q9 rotation.
                    teleop_arm_jacobian.bottomRows<3>()
                        = flange_jacobian.bottomRows<3>();
                }
                const ArmSingularityStatus singularity
                    = EvaluateArmSingularity(teleop_arm_jacobian,
                        wrist_model.teleop_singularity_characteristic_length_m,
                        wrist_model.teleop_singularity_slow_sigma,
                        wrist_model.teleop_singularity_critical_sigma,
                        wrist_model.teleop_singularity_min_motion_scale);
                last_arm_singularity_sigma_min = singularity.sigma_min;
                last_arm_motion_scale = singularity.motion_scale;
                if (teleop_enabled
                    && singularity.sigma_min
                           >= wrist_model.teleop_singularity_slow_sigma) {
                    // Mode 1 may let this engagement's redundancy reference
                    // follow deliberate healthy motion slowly. Mode 2 defaults
                    // to a zero rate, preserving the clutch-captured arm
                    // posture while q8/q9 take the reachable orientation.
                    // Near a singularity every mode freezes its latest healthy
                    // posture reference.
                    const double posture_reference_rate
                        = teleop_mode == TeleopControlMode::ArmWrist9Dof
                              ? wrist_model.mode2_posture_reference_rate_per_s
                              : wrist_model.teleop_posture_reference_rate_per_s;
                    const double alpha = std::clamp(
                        0.001 * posture_reference_rate, 0.0, 1.0);
                    teleop_safe_arm_q += alpha * (qa - teleop_safe_arm_q);
                }
                ShapeTeleopTarget(requested_position, requested_rotation,
                    requested_linear_velocity, requested_angular_velocity,
                    singularity.motion_scale);
                last_arm_orientation_error_world.setZero();
                if (!teleop_enabled) {
                    // A released clutch is a joint brake/hold, not another
                    // Cartesian redistribution task. Capture happened on the
                    // release edge in ResetTeleopReference(). Applying this
                    // immediately (without a startup ramp) removes residual
                    // target-filter velocity and prevents the 9-DoF allocator
                    // from moving the arm while settling the wrist.
                    osc_torque.head<7>() = InertiaShapedJointHold(arm_mass,
                        q_reference.head<7>() - qa, dqa,
                        wrist_model.clutch_hold_natural_frequency_hz,
                        wrist_model.clutch_hold_damping_ratio);
                    raw_wrist_target = teleop_wrist_hold_q;
                    error.head<3>() = teleop_position - task_position;
                    error.tail<3>()
                        = RotationError(probe_rotation, teleop_rotation);
                } else if (teleop_mode == TeleopControlMode::Arm7Dof
                    || teleop_mode == TeleopControlMode::Hold) {
                    const Eigen::VectorXd arm_twist
                        = teleop_arm_jacobian * dqa;
                    Eigen::VectorXd velocity_error(6), desired_acceleration(6);
                    error.head<3>() = teleop_position - task_position;
                    error.tail<3>()
                        = RotationError(probe_rotation, teleop_rotation);
                    last_arm_orientation_error_world = error.tail<3>();
                    velocity_error.head<3>()
                        = teleop_linear_velocity - arm_twist.head<3>();
                    velocity_error.tail<3>()
                        = teleop_angular_velocity - arm_twist.tail<3>();
                    desired_acceleration.setZero();
                    operational_damping = AutomaticOperationalDamping(
                        arm_mass, teleop_arm_jacobian,
                        wrist_model.teleop_max_operational_damping);
                    const auto arm_result = OperationalSpaceTorque(arm_mass,
                        teleop_arm_jacobian, error, velocity_error,
                        desired_acceleration,
                        teleop_safe_arm_q - qa, dqa,
                        wrist_model.teleop_arm_translation_kp,
                        wrist_model.teleop_arm_translation_kd,
                        wrist_model.teleop_arm_rotation_kp,
                        wrist_model.teleop_arm_rotation_kd,
                        wrist_model.nullspace_kp,
                        wrist_model.nullspace_kd, operational_damping);
                    osc_torque.head<7>() = arm_result.torque;
                    task_lambda = arm_result.lambda;
                    raw_wrist_target = teleop_wrist_hold_q;
                } else {
                    double mode3_position_priority_scale = 1.0;
                    if (teleop_mode == TeleopControlMode::OrientationForce) {
                        const double position_error_norm
                            = (teleop_anchor_position - task_position).norm();
                        const double fade_width
                            = wrist_model
                                  .mode3_force_disable_position_error_m
                              - wrist_model
                                    .mode3_force_full_position_error_m;
                        mode3_position_priority_scale = SmoothRamp(
                            (wrist_model
                                     .mode3_force_disable_position_error_m
                                  - position_error_norm)
                            / fade_width);
                        mode3_position_recovery_active
                            = mode3_position_priority_scale < 0.999;
                    }
                    const Eigen::Matrix3d rotation_delta_world
                        = requested_rotation
                          * teleop_probe_anchor_rotation.transpose();
                    const Eigen::Matrix3d rotation_delta_flange
                        = teleop_flange_anchor_rotation.transpose()
                          * rotation_delta_world
                          * teleop_flange_anchor_rotation;
                    const Eigen::Matrix3d desired_wrist_rotation
                        = rotation_delta_flange * teleop_wrist_anchor_rotation;
                    const Eigen::Vector2d wrist_ik_seed
                        = wrist_target_initialized ? shaped_wrist_target
                                                   : q_wrist;
                    raw_wrist_target = SolveWristOrientationTarget(
                        wrist_model, wrist_ik_seed, desired_wrist_rotation);
                    if (teleop_mode == TeleopControlMode::OrientationForce) {
                        // If the contact point is displaced, pause new q8/q9
                        // orientation progress. Joint-8 rotation moves the
                        // offset physical TCP, so continuing the wrist target
                        // while the arm is recovering can detach the probe.
                        // Resume smoothly after XYZ returns inside 1 mm.
                        raw_wrist_target = q_wrist
                            + mode3_position_priority_scale
                                  * (raw_wrist_target - q_wrist);
                    }
                    const WristKinematics desired_wrist
                        = EvaluateWrist(wrist_model, raw_wrist_target);
                    const Eigen::Matrix3d raw_desired_flange_rotation
                        = requested_rotation
                          * desired_wrist.rotation_flange.transpose();
                    const Eigen::Matrix3d desired_flange_rotation
                        = ShapeArmFlangeTarget(raw_desired_flange_rotation,
                            singularity.motion_scale
                                * mode3_position_priority_scale);

                    if (teleop_mode == TeleopControlMode::OrientationForce
                        && teleop_enabled) {
                        // Hierarchical Mode 3 allocation:
                        //   1. q8/q9 own every orientation component their two
                        //      physical axes can generate;
                        //   2. Flexiv holds the physical probe point and only
                        //      supplies the small residual flange rotation.
                        // This avoids making all nine joints race toward the
                        // same orientation error while the slower moteus
                        // position loop is still following its target.
                        const Eigen::Vector3d tool_z = probe_rotation.col(2);
                        const Eigen::Vector3d position_error
                            = teleop_anchor_position - task_position;
                        const Eigen::Vector3d orientation_error
                            = RotationError(probe_rotation, requested_rotation);
                        error.head<3>() = position_error;
                        error.tail<3>() = orientation_error;
                        const Eigen::Vector3d arm_orientation_error
                            = RotationError(
                            flange_rotation, desired_flange_rotation);
                        last_arm_orientation_error_world
                            = arm_orientation_error;
                        // Position damping observes the complete physical tip
                        // velocity, including q8/q9. Flexiv therefore cancels
                        // only the tip displacement caused by wrist rotation.
                        // Its angular task observes flange motion only, so it
                        // never tries to cancel the useful q8/q9 rotation.
                        const Eigen::Vector3d position_velocity_error
                            = -full_twist.head<3>();
                        const Eigen::Vector3d orientation_velocity_error
                            = -(flange_jacobian.bottomRows<3>() * dqa);
                        const Eigen::Matrix<double, 3, 7> position_jacobian
                            = teleop_arm_jacobian.topRows<3>();
                        const Eigen::Matrix<double, 3, 7> orientation_jacobian
                            = flange_jacobian.bottomRows<3>();
                        operational_damping = AutomaticOperationalDamping(
                            arm_mass, position_jacobian,
                            wrist_model.teleop_max_operational_damping);
                        const auto motion_result
                            = PositionPriorityOperationalSpaceTorque(
                            arm_mass, position_jacobian,
                            orientation_jacobian, position_error,
                            position_velocity_error, arm_orientation_error,
                            orientation_velocity_error,
                            teleop_safe_arm_q - qa, dqa,
                            wrist_model.mode3_position_kp,
                            wrist_model.mode3_position_kd,
                            wrist_model.mode3_arm_rotation_kp,
                            wrist_model.mode3_arm_rotation_kd,
                            wrist_model.nullspace_kp,
                            wrist_model.nullspace_kd, operational_damping,
                            mode3_position_priority_scale);
                        osc_torque.head<7>() = motion_result.torque;
                        task_lambda = motion_result.lambda;

                        const Eigen::Vector3d measured_force_world
                            = Eigen::Map<const Eigen::Vector3d>(
                                  states.ext_wrench_in_world.data())
                              - force_bias_world;
                        last_probe_force_z_n
                            = tool_z.dot(measured_force_world);
                        last_force_error_n
                            = teleop_command.target_force_z_n
                              - last_probe_force_z_n;
                        const bool contact_has_target_sign
                            = teleop_command.target_force_z_n
                                  * last_probe_force_z_n
                              > 0.0;
                        if (!mode3_force_active
                            && contact_has_target_sign
                            && std::abs(last_probe_force_z_n)
                                   >= wrist_model
                                       .mode3_contact_enable_threshold_n
                            && position_error.norm()
                                   <= wrist_model
                                       .mode3_force_disable_position_error_m) {
                            mode3_force_active = true;
                            mode3_force_integral_n = 0.0;
                            mode3_contact_loss_cycles = 0;
                            std::cout
                                << "RT_MODE3 force control latched at measured_Fz="
                                << last_probe_force_z_n << "N" << std::endl;
                        }
                        if (mode3_force_active) {
                            const bool contact_still_valid
                                = contact_has_target_sign
                                  && std::abs(last_probe_force_z_n)
                                         >= wrist_model
                                             .mode3_contact_release_threshold_n;
                            if (contact_still_valid) {
                                mode3_contact_loss_cycles = 0;
                            } else {
                                ++mode3_contact_loss_cycles;
                            }
                            const std::uint32_t release_cycles
                                = static_cast<std::uint32_t>(std::max(
                                    1.0, std::ceil(1000.0
                                        * wrist_model
                                              .mode3_contact_release_delay_s)));
                            if (mode3_contact_loss_cycles >= release_cycles) {
                                mode3_force_active = false;
                                mode3_force_integral_n = 0.0;
                                mode3_force_position_scale = 0.0;
                                std::cout
                                    << "RT_MODE3 contact lost; force term off, "
                                       "captured XYZ hold remains active"
                                    << std::endl;
                            }
                        }
                        if (mode3_force_active) {
                            // Fixed Cartesian point has priority over force.
                            // The force term is fully active only close to the
                            // captured point, then fades smoothly to zero. It
                            // can therefore never turn into a free-space
                            // surface-search command after contact is lost.
                            const double position_fade_width
                                = wrist_model
                                      .mode3_force_disable_position_error_m
                                  - wrist_model
                                        .mode3_force_full_position_error_m;
                            const double position_fade_input
                                = (wrist_model
                                         .mode3_force_disable_position_error_m
                                      - position_error.norm())
                                  / position_fade_width;
                            mode3_force_position_scale
                                = SmoothRamp(position_fade_input);

                            // Accept the requested force as a band rather than
                            // chasing one noisy number. For a -15 N target and
                            // 1 N tolerance, every reading in [-16,-14] N has
                            // zero PI error.
                            double force_error_outside_tolerance = 0.0;
                            if (std::abs(last_force_error_n)
                                > wrist_model.mode3_force_tolerance_n) {
                                force_error_outside_tolerance
                                    = last_force_error_n
                                      - std::copysign(
                                          wrist_model.mode3_force_tolerance_n,
                                          last_force_error_n);
                            }
                            if (force_error_outside_tolerance == 0.0
                                || mode3_force_position_scale < 0.999) {
                                // Remove old integral gently while the force
                                // is accepted or while position recovery owns
                                // the task; this prevents hidden wind-up.
                                mode3_force_integral_n
                                    *= std::exp(-2.0 * 0.001);
                            } else {
                                mode3_force_integral_n += 0.001
                                    * wrist_model.mode3_force_ki_per_s
                                    * force_error_outside_tolerance;
                                mode3_force_integral_n = std::clamp(
                                    mode3_force_integral_n,
                                    -wrist_model.mode3_force_integral_limit_n,
                                    wrist_model.mode3_force_integral_limit_n);
                            }
                            const double axial_velocity
                                = tool_z.dot(full_twist.head<3>());
                            double actuator_force_z
                                = -teleop_command.target_force_z_n
                                  - wrist_model.mode3_force_kp
                                        * force_error_outside_tolerance
                                  - mode3_force_integral_n
                                  - wrist_model.mode3_force_damping_n_s_m
                                        * axial_velocity;
                            actuator_force_z = mode3_force_position_scale
                                * std::clamp(actuator_force_z,
                                    -wrist_model.mode3_force_command_limit_n,
                                    wrist_model.mode3_force_command_limit_n);
                            const Eigen::Matrix<double, 1, 9> force_jacobian
                                = tool_z.transpose()
                                  * jacobian.topRows<3>();
                            const Eigen::VectorXd force_torque
                                = force_jacobian.transpose()
                                  * actuator_force_z;
                            osc_torque += force_torque;
                            hybrid_force_feedforward
                                = force_torque.tail<2>();
                        } else {
                            // A non-zero force target must never become an
                            // autonomous free-space search. The operator first
                            // makes gentle contact, then this gate latches.
                            mode3_force_integral_n = 0.0;
                            mode3_contact_loss_cycles = 0;
                            mode3_force_position_scale = 0.0;
                        }
                    } else {
                        // Mode 2 uses the same natural wrist-first allocation
                        // as Mode 3, but its Cartesian position follows the
                        // haptic target. There is no artificial angle gain:
                        // q8/q9 solve their reachable 2-axis orientation 1:1,
                        // while Flexiv supplies translation and only the
                        // remaining flange rotation. Position is the primary
                        // task; residual arm orientation is projected into its
                        // dynamically consistent null space. This prevents the
                        // seven-axis arm from performing a large reconfiguration
                        // merely to reduce a small residual orientation error.
                        error.head<3>() = teleop_position - task_position;
                        error.tail<3>()
                            = RotationError(probe_rotation, requested_rotation);
                        const Eigen::Vector3d arm_orientation_error = RotationError(
                            flange_rotation, desired_flange_rotation);
                        last_arm_orientation_error_world = arm_orientation_error;
                        const Eigen::Vector3d position_velocity_error
                            = teleop_linear_velocity - full_twist.head<3>();
                        const Eigen::Vector3d orientation_velocity_error
                            = -(flange_jacobian.bottomRows<3>() * dqa);
                        const Eigen::Matrix<double, 3, 7> position_jacobian
                            = teleop_arm_jacobian.topRows<3>();
                        const Eigen::Matrix<double, 3, 7> orientation_jacobian
                            = flange_jacobian.bottomRows<3>();
                        operational_damping = AutomaticOperationalDamping(
                            arm_mass, position_jacobian,
                            wrist_model.teleop_max_operational_damping);
                        const auto result = PositionPriorityOperationalSpaceTorque(
                            arm_mass, position_jacobian, orientation_jacobian,
                            error.head<3>(), position_velocity_error,
                            arm_orientation_error, orientation_velocity_error,
                            teleop_safe_arm_q - qa, dqa,
                            wrist_model.teleop_arm_translation_kp,
                            wrist_model.teleop_arm_translation_kd,
                            wrist_model.mode2_arm_rotation_kp,
                            wrist_model.mode2_arm_rotation_kd,
                            wrist_model.nullspace_kp,
                            wrist_model.nullspace_kd,
                            operational_damping);
                        osc_torque.head<7>() = result.torque;
                        task_lambda = result.lambda;
                        last_force_error_n = 0.0;
                    }
                }
            } else {
                LoopTarget target;
                if (options.trajectory == "orientation") {
                    target = ContinuousOrientationTarget(position_origin,
                        rotation_origin, elapsed, 1.0 / options.duration_s,
                        options.orientation_deg * kPi / 180.0);
                } else if (options.trajectory == "spin") {
                    target = ContinuousSpinTarget(position_origin,
                        rotation_origin, elapsed, 1.0 / options.duration_s,
                        options.orientation_deg * kPi / 180.0);
                } else if (options.trajectory == "rectangle") {
                    target = ContinuousRoundedRectangleTarget(position_origin,
                        rotation_origin, elapsed, options.rectangle_width_m,
                        options.rectangle_height_m,
                        options.rectangle_corner_radius_m,
                        1.0 / options.duration_s, options.tangent_axis);
                } else {
                    target = ContinuousLoopTarget(position_origin,
                        rotation_origin, elapsed, options.radius_m,
                        1.0 / options.duration_s,
                        options.orientation_deg * kPi / 180.0);
                }
                Eigen::VectorXd velocity_error(6), desired_acceleration(6);
                error.head<3>() = target.position - task_position;
                error.tail<3>() = RotationError(probe_rotation, target.rotation);
                velocity_error.head<3>()
                    = target.linear_velocity - full_twist.head<3>();
                velocity_error.tail<3>()
                    = target.angular_velocity - full_twist.tail<3>();
                desired_acceleration.head<3>() = target.linear_acceleration;
                desired_acceleration.tail<3>() = target.angular_acceleration;
                if (options.trajectory == "rectangle") {
                    const double phase = 2.0 * kPi * elapsed / options.duration_s;
                    const double ramp_u = std::clamp(elapsed / 2.0, 0.0, 1.0);
                    const double envelope = 10.0 * std::pow(ramp_u, 3)
                                            - 15.0 * std::pow(ramp_u, 4)
                                            + 6.0 * std::pow(ramp_u, 5);
                    raw_wrist_target = q_reference.tail<2>()
                        + envelope
                              * Eigen::Vector2d(
                                  wrist_model.wrist_rectangle_excursion_rad[0]
                                      * std::sin(phase),
                                  wrist_model.wrist_rectangle_excursion_rad[1]
                                      * std::sin(2.0 * phase));
                    const Eigen::Vector2d lower
                        = wrist_model.joint_min.array()
                          + wrist_model.joint_limit_margin_rad;
                    const Eigen::Vector2d upper
                        = wrist_model.joint_max.array()
                          - wrist_model.joint_limit_margin_rad;
                    raw_wrist_target
                        = raw_wrist_target.cwiseMax(lower).cwiseMin(upper);
                } else {
                    const Eigen::Matrix3d desired_wrist_rotation_flange
                        = flange_rotation_reference.transpose()
                          * target.rotation;
                    raw_wrist_target = SolveWristOrientationTarget(
                        wrist_model, q_wrist,
                        desired_wrist_rotation_flange);
                }
                Eigen::VectorXd null_error = q_reference - q;
                null_error.tail<2>().setZero();
                operational_damping
                    = options.inertia_mode == "auto"
                          ? AutomaticOperationalDamping(mass, jacobian)
                          : 0.03;
                const auto result = OperationalSpaceTorque(mass, jacobian,
                    error, velocity_error, desired_acceleration, null_error,
                    dq, wrist_model.task_translation_kp,
                    wrist_model.task_translation_kd,
                    wrist_model.task_rotation_kp,
                    wrist_model.task_rotation_kd,
                    wrist_model.nullspace_kp,
                    wrist_model.nullspace_kd, operational_damping);
                osc_torque = result.torque;
                task_lambda = result.lambda;
            }
            const Eigen::Vector2d wrist_target
                = ShapeWristTarget(raw_wrist_target, q_wrist);
            Eigen::VectorXd absolute_limit(9), rate_limit(9);
            absolute_limit.head<7>() = wrist_model.arm_torque_limit;
            absolute_limit.tail<2>() << 6.0, 2.0;
            rate_limit.head<7>() = wrist_model.arm_torque_slew;
            rate_limit.tail<2>() = wrist_model.wrist_torque_slew;
            // The composite OSC uses live M_wrist(q).  Add the wrist's
            // physical gravity/friction feed-forward before applying the
            // independent actuator rating and torque-rate bounds.  Gravity is
            // transformed from world into the moving flange frame every 1 ms.
            if (torque_ramp_started.time_since_epoch().count() == 0) {
                torque_ramp_started = started;
                torque_ramp_duration_s = wrist_model.torque_startup_ramp_s;
            }
            const double torque_scale = SmoothRamp(
                std::chrono::duration<double>(now - torque_ramp_started).count()
                / torque_ramp_duration_s);
            Eigen::VectorXd requested_torque = torque_scale * osc_torque;
            requested_torque.head<7>() += arm_coriolis;
            // The dynamically consistent null-space projection alone was
            // measured at only 0.1--0.2 Nm, below joint 8's identified
            // 0.563 Nm breakaway friction. Add direct output-joint tracking;
            // the primary probe OSC simultaneously compensates it with the
            // seven Flexiv joints.
            requested_torque.tail<2>()
                += torque_scale * (
                    wrist_model.wrist_tracking_kp.cwiseProduct(
                       wrist_target - q_wrist)
                    - wrist_model.wrist_tracking_kd.cwiseProduct(dq_wrist));
            const Eigen::Vector2d wrist_gravity = dynamic_gravity.wrist;
            const Eigen::Vector2d friction_direction_velocity
                = dq_wrist + 2.0 * (wrist_target - q_wrist);
            const Eigen::Vector2d wrist_friction
                = wrist_model.viscous_friction.cwiseProduct(dq_wrist)
                  + wrist_model.coulomb_friction.cwiseProduct(
                        (friction_direction_velocity / 0.02).array().tanh().matrix())
                  + wrist_model.torque_bias;
            // Flexiv retains its calibrated Elements gravity compensation at
            // q8=q9=0. Add only the configuration-dependent change produced
            // by the articulated wrist, so gravity is complete but never
            // counted twice.
            requested_torque.head<7>() += dynamic_gravity.arm_delta;
            requested_torque.tail<2>()
                += wrist_gravity
                   + torque_scale
                         * (WristCoriolis(wrist_model, q_wrist, dq_wrist)
                             + wrist_friction);
            const Eigen::VectorXd torque = RateAndMagnitudeLimit(
                requested_torque, previous_torque, absolute_limit, rate_limit);
            previous_torque = torque;
            // In the commissioned hybrid mode, moteus already supplies the
            // fast output-joint PD around wrist_target. Sending the OSC task
            // PD and direct tracker again as feed-forward created duplicate
            // stiffness and was the main difference from stable Mode-2
            // teleoperation. Keep only slow physical gravity feed-forward in
            // hybrid mode. Pure-torque mode still receives the full OSC torque.
            Eigen::Vector2d wrist_actuator_torque = torque.tail<2>();
            if (options.wrist_execution_mode == "hybrid_position_ff") {
                wrist_actuator_torque
                    = wrist_gravity + torque_scale * hybrid_force_feedforward;
            }
            const Eigen::Vector2d wrist_absolute_limit(6.0, 2.0);
            for (int index = 0; index < 2; ++index) {
                const double maximum_step
                    = wrist_model.wrist_torque_slew[index] * 0.001;
                wrist_actuator_torque[index] = std::clamp(
                    wrist_actuator_torque[index],
                    previous_wrist_actuator_torque[index] - maximum_step,
                    previous_wrist_actuator_torque[index] + maximum_step);
                wrist_actuator_torque[index] = std::clamp(
                    wrist_actuator_torque[index], -wrist_absolute_limit[index],
                    wrist_absolute_limit[index]);
            }
            previous_wrist_actuator_torque = wrist_actuator_torque;
            robot.StreamJointTorque(
                std::vector<double>(torque.data(), torque.data() + 7), true, true);
            wrist_io.WriteTorque(
                {wrist_actuator_torque[0], wrist_actuator_torque[1]},
                {wrist_target[0], wrist_target[1]});
            last_position_error_m = error.head<3>().norm();
            last_orientation_error_rad = error.tail<3>().norm();
            if (teleop_io != nullptr) {
                TeleopState teleop_state;
                teleop_state.probe_pose = PoseArray(task_position, probe_rotation);
                std::copy_n(states.ext_wrench_in_world.begin(), 6,
                    teleop_state.external_wrench_world.begin());
                teleop_state.wrist_q_rad = {q_wrist[0], q_wrist[1]};
                teleop_state.mode = teleop_mode;
                teleop_state.status_flags
                    = (teleop_command_fresh ? 1U : 0U)
                      | (teleop_enabled ? 2U : 0U)
                      | (mode3_force_active ? 16U : 0U)
                      | (mode3_position_recovery_active ? 32U : 0U);
                teleop_state.probe_force_z_n = last_probe_force_z_n;
                teleop_state.position_error_m = last_position_error_m;
                teleop_state.orientation_error_rad
                    = last_orientation_error_rad;
                teleop_state.force_error_n = last_force_error_n;
                teleop_state.arm_orientation_error_world = {
                    last_arm_orientation_error_world.x(),
                    last_arm_orientation_error_world.y(),
                    last_arm_orientation_error_world.z()};
                teleop_state.arm_singularity_sigma_min
                    = last_arm_singularity_sigma_min;
                teleop_state.arm_motion_scale = last_arm_motion_scale;
                teleop_io->WriteState(teleop_state);
            }
            if (log && cycle_count++ % 10 == 0) {
                log << std::setprecision(10) << elapsed << ',' << error.head<3>().norm()
                    << ',' << error.tail<3>().norm() << ',' << dqa.norm() << ','
                    << dq_wrist.norm() << ',' << torque.head<7>().norm() << ','
                    << wrist_actuator_torque.norm() << ',' << q_wrist[0] << ','
                    << q_wrist[1] << ',' << wrist_target[0] << ',' << wrist_target[1]
                    << ',' << wrist_actuator_torque[0] << ','
                    << wrist_actuator_torque[1]
                    << ',' << measured_torque[0] << ',' << measured_torque[1]
                    << ',' << SymmetricConditionNumber(mass) << ','
                    << SymmetricConditionNumber(task_lambda) << ','
                    << operational_damping << ','
                    << last_arm_singularity_sigma_min << ','
                    << last_arm_motion_scale << ','
                    << last_arm_orientation_error_world.x() << ','
                    << last_arm_orientation_error_world.y() << ','
                    << last_arm_orientation_error_world.z() << ','
                    << dynamic_gravity.arm_delta.norm() << ','
                    << wrist_gravity[0] << ',' << wrist_gravity[1] << '\n';
            }
        } catch (const std::exception& error) {
            std::cerr << "RT_ERROR " << error.what() << std::endl;
            g_stop = true;
        }
    }
};

} // namespace

int main(int argc, char** argv)
{
    try {
        const Options options = Parse(argc, argv);
        if (!options.real) {
            if (options.trajectory == "orientation") {
                WriteOrientationDryRunCsv(options.dry_run_csv,
                    options.duration_s,
                    options.orientation_deg * kPi / 180.0);
            } else if (options.trajectory == "spin") {
                WriteSpinDryRunCsv(options.dry_run_csv, options.duration_s,
                    options.orientation_deg * kPi / 180.0);
            } else if (options.trajectory == "rectangle") {
                WriteRoundedRectangleDryRunCsv(options.dry_run_csv,
                    options.duration_s, options.rectangle_width_m,
                    options.rectangle_height_m,
                    options.rectangle_corner_radius_m, options.tangent_axis);
            } else {
                WriteDryRunCsv(options.dry_run_csv, options.duration_s,
                    options.radius_m, 1.0 / options.duration_s,
                    options.orientation_deg * kPi / 180.0);
            }
            std::cout << "DRY_RUN_ONLY trajectory=" << options.dry_run_csv
                      << " no robot/wrist command was sent" << std::endl;
            return 0;
        }
        std::signal(SIGINT, SignalHandler);
        std::signal(SIGTERM, SignalHandler);
        WristSharedMemory wrist_io(options.shared_memory);
        std::unique_ptr<TeleopSharedMemory> teleop_io;
        if (!options.teleop_shared_memory.empty()) {
            teleop_io = std::make_unique<TeleopSharedMemory>(
                options.teleop_shared_memory);
        }
        const WristModel wrist_model = LoadWristModel(options.model_config);
        std::array<double, 2> qw {}, dqw {}, tw {};
        std::uint64_t wrist_age = 0;
        if (!wrist_io.ReadState(qw, dqw, tw, wrist_age)
            || wrist_age
                   > static_cast<std::uint64_t>(options.wrist_state_timeout_ms * 1000000.0)) {
            throw std::runtime_error("wrist bridge not ready before robot mode switch");
        }
        rdk::Robot robot(options.robot_sn);
        if (robot.fault() || !robot.operational()) throw std::runtime_error(
            "robot must already be enabled, operational and fault-free");
        rdk::Model model(robot);
        // Explicitly refresh the locally cached dynamics after Python has
        // verified the live Elements Tool.  Model::M() then contains the
        // current calibrated payload at the q8=q9=0 baseline.
        model.Reload();
        const auto states = robot.states();
        if (states.q.size() != 7) throw std::runtime_error("requires a seven-joint arm");
        const Eigen::Vector2d q_wrist(qw[0], qw[1]);
        const auto wrist = EvaluateWrist(wrist_model, q_wrist);
        if (options.inertia_mode == "auto") {
            const auto tool_spatial = SpatialInertiaAtOrigin(
                wrist_model.active_tool_mass, wrist_model.active_tool_com,
                wrist_model.active_tool_inertia);
            std::cout << "AUTO_INERTIA mode=coupled spatial=6x6 joint=9x9 task=6x6 "
                      << "tool_spatial_condition="
                      << SymmetricConditionNumber(tool_spatial) << std::endl;
        }
        const Eigen::Vector3d initial_task_offset
            = teleop_io != nullptr ? wrist.position_flange
                                   : (options.endpoint == "pivot"
                                             ? wrist_model.joint1_origin
                                             : wrist.position_flange);
        const Eigen::Vector3d p0 = PosePosition(states.flange_pose)
                                   + PoseRotation(states.flange_pose)
                                         * initial_task_offset;
        const Eigen::Matrix3d r0
            = PoseRotation(states.flange_pose) * wrist.rotation_flange;
        Eigen::VectorXd q_reference(9);
        q_reference << Eigen::Map<const Eigen::VectorXd>(states.q.data(), 7), q_wrist;
        Controller controller {robot, model, wrist_io, wrist_model, options,
            q_reference, Eigen::VectorXd::Zero(9), p0, r0,
            PoseRotation(states.flange_pose),
            std::chrono::steady_clock::now(), std::ofstream("/tmp/flexiv_9dof_osc_rt.csv")};
        controller.teleop_io = teleop_io.get();
        controller.force_bias_world = Eigen::Map<const Eigen::Vector3d>(
            states.ext_wrench_in_world.data());
        controller.ResetTeleopReference(p0, r0,
            PoseRotation(states.flange_pose), wrist.rotation_flange,
            q_wrist, Eigen::Map<const Eigen::VectorXd>(states.q.data(), 7),
            TeleopControlMode::Hold, false, std::chrono::steady_clock::now());
        controller.log << "time_s,tip_position_error_m,orientation_error_rad,arm_dq_norm,"
                          "wrist_dq_norm,arm_tau_norm,wrist_tau_norm,q8_rad,q9_rad,"
                          "target_q8_rad,target_q9_rad,command_tau8_nm,command_tau9_nm,"
                          "applied_tau8_nm,applied_tau9_nm,mass_condition,lambda_condition,"
                          "operational_damping,arm_singularity_sigma_min,arm_motion_scale,"
                          "arm_rotation_error_x_rad,arm_rotation_error_y_rad,"
                          "arm_rotation_error_z_rad,arm_wrist_gravity_delta_norm_nm,"
                          "wrist_gravity8_nm,wrist_gravity9_nm\n";
        if (options.inertia_mode == "legacy-block") {
            std::cerr << "WARNING legacy block-diagonal M omits arm/wrist coupling.\n";
        }
        std::cout
            << "DYNAMIC_GRAVITY flexiv_internal_baseline=1 wrist_arm_delta="
            << static_cast<int>(wrist_model.dynamic_wrist_gravity_compensation)
            << " wrist_full=1 filter_hz="
            << wrist_model.dynamic_gravity_filter_hz << std::endl;
        if (teleop_io != nullptr) {
            std::cout << "RT_TELEOP_READY modes=hold,7dof_osc,9dof_osc,"
                         "orientation_force_osc rate_hz=1000 hold_frequency_hz="
                      << wrist_model.clutch_hold_natural_frequency_hz
                      << " hold_damping_ratio="
                      << wrist_model.clutch_hold_damping_ratio
                      << " mode3_fixed_xyz=1 mode3_position_kp="
                      << wrist_model.mode3_position_kp
                      << " mode3_position_kd="
                      << wrist_model.mode3_position_kd
                      << " wrist_first_allocation=1"
                      << " mode2_position_priority=1"
                      << " mode2_arm_max_angular_velocity_rad_s="
                      << wrist_model.mode2_arm_max_angular_velocity_rad_s
                      << " mode2_posture_reference_rate_per_s="
                      << wrist_model.mode2_posture_reference_rate_per_s
                      << " singularity_slow_sigma="
                      << wrist_model.teleop_singularity_slow_sigma
                      << " singularity_critical_sigma="
                      << wrist_model.teleop_singularity_critical_sigma
                      << " singularity_min_motion_scale="
                      << wrist_model.teleop_singularity_min_motion_scale
                      << " mode3_contact_gate_N="
                      << wrist_model.mode3_contact_enable_threshold_n
                      << " mode3_force_tolerance_N="
                      << wrist_model.mode3_force_tolerance_n
                      << " mode3_force_position_fade_m="
                      << wrist_model.mode3_force_full_position_error_m << ':'
                      << wrist_model.mode3_force_disable_position_error_m
                      << std::endl;
        }
        rdk::Scheduler scheduler;
        scheduler.AddTask(std::bind(&Controller::Step, &controller), "9dof_osc_1khz",
            1, scheduler.max_priority(), options.cpu_affinity);
        // Fail any Scheduler privilege check before entering joint-torque
        // mode. This also protects users who invoke the binary directly.
        robot.SetTimelinessFailureLimit(1.0);
        robot.SwitchMode(rdk::Mode::RT_JOINT_TORQUE);
        controller.started = std::chrono::steady_clock::now();
        scheduler.Start();
        while (!g_stop) std::this_thread::sleep_for(std::chrono::milliseconds(2));
        scheduler.Stop();
        wrist_io.WriteTorque({0.0, 0.0});
        wrist_io.RequestStop();
        robot.Stop();
        std::cout << "9DOF_OSC_STOPPED log=/tmp/flexiv_9dof_osc_rt.csv" << std::endl;
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "ERROR " << error.what() << std::endl;
        return 1;
    }
}
