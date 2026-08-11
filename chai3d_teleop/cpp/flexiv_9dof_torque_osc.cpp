#include "rt_osc_common.hpp"
#include "wrist_shared_memory.hpp"

#include <flexiv/rdk/model.hpp>
#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/scheduler.hpp>

#include <limits>
#include <map>
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
    double wrist_priority_gain {2.0};
    Eigen::Vector2d wrist_tracking_kp {8.0, 3.0};
    Eigen::Vector2d wrist_tracking_kd {0.6, 0.25};
    Eigen::Vector2d wrist_rectangle_excursion_rad {
        35.0 * kPi / 180.0, 60.0 * kPi / 180.0};
    double ik_damping {0.03};
    double ik_max_step_rad {4.0 * kPi / 180.0};
    int ik_max_iterations {30};
    double joint_limit_margin_rad {2.0 * kPi / 180.0};
    Eigen::Vector2d joint_min {-0.5 * kPi, -kPi};
    Eigen::Vector2d joint_max {0.5 * kPi, kPi};
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
    const Eigen::MatrixXd& jacobian)
{
    Eigen::LDLT<Eigen::MatrixXd> mass_solver(mass);
    if (mass_solver.info() != Eigen::Success) {
        throw std::runtime_error("cannot factor auto coupled M(q)");
    }
    Eigen::Matrix<double, 6, 6> inverse_lambda
        = jacobian * mass_solver.solve(jacobian.transpose());
    inverse_lambda = 0.5 * (inverse_lambda + inverse_lambda.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(
        inverse_lambda, Eigen::EigenvaluesOnly);
    if (solver.info() != Eigen::Success) {
        throw std::runtime_error("cannot inspect 6x6 operational inertia");
    }
    const double minimum = std::max(0.0, solver.eigenvalues().minCoeff());
    const double maximum = solver.eigenvalues().maxCoeff();
    constexpr double target_condition = 200.0;
    const double required_squared
        = std::max(0.0,
            (maximum - target_condition * minimum) / (target_condition - 1.0));
    return std::clamp(std::sqrt(required_squared), 0.01, 0.20);
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

Eigen::Matrix3d AmplifyRotationAboutReference(const Eigen::Matrix3d& reference,
    const Eigen::Matrix3d& desired, double gain)
{
    const Eigen::AngleAxisd relative(reference.transpose() * desired);
    return reference
           * Eigen::AngleAxisd(gain * relative.angle(), relative.axis())
                 .toRotationMatrix();
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
    std::string robot_sn {"Rizon4s-123456"};
    std::string shared_memory;
    std::string model_config;
    std::string dry_run_csv {"/tmp/flexiv_9dof_osc_loop.csv"};
    std::string trajectory {"orientation"};
    double duration_s {20.0};
    double radius_m {0.030};
    double orientation_deg {3.0};
    double rectangle_width_m {0.060};
    double rectangle_height_m {0.040};
    double rectangle_corner_radius_m {0.010};
    int tangent_axis {0};
    std::string inertia_mode {"auto"};
    double wrist_state_timeout_ms {250.0};
    double wrist_hard_timeout_ms {2000.0};
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
        else if (arg == "--wrist-model") result.model_config = value();
        else if (arg == "--duration-s") result.duration_s = std::stod(value());
        else if (arg == "--radius-m") result.radius_m = std::stod(value());
        else if (arg == "--orientation-deg") result.orientation_deg = std::stod(value());
        else if (arg == "--trajectory") result.trajectory = value();
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
        else if (arg == "--wrist-state-timeout-ms") {
            result.wrist_state_timeout_ms = std::stod(value());
        }
        else if (arg == "--wrist-hard-timeout-ms") {
            result.wrist_hard_timeout_ms = std::stod(value());
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
    if (result.duration_s < 8.0 || result.radius_m <= 0.0 || result.radius_m > 0.05
        || result.orientation_deg <= 0.0 || result.orientation_deg > 25.0
        || result.wrist_state_timeout_ms < 100.0
        || result.wrist_state_timeout_ms > 1000.0
        || result.wrist_hard_timeout_ms <= result.wrist_state_timeout_ms
        || result.wrist_hard_timeout_ms > 5000.0) {
        throw std::invalid_argument("invalid duration/radius/orientation");
    }
    if (result.trajectory != "orientation" && result.trajectory != "rectangle"
        && result.trajectory != "loop") {
        throw std::invalid_argument(
            "--trajectory must be orientation, rectangle or loop");
    }
    if (result.inertia_mode != "auto" && result.inertia_mode != "legacy-block") {
        throw std::invalid_argument("--inertia-mode must be auto or legacy-block");
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
        else throw std::runtime_error("unknown wrist model key: " + key);
    }
    const auto tool_spatial = SpatialInertiaAtOrigin(model.active_tool_mass,
        model.active_tool_com, model.active_tool_inertia);
    Eigen::LDLT<Eigen::Matrix<double, 6, 6>> tool_solver(tool_spatial);
    if (model.active_tool_mass <= 0.0 || tool_solver.info() != Eigen::Success
        || !tool_solver.isPositive() || model.wrist_priority_gain < 1.0
        || (model.wrist_tracking_kp.array() <= 0.0).any()
        || (model.wrist_tracking_kd.array() <= 0.0).any()
        || (model.wrist_rectangle_excursion_rad.array() <= 0.0).any()
        || (model.wrist_rectangle_excursion_rad
                   - (model.joint_max - model.joint_min) * 0.5)
                   .maxCoeff()
               >= 0.0
        || model.ik_damping <= 0.0 || model.ik_max_step_rad <= 0.0
        || model.ik_max_iterations < 1 || model.ik_max_iterations > 100
        || model.joint_limit_margin_rad < 0.0
        || (model.joint_max - model.joint_min).minCoeff()
               <= 2.0 * model.joint_limit_margin_rad) {
        throw std::runtime_error("invalid wrist IK configuration");
    }
    return model;
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
                    = arm_coriolis + 12.0 * (stale_hold_arm_q - qa) - 4.0 * dqa;
                Eigen::VectorXd arm_absolute_limit(7), arm_rate_limit(7);
                arm_absolute_limit << 20.0, 20.0, 15.0, 15.0, 8.0, 8.0, 5.0;
                arm_rate_limit << 200.0, 200.0, 150.0, 150.0, 80.0, 80.0, 50.0;
                const Eigen::VectorXd arm_torque = RateAndMagnitudeLimit(
                    hold_request, previous_torque.head(7), arm_absolute_limit,
                    arm_rate_limit);
                previous_torque.head(7) = arm_torque;
                previous_torque.tail(2).setZero();
                robot.StreamJointTorque(std::vector<double>(
                    arm_torque.data(), arm_torque.data() + arm_torque.size()),
                    true, true);
                wrist_io.WriteTorque({0.0, 0.0});
                return;
            }
            if (stale_hold_active) {
                stale_hold_active = false;
                std::cerr << "RT_INFO wrist state recovered; resuming 9-DoF OSC"
                          << std::endl;
            }
            const Eigen::Vector2d q_wrist(qw[0], qw[1]);
            const Eigen::Vector2d dq_wrist(dqw[0], dqw[1]);
            const Eigen::MatrixXd arm_mass = arm_model.M();
            const Eigen::MatrixXd flange_jacobian = arm_model.J("flange");
            const WristKinematics wrist = EvaluateWrist(wrist_model, q_wrist);
            const Eigen::Vector3d flange_position = PosePosition(states.flange_pose);
            const Eigen::Matrix3d flange_rotation = PoseRotation(states.flange_pose);
            // Control the real live end point. The wrist changes the tip
            // position as well as its attitude, so both the arm and wrist
            // columns must participate in the translational Jacobian.
            const Eigen::Vector3d tip_offset_world
                = flange_rotation * wrist.position_flange;
            const Eigen::Vector3d task_position
                = flange_position + tip_offset_world;
            const Eigen::Matrix3d probe_rotation
                = flange_rotation * wrist.rotation_flange;

            Eigen::Matrix<double, 6, 9> jacobian;
            jacobian.leftCols<7>().topRows<3>()
                = flange_jacobian.topRows<3>()
                  - Skew(tip_offset_world) * flange_jacobian.bottomRows<3>();
            jacobian.leftCols<7>().bottomRows<3>()
                = flange_jacobian.bottomRows<3>();
            jacobian.rightCols<2>().topRows<3>()
                = flange_rotation * wrist.jacobian_flange.topRows<3>();
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
            LoopTarget target;
            if (options.trajectory == "orientation") {
                target = ContinuousOrientationTarget(position_origin,
                    rotation_origin, elapsed, 1.0 / options.duration_s,
                    options.orientation_deg * kPi / 180.0);
            } else if (options.trajectory == "rectangle") {
                target = ContinuousRoundedRectangleTarget(position_origin,
                    rotation_origin, elapsed, options.rectangle_width_m,
                    options.rectangle_height_m,
                    options.rectangle_corner_radius_m,
                    1.0 / options.duration_s, options.tangent_axis);
            } else {
                target = ContinuousLoopTarget(position_origin, rotation_origin,
                    elapsed, options.radius_m, 1.0 / options.duration_s,
                    options.orientation_deg * kPi / 180.0);
            }
            const Eigen::VectorXd twist = jacobian * dq;
            Eigen::VectorXd error(6), velocity_error(6), desired_acceleration(6);
            error.head<3>() = target.position - task_position;
            error.tail<3>() = RotationError(probe_rotation, target.rotation);
            velocity_error.head<3>() = target.linear_velocity - twist.head<3>();
            velocity_error.tail<3>() = target.angular_velocity - twist.tail<3>();
            desired_acceleration.head<3>() = target.linear_acceleration;
            desired_acceleration.tail<3>() = target.angular_acceleration;
            // Ask the distal wrist to realize the target orientation relative
            // to the startup flange, not relative to the already-moving arm.
            // The old latter formulation let the Flexiv arm satisfy the task
            // first, causing the wrist target to collapse back toward its
            // startup angle and making q8/q9 barely move.
            Eigen::Vector2d wrist_target;
            if (options.trajectory == "rectangle") {
                const double phase = 2.0 * kPi * elapsed / options.duration_s;
                const double ramp_u = std::clamp(elapsed / 2.0, 0.0, 1.0);
                const double envelope = 10.0 * std::pow(ramp_u, 3)
                                        - 15.0 * std::pow(ramp_u, 4)
                                        + 6.0 * std::pow(ramp_u, 5);
                wrist_target = q_reference.tail<2>()
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
                wrist_target = wrist_target.cwiseMax(lower).cwiseMin(upper);
            } else {
                const Eigen::Matrix3d wrist_rotation_reference
                    = flange_rotation_reference.transpose() * rotation_origin;
                const Eigen::Matrix3d nominal_wrist_rotation_flange
                    = flange_rotation_reference.transpose() * target.rotation;
                const Eigen::Matrix3d desired_wrist_rotation_flange
                    = AmplifyRotationAboutReference(wrist_rotation_reference,
                        nominal_wrist_rotation_flange,
                        wrist_model.wrist_priority_gain);
                wrist_target = SolveWristOrientationTarget(
                    wrist_model, q_wrist, desired_wrist_rotation_flange);
            }
            Eigen::VectorXd null_error = q_reference - q;
            null_error.tail<2>() = wrist_target - q_wrist;
            const double operational_damping
                = options.inertia_mode == "auto"
                      ? AutomaticOperationalDamping(mass, jacobian)
                      : 0.03;
            auto result = OperationalSpaceTorque(mass, jacobian, error,
                velocity_error, desired_acceleration, null_error, dq,
                100.0, 20.0, 70.0, 16.0, 7.0, 1.8,
                operational_damping);
            Eigen::VectorXd absolute_limit(9), rate_limit(9);
            absolute_limit << 20.0, 20.0, 15.0, 15.0, 8.0, 8.0, 5.0, 6.0, 2.0;
            rate_limit << 200.0, 200.0, 150.0, 150.0, 80.0, 80.0, 50.0, 30.0, 20.0;
            // The composite OSC uses live M_wrist(q).  Add the wrist's
            // physical gravity/friction feed-forward before applying the
            // independent actuator rating and torque-rate bounds.  Gravity is
            // transformed from world into the moving flange frame every 1 ms.
            Eigen::VectorXd requested_torque = result.torque;
            requested_torque.head<7>() += arm_coriolis;
            // The dynamically consistent null-space projection alone was
            // measured at only 0.1--0.2 Nm, below joint 8's identified
            // 0.563 Nm breakaway friction. Add direct output-joint tracking;
            // the primary probe OSC simultaneously compensates it with the
            // seven Flexiv joints.
            requested_torque.tail<2>()
                += wrist_model.wrist_tracking_kp.cwiseProduct(
                       wrist_target - q_wrist)
                   - wrist_model.wrist_tracking_kd.cwiseProduct(dq_wrist);
            const Eigen::Vector3d gravity_flange
                = flange_rotation.transpose() * Eigen::Vector3d(0.0, 0.0, -9.80665);
            const Eigen::Vector2d wrist_gravity
                = -wrist_model.rigid_body_scale
                  * (wrist.link1_linear_jacobian.transpose()
                         * (wrist_model.mass1 * gravity_flange)
                      + wrist.link2_linear_jacobian.transpose()
                            * (wrist_model.mass2 * gravity_flange));
            const Eigen::Vector2d friction_direction_velocity
                = dq_wrist + 2.0 * (wrist_target - q_wrist);
            const Eigen::Vector2d wrist_friction
                = wrist_model.viscous_friction.cwiseProduct(dq_wrist)
                  + wrist_model.coulomb_friction.cwiseProduct(
                        (friction_direction_velocity / 0.02).array().tanh().matrix())
                  + wrist_model.torque_bias;
            requested_torque.tail<2>()
                += WristCoriolis(wrist_model, q_wrist, dq_wrist)
                   + wrist_gravity + wrist_friction;
            const Eigen::VectorXd torque = RateAndMagnitudeLimit(
                requested_torque, previous_torque, absolute_limit, rate_limit);
            previous_torque = torque;
            robot.StreamJointTorque(
                std::vector<double>(torque.data(), torque.data() + 7), true, true);
            wrist_io.WriteTorque(
                {torque[7], torque[8]}, {wrist_target[0], wrist_target[1]});
            if (log && cycle_count++ % 10 == 0) {
                log << std::setprecision(10) << elapsed << ',' << error.head<3>().norm()
                    << ',' << error.tail<3>().norm() << ',' << dqa.norm() << ','
                    << dq_wrist.norm() << ',' << torque.head<7>().norm() << ','
                    << torque.tail<2>().norm() << ',' << q_wrist[0] << ','
                    << q_wrist[1] << ',' << wrist_target[0] << ',' << wrist_target[1]
                    << ',' << SymmetricConditionNumber(mass) << ','
                    << SymmetricConditionNumber(result.lambda) << ','
                    << operational_damping << '\n';
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
        const Eigen::Vector3d p0 = PosePosition(states.flange_pose)
                                   + PoseRotation(states.flange_pose)
                                         * wrist.position_flange;
        const Eigen::Matrix3d r0
            = PoseRotation(states.flange_pose) * wrist.rotation_flange;
        Eigen::VectorXd q_reference(9);
        q_reference << Eigen::Map<const Eigen::VectorXd>(states.q.data(), 7), q_wrist;
        Controller controller {robot, model, wrist_io, wrist_model, options,
            q_reference, Eigen::VectorXd::Zero(9), p0, r0,
            PoseRotation(states.flange_pose),
            std::chrono::steady_clock::now(), std::ofstream("/tmp/flexiv_9dof_osc_rt.csv")};
        controller.log << "time_s,tip_position_error_m,orientation_error_rad,arm_dq_norm,"
                          "wrist_dq_norm,arm_tau_norm,wrist_tau_norm,q8_rad,q9_rad,"
                          "target_q8_rad,target_q9_rad,mass_condition,lambda_condition,"
                          "operational_damping\n";
        if (options.inertia_mode == "legacy-block") {
            std::cerr << "WARNING legacy block-diagonal M omits arm/wrist coupling.\n";
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
