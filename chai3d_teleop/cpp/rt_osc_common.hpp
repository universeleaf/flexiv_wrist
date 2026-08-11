#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace teleop::osc {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDt = 0.001;

inline std::atomic<bool> g_stop {false};

inline void SignalHandler(int) { g_stop = true; }

inline Eigen::Matrix3d Skew(const Eigen::Vector3d& v)
{
    Eigen::Matrix3d result;
    result << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
    return result;
}

inline Eigen::Matrix3d PoseRotation(const std::array<double, 7>& pose)
{
    Eigen::Quaterniond quaternion(pose[3], pose[4], pose[5], pose[6]);
    if (quaternion.norm() < 1e-9) {
        throw std::runtime_error("invalid zero pose quaternion");
    }
    return quaternion.normalized().toRotationMatrix();
}

inline Eigen::Vector3d PosePosition(const std::array<double, 7>& pose)
{
    return Eigen::Vector3d(pose[0], pose[1], pose[2]);
}

inline Eigen::Vector3d RotationError(
    const Eigen::Matrix3d& current, const Eigen::Matrix3d& desired)
{
    // World-frame geometric orientation error, continuous near zero.
    return 0.5
           * (current.col(0).cross(desired.col(0))
               + current.col(1).cross(desired.col(1))
               + current.col(2).cross(desired.col(2)));
}

struct LoopTarget {
    Eigen::Vector3d position;
    Eigen::Vector3d linear_velocity;
    Eigen::Vector3d linear_acceleration;
    Eigen::Matrix3d rotation;
    Eigen::Vector3d angular_velocity;
    Eigen::Vector3d angular_acceleration;
};

inline Eigen::Matrix3d LoopRotationFromPhase(const Eigen::Matrix3d& initial_rotation,
    double phase, double orientation_amplitude_rad)
{
    // The distal wrist axes are approximately flange Y and moving -Z.  Give
    // those two directions the dominant, different-frequency components so a
    // 9-DoF controller has a visible reason to use both wrist motors.  The
    // smaller X component still exercises the residual arm-only orientation.
    const Eigen::AngleAxisd tilt_x(0.10 * orientation_amplitude_rad * std::sin(phase),
        Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd tilt_y(orientation_amplitude_rad * std::sin(2.0 * phase),
        Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd tilt_z(0.85 * orientation_amplitude_rad * std::sin(3.0 * phase),
        Eigen::Vector3d::UnitZ());
    return initial_rotation * tilt_x.toRotationMatrix() * tilt_y.toRotationMatrix()
           * tilt_z.toRotationMatrix();
}

inline Eigen::Vector3d LoopAngularVelocityPerPhase(
    const Eigen::Matrix3d& initial_rotation, double phase,
    double orientation_amplitude_rad)
{
    constexpr double epsilon = 1e-5;
    const Eigen::Matrix3d minus = LoopRotationFromPhase(
        initial_rotation, phase - epsilon, orientation_amplitude_rad);
    const Eigen::Matrix3d plus = LoopRotationFromPhase(
        initial_rotation, phase + epsilon, orientation_amplitude_rad);
    return RotationError(minus, plus) / (2.0 * epsilon);
}

inline LoopTarget LoopTargetFromPhase(const Eigen::Vector3d& origin,
    const Eigen::Matrix3d& initial_rotation, double phase, double phase_rate,
    double phase_acceleration, double radius_m, double orientation_amplitude_rad)
{
    const Eigen::Vector3d delta(radius_m * (std::cos(phase) - 1.0),
        0.65 * radius_m * std::sin(phase), 0.35 * radius_m * std::sin(2.0 * phase));
    const Eigen::Vector3d d_delta_d_phase(-radius_m * std::sin(phase),
        0.65 * radius_m * std::cos(phase), 0.70 * radius_m * std::cos(2.0 * phase));
    const Eigen::Vector3d d2_delta_d_phase2(-radius_m * std::cos(phase),
        -0.65 * radius_m * std::sin(phase), -1.40 * radius_m * std::sin(2.0 * phase));
    const Eigen::Vector3d velocity = d_delta_d_phase * phase_rate;
    const Eigen::Vector3d acceleration
        = d2_delta_d_phase2 * phase_rate * phase_rate
          + d_delta_d_phase * phase_acceleration;
    const Eigen::Matrix3d rotation
        = LoopRotationFromPhase(initial_rotation, phase, orientation_amplitude_rad);
    const Eigen::Vector3d omega_per_phase = LoopAngularVelocityPerPhase(
        initial_rotation, phase, orientation_amplitude_rad);
    constexpr double epsilon = 1e-5;
    const Eigen::Vector3d domega_dphase
        = (LoopAngularVelocityPerPhase(initial_rotation, phase + epsilon,
               orientation_amplitude_rad)
              - LoopAngularVelocityPerPhase(initial_rotation, phase - epsilon,
                  orientation_amplitude_rad))
          / (2.0 * epsilon);
    const Eigen::Vector3d angular_velocity = omega_per_phase * phase_rate;
    const Eigen::Vector3d angular_acceleration
        = domega_dphase * phase_rate * phase_rate
          + omega_per_phase * phase_acceleration;
    return {origin + delta, velocity, acceleration, rotation, angular_velocity,
        angular_acceleration};
}

inline LoopTarget ClosedLoopTarget(const Eigen::Vector3d& origin,
    const Eigen::Matrix3d& initial_rotation, double time_s, double radius_m,
    double frequency_hz, double orientation_amplitude_rad)
{
    const double omega = 2.0 * kPi * frequency_hz;
    const double phase = omega * time_s;
    // Starts exactly at the measured endpoint and closes after one period.
    return LoopTargetFromPhase(origin, initial_rotation, phase, omega, 0.0,
        radius_m, orientation_amplitude_rad);
}

inline LoopTarget ContinuousLoopTarget(const Eigen::Vector3d& origin,
    const Eigen::Matrix3d& initial_rotation, double elapsed_s, double radius_m,
    double frequency_hz, double orientation_amplitude_rad)
{
    if (frequency_hz <= 0.0) {
        throw std::invalid_argument("loop frequency must be positive");
    }
    const double omega = 2.0 * kPi * frequency_hz;
    const double period_s = 1.0 / frequency_hz;
    // Enter the periodic orbit without a velocity step.  The phase speed uses
    // a quintic 0->1 ramp whose integral is analytic; after the ramp the
    // target continues at constant phase speed forever.
    const double ramp_s = std::min(2.0, 0.25 * period_s);
    if (elapsed_s < ramp_s) {
        const double u = std::clamp(elapsed_s / ramp_s, 0.0, 1.0);
        const double speed_scale
            = 10.0 * std::pow(u, 3) - 15.0 * std::pow(u, 4)
              + 6.0 * std::pow(u, 5);
        const double ds_du = 30.0 * u * u - 60.0 * std::pow(u, 3)
                             + 30.0 * std::pow(u, 4);
        const double integrated_speed
            = 2.5 * std::pow(u, 4) - 3.0 * std::pow(u, 5)
              + std::pow(u, 6);
        return LoopTargetFromPhase(origin, initial_rotation,
            omega * ramp_s * integrated_speed, omega * speed_scale,
            omega * ds_du / ramp_s, radius_m, orientation_amplitude_rad);
    }
    const double phase
        = omega * (elapsed_s - ramp_s) + 0.5 * omega * ramp_s;
    return LoopTargetFromPhase(origin, initial_rotation, phase, omega, 0.0,
        radius_m, orientation_amplitude_rad);
}

inline LoopTarget OrientationTargetFromPhase(const Eigen::Vector3d& fixed_position,
    const Eigen::Matrix3d& initial_rotation, double phase, double phase_rate,
    double phase_acceleration, double orientation_amplitude_rad)
{
    const Eigen::Matrix3d rotation
        = LoopRotationFromPhase(initial_rotation, phase, orientation_amplitude_rad);
    const Eigen::Vector3d omega_per_phase = LoopAngularVelocityPerPhase(
        initial_rotation, phase, orientation_amplitude_rad);
    constexpr double epsilon = 1e-5;
    const Eigen::Vector3d domega_dphase
        = (LoopAngularVelocityPerPhase(initial_rotation, phase + epsilon,
               orientation_amplitude_rad)
              - LoopAngularVelocityPerPhase(initial_rotation, phase - epsilon,
                  orientation_amplitude_rad))
          / (2.0 * epsilon);
    return {fixed_position, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        rotation, omega_per_phase * phase_rate,
        domega_dphase * phase_rate * phase_rate
            + omega_per_phase * phase_acceleration};
}

inline LoopTarget ContinuousOrientationTarget(
    const Eigen::Vector3d& fixed_position,
    const Eigen::Matrix3d& initial_rotation, double elapsed_s,
    double frequency_hz, double orientation_amplitude_rad)
{
    if (frequency_hz <= 0.0) {
        throw std::invalid_argument("orientation loop frequency must be positive");
    }
    const double omega = 2.0 * kPi * frequency_hz;
    const double period_s = 1.0 / frequency_hz;
    const double ramp_s = std::min(2.0, 0.25 * period_s);
    if (elapsed_s < ramp_s) {
        const double u = std::clamp(elapsed_s / ramp_s, 0.0, 1.0);
        const double speed_scale
            = 10.0 * std::pow(u, 3) - 15.0 * std::pow(u, 4)
              + 6.0 * std::pow(u, 5);
        const double ds_du = 30.0 * u * u - 60.0 * std::pow(u, 3)
                             + 30.0 * std::pow(u, 4);
        const double integrated_speed
            = 2.5 * std::pow(u, 4) - 3.0 * std::pow(u, 5)
              + std::pow(u, 6);
        return OrientationTargetFromPhase(fixed_position, initial_rotation,
            omega * ramp_s * integrated_speed, omega * speed_scale,
            omega * ds_du / ramp_s, orientation_amplitude_rad);
    }
    const double phase
        = omega * (elapsed_s - ramp_s) + 0.5 * omega * ramp_s;
    return OrientationTargetFromPhase(fixed_position, initial_rotation, phase,
        omega, 0.0, orientation_amplitude_rad);
}

struct RoundedRectangleState {
    Eigen::Vector2d position;
    Eigen::Vector2d tangent;
    double curvature;
};

inline RoundedRectangleState RoundedRectangleAtArcLength(
    double arc_length, double width_m, double height_m, double corner_radius_m)
{
    const double straight_x = width_m - 2.0 * corner_radius_m;
    const double straight_y = height_m - 2.0 * corner_radius_m;
    const double quarter_arc = 0.5 * kPi * corner_radius_m;
    const double perimeter
        = 2.0 * straight_x + 2.0 * straight_y + 4.0 * quarter_arc;
    double s = std::fmod(arc_length, perimeter);
    if (s < 0.0) s += perimeter;
    const double half_width = 0.5 * width_m;
    const double half_height = 0.5 * height_m;
    const Eigen::Vector2d start(-half_width + corner_radius_m, -half_height);
    Eigen::Vector2d position;
    Eigen::Vector2d tangent;
    double curvature = 0.0;

    auto arc = [&](const Eigen::Vector2d& center, double start_angle,
                   double local_length) {
        const double angle = start_angle + local_length / corner_radius_m;
        position = center
                   + corner_radius_m
                         * Eigen::Vector2d(std::cos(angle), std::sin(angle));
        tangent = Eigen::Vector2d(-std::sin(angle), std::cos(angle));
        curvature = 1.0 / corner_radius_m;
    };

    if (s < straight_x) {
        position = start + Eigen::Vector2d(s, 0.0);
        tangent = Eigen::Vector2d::UnitX();
    } else if ((s -= straight_x) < quarter_arc) {
        arc({half_width - corner_radius_m, -half_height + corner_radius_m},
            -0.5 * kPi, s);
    } else if ((s -= quarter_arc) < straight_y) {
        position = {half_width, -half_height + corner_radius_m + s};
        tangent = Eigen::Vector2d::UnitY();
    } else if ((s -= straight_y) < quarter_arc) {
        arc({half_width - corner_radius_m, half_height - corner_radius_m}, 0.0, s);
    } else if ((s -= quarter_arc) < straight_x) {
        position = {half_width - corner_radius_m - s, half_height};
        tangent = -Eigen::Vector2d::UnitX();
    } else if ((s -= straight_x) < quarter_arc) {
        arc({-half_width + corner_radius_m, half_height - corner_radius_m},
            0.5 * kPi, s);
    } else if ((s -= quarter_arc) < straight_y) {
        position = {-half_width, half_height - corner_radius_m - s};
        tangent = -Eigen::Vector2d::UnitY();
    } else {
        s -= straight_y;
        arc({-half_width + corner_radius_m, -half_height + corner_radius_m},
            kPi, s);
    }
    return {position - start, tangent, curvature};
}

inline LoopTarget RoundedRectangleTargetFromPhase(const Eigen::Vector3d& origin,
    const Eigen::Matrix3d& initial_rotation, double phase, double phase_rate,
    double phase_acceleration, double width_m, double height_m,
    double corner_radius_m, int tangent_axis)
{
    const double straight_x = width_m - 2.0 * corner_radius_m;
    const double straight_y = height_m - 2.0 * corner_radius_m;
    const double perimeter = 2.0 * straight_x + 2.0 * straight_y
                             + 2.0 * kPi * corner_radius_m;
    const double arc_per_phase = perimeter / (2.0 * kPi);
    const auto path = RoundedRectangleAtArcLength(
        phase * arc_per_phase, width_m, height_m, corner_radius_m);

    Eigen::Vector3d path_x(initial_rotation(0, tangent_axis),
        initial_rotation(1, tangent_axis), 0.0);
    if (path_x.norm() < 0.1) {
        throw std::runtime_error(
            "selected tangent tool axis has almost no XY projection at startup");
    }
    path_x.normalize();
    const Eigen::Vector3d path_y = Eigen::Vector3d::UnitZ().cross(path_x);
    const Eigen::Vector3d tangent_world
        = path.tangent.x() * path_x + path.tangent.y() * path_y;
    const Eigen::Vector3d normal_world
        = Eigen::Vector3d::UnitZ().cross(tangent_world);
    const double speed = arc_per_phase * phase_rate;
    const double tangential_acceleration = arc_per_phase * phase_acceleration;
    const double heading = std::atan2(path.tangent.y(), path.tangent.x());
    const double heading_rate = path.curvature * speed;
    const double heading_acceleration = path.curvature * tangential_acceleration;

    return {
        origin + path.position.x() * path_x + path.position.y() * path_y,
        speed * tangent_world,
        tangential_acceleration * tangent_world
            + path.curvature * speed * speed * normal_world,
        Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ()).toRotationMatrix()
            * initial_rotation,
        heading_rate * Eigen::Vector3d::UnitZ(),
        heading_acceleration * Eigen::Vector3d::UnitZ(),
    };
}

inline LoopTarget ContinuousRoundedRectangleTarget(const Eigen::Vector3d& origin,
    const Eigen::Matrix3d& initial_rotation, double elapsed_s, double width_m,
    double height_m, double corner_radius_m, double frequency_hz, int tangent_axis)
{
    if (frequency_hz <= 0.0) {
        throw std::invalid_argument("rectangle loop frequency must be positive");
    }
    const double omega = 2.0 * kPi * frequency_hz;
    const double period_s = 1.0 / frequency_hz;
    const double ramp_s = std::min(2.0, 0.25 * period_s);
    if (elapsed_s < ramp_s) {
        const double u = std::clamp(elapsed_s / ramp_s, 0.0, 1.0);
        const double speed_scale
            = 10.0 * std::pow(u, 3) - 15.0 * std::pow(u, 4)
              + 6.0 * std::pow(u, 5);
        const double ds_du = 30.0 * u * u - 60.0 * std::pow(u, 3)
                             + 30.0 * std::pow(u, 4);
        const double integrated_speed
            = 2.5 * std::pow(u, 4) - 3.0 * std::pow(u, 5)
              + std::pow(u, 6);
        return RoundedRectangleTargetFromPhase(origin, initial_rotation,
            omega * ramp_s * integrated_speed, omega * speed_scale,
            omega * ds_du / ramp_s, width_m, height_m, corner_radius_m,
            tangent_axis);
    }
    const double phase
        = omega * (elapsed_s - ramp_s) + 0.5 * omega * ramp_s;
    return RoundedRectangleTargetFromPhase(origin, initial_rotation, phase, omega,
        0.0, width_m, height_m, corner_radius_m, tangent_axis);
}

inline LoopTarget SingleSmoothLoopTarget(const Eigen::Vector3d& origin,
    const Eigen::Matrix3d& initial_rotation, double elapsed_s, double duration_s,
    double radius_m, double orientation_amplitude_rad)
{
    const double u = std::clamp(elapsed_s / duration_s, 0.0, 1.0);
    const double smooth = 10.0 * std::pow(u, 3) - 15.0 * std::pow(u, 4)
                          + 6.0 * std::pow(u, 5);
    const double ds_du = 30.0 * u * u - 60.0 * std::pow(u, 3)
                         + 30.0 * std::pow(u, 4);
    const double d2s_du2 = 60.0 * u - 180.0 * u * u + 120.0 * std::pow(u, 3);
    return LoopTargetFromPhase(origin, initial_rotation, 2.0 * kPi * smooth,
        2.0 * kPi * ds_du / duration_s,
        2.0 * kPi * d2s_du2 / (duration_s * duration_s), radius_m,
        orientation_amplitude_rad);
}

struct OscResult {
    Eigen::VectorXd torque;
    Eigen::MatrixXd lambda;
    Eigen::VectorXd task_error;
};

inline OscResult OperationalSpaceTorque(const Eigen::MatrixXd& mass,
    const Eigen::MatrixXd& jacobian, const Eigen::VectorXd& task_error,
    const Eigen::VectorXd& task_velocity_error,
    const Eigen::VectorXd& desired_acceleration, const Eigen::VectorXd& null_error,
    const Eigen::VectorXd& joint_velocity, double kp_translation,
    double kd_translation, double kp_rotation, double kd_rotation,
    double kp_null, double kd_null, double lambda_damping)
{
    const int dof = static_cast<int>(mass.rows());
    if (mass.cols() != dof || jacobian.rows() != 6 || jacobian.cols() != dof
        || task_error.size() != 6 || task_velocity_error.size() != 6
        || desired_acceleration.size() != 6 || null_error.size() != dof
        || joint_velocity.size() != dof) {
        throw std::invalid_argument("OSC matrix/vector dimensions are inconsistent");
    }
    Eigen::LDLT<Eigen::MatrixXd> mass_solver(mass);
    if (mass_solver.info() != Eigen::Success) {
        throw std::runtime_error("joint mass matrix factorization failed");
    }
    const Eigen::MatrixXd minv_jt = mass_solver.solve(jacobian.transpose());
    Eigen::Matrix<double, 6, 6> inverse_lambda = jacobian * minv_jt;
    inverse_lambda.diagonal().array() += lambda_damping * lambda_damping;
    Eigen::LDLT<Eigen::Matrix<double, 6, 6>> lambda_solver(inverse_lambda);
    if (lambda_solver.info() != Eigen::Success) {
        throw std::runtime_error("operational inertia factorization failed");
    }
    const Eigen::Matrix<double, 6, 6> lambda
        = lambda_solver.solve(Eigen::Matrix<double, 6, 6>::Identity());

    Eigen::Matrix<double, 6, 1> acceleration_command = desired_acceleration;
    acceleration_command.head<3>()
        += kp_translation * task_error.head<3>()
           + kd_translation * task_velocity_error.head<3>();
    acceleration_command.tail<3>()
        += kp_rotation * task_error.tail<3>()
           + kd_rotation * task_velocity_error.tail<3>();
    const Eigen::VectorXd task_torque
        = jacobian.transpose() * lambda * acceleration_command;

    const Eigen::MatrixXd jbar = minv_jt * lambda;
    const Eigen::MatrixXd null_projector
        = Eigen::MatrixXd::Identity(dof, dof) - jbar * jacobian;
    const Eigen::VectorXd posture_torque
        = kp_null * null_error - kd_null * joint_velocity;
    return {task_torque + null_projector.transpose() * posture_torque, lambda,
        task_error};
}

inline Eigen::VectorXd RateAndMagnitudeLimit(const Eigen::VectorXd& requested,
    const Eigen::VectorXd& previous, const Eigen::VectorXd& absolute_limit,
    const Eigen::VectorXd& rate_limit_nm_s)
{
    if (requested.size() != previous.size() || requested.size() != absolute_limit.size()
        || requested.size() != rate_limit_nm_s.size()) {
        throw std::invalid_argument("torque limiter dimensions differ");
    }
    Eigen::VectorXd result = requested;
    for (Eigen::Index i = 0; i < result.size(); ++i) {
        const double maximum_step = rate_limit_nm_s[i] * kDt;
        result[i] = std::clamp(result[i], previous[i] - maximum_step,
            previous[i] + maximum_step);
        result[i] = std::clamp(result[i], -absolute_limit[i], absolute_limit[i]);
    }
    return result;
}

inline void WriteDryRunCsv(const std::string& path, double duration_s,
    double radius_m, double frequency_hz, double orientation_amplitude_rad)
{
    std::ofstream output(path);
    if (!output) {
        throw std::runtime_error("cannot open dry-run CSV: " + path);
    }
    output << "time_s,x_m,y_m,z_m,tilt_x_deg,tilt_y_deg,tilt_z_deg\n";
    const Eigen::Vector3d origin = Eigen::Vector3d::Zero();
    const Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    for (double time_s = 0.0; time_s <= duration_s; time_s += 0.01) {
        const auto target = ClosedLoopTarget(origin, rotation, time_s, radius_m,
            frequency_hz, orientation_amplitude_rad);
        const double phase = 2.0 * kPi * frequency_hz * time_s;
        output << std::setprecision(10) << time_s << ',' << target.position.x() << ','
               << target.position.y() << ',' << target.position.z() << ','
               << 0.10 * orientation_amplitude_rad * std::sin(phase) * 180.0
                      / kPi << ','
               << orientation_amplitude_rad * std::sin(2.0 * phase) * 180.0
                      / kPi << ','
               << 0.85 * orientation_amplitude_rad * std::sin(3.0 * phase)
                      * 180.0 / kPi
               << '\n';
    }
}

inline void WriteRoundedRectangleDryRunCsv(const std::string& path,
    double duration_s, double width_m, double height_m, double corner_radius_m,
    int tangent_axis)
{
    std::ofstream output(path);
    if (!output) throw std::runtime_error("cannot open rectangle dry-run CSV");
    output << "time_s,x_m,y_m,z_m,tangent_heading_deg\n";
    const Eigen::Vector3d origin = Eigen::Vector3d::Zero();
    const Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    const int steps = static_cast<int>(std::ceil(duration_s / 0.01));
    for (int index = 0; index <= steps; ++index) {
        const double time_s = std::min(duration_s, index * 0.01);
        const auto target = RoundedRectangleTargetFromPhase(origin, rotation,
            2.0 * kPi * time_s / duration_s, 2.0 * kPi / duration_s, 0.0,
            width_m, height_m, corner_radius_m, tangent_axis);
        const double heading = std::atan2(
            target.linear_velocity.y(), target.linear_velocity.x());
        output << std::setprecision(10) << time_s << ',' << target.position.x()
               << ',' << target.position.y() << ',' << target.position.z() << ','
               << heading * 180.0 / kPi << '\n';
    }
}

inline void WriteOrientationDryRunCsv(const std::string& path,
    double duration_s, double orientation_amplitude_rad)
{
    std::ofstream output(path);
    if (!output) throw std::runtime_error("cannot open orientation dry-run CSV");
    output << "time_s,x_m,y_m,z_m,tilt_x_deg,tilt_y_deg,tilt_z_deg\n";
    const Eigen::Vector3d fixed_position = Eigen::Vector3d::Zero();
    const Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    const int steps = static_cast<int>(std::ceil(duration_s / 0.01));
    for (int index = 0; index <= steps; ++index) {
        const double time_s = std::min(duration_s, index * 0.01);
        const double phase = 2.0 * kPi * time_s / duration_s;
        const auto target = OrientationTargetFromPhase(fixed_position, rotation,
            phase, 2.0 * kPi / duration_s, 0.0,
            orientation_amplitude_rad);
        output << std::setprecision(10) << time_s << ',' << target.position.x()
               << ',' << target.position.y() << ',' << target.position.z() << ','
               << 0.10 * orientation_amplitude_rad * std::sin(phase) * 180.0
                      / kPi << ','
               << orientation_amplitude_rad * std::sin(2.0 * phase) * 180.0
                      / kPi << ','
               << 0.85 * orientation_amplitude_rad * std::sin(3.0 * phase)
                      * 180.0 / kPi
               << '\n';
    }
}

} // namespace teleop::osc
