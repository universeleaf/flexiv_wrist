#include "rt_osc_common.hpp"

#include <flexiv/rdk/model.hpp>
#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/scheduler.hpp>

#include <thread>

using namespace flexiv;
using namespace teleop::osc;

namespace {

struct Options {
    std::string robot_sn {"Rizon4s-062232"};
    std::string dry_run_csv {"/tmp/flexiv_7dof_osc_loop.csv"};
    std::string endpoint {"tcp"};
    std::string trajectory {"orientation"};
    Eigen::Vector3d pivot_offset_flange {0.0, 0.0, 0.112};
    double duration_s {20.0};
    double radius_m {0.030};
    double orientation_deg {3.0};
    double rectangle_width_m {0.060};
    double rectangle_height_m {0.040};
    double rectangle_corner_radius_m {0.010};
    int tangent_axis {0};
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
        if (arg == "--robot-sn") {
            result.robot_sn = value();
        } else if (arg == "--duration-s") {
            result.duration_s = std::stod(value());
        } else if (arg == "--radius-m") {
            result.radius_m = std::stod(value());
        } else if (arg == "--orientation-deg") {
            result.orientation_deg = std::stod(value());
        } else if (arg == "--trajectory") {
            result.trajectory = value();
        } else if (arg == "--rectangle-width-m") {
            result.rectangle_width_m = std::stod(value());
        } else if (arg == "--rectangle-height-m") {
            result.rectangle_height_m = std::stod(value());
        } else if (arg == "--rectangle-corner-radius-m") {
            result.rectangle_corner_radius_m = std::stod(value());
        } else if (arg == "--tangent-axis") {
            const std::string axis = value();
            if (axis == "x") result.tangent_axis = 0;
            else if (axis == "z") result.tangent_axis = 2;
            else throw std::invalid_argument("--tangent-axis must be x or z");
        } else if (arg == "--endpoint") {
            result.endpoint = value();
        } else if (arg == "--cpu-affinity") {
            result.cpu_affinity = std::stoi(value());
        } else if (arg == "--dry-run-csv") {
            result.dry_run_csv = value();
        } else if (arg == "--real-confirm") {
            result.real = value() == "RUN_7DOF_TORQUE_OSC";
            if (!result.real) {
                throw std::invalid_argument(
                    "real confirmation must equal RUN_7DOF_TORQUE_OSC");
            }
        } else if (arg == "-h" || arg == "--help") {
            std::cout
                << "7-DoF periodic RT torque OSC. --duration-s is one loop period; "
                   "real motion continues until Ctrl-C.\n"
                << "Real: --real-confirm RUN_7DOF_TORQUE_OSC [--robot-sn SN]\n";
            std::exit(0);
        } else {
            throw std::invalid_argument("unknown argument: " + arg);
        }
    }
    const double maximum_orientation_deg
        = result.trajectory == "spin" ? 60.0 : 45.0;
    if (result.duration_s < 8.0 || result.radius_m <= 0.0 || result.radius_m > 0.05
        || result.orientation_deg <= 0.0
        || result.orientation_deg > maximum_orientation_deg) {
        throw std::invalid_argument(
            "require loop period>=8 s, radius in (0,0.05] m, orientation in (0,45] deg (spin <=60 deg)");
    }
    if (result.trajectory != "orientation" && result.trajectory != "spin"
        && result.trajectory != "rectangle" && result.trajectory != "loop") {
        throw std::invalid_argument(
            "--trajectory must be orientation, spin, rectangle or loop");
    }
    if (result.rectangle_width_m <= 0.0 || result.rectangle_width_m > 0.15
        || result.rectangle_height_m <= 0.0 || result.rectangle_height_m > 0.15
        || result.rectangle_corner_radius_m <= 0.0
        || 2.0 * result.rectangle_corner_radius_m
               >= std::min(result.rectangle_width_m, result.rectangle_height_m)) {
        throw std::invalid_argument("invalid rounded rectangle dimensions");
    }
    if (result.endpoint != "tcp" && result.endpoint != "flange"
        && result.endpoint != "pivot") {
        throw std::invalid_argument("--endpoint must be pivot, tcp or flange");
    }
    return result;
}

struct Controller {
    rdk::Robot& robot;
    rdk::Model& model;
    Options options;
    Eigen::VectorXd q_reference;
    Eigen::VectorXd previous_torque;
    Eigen::Vector3d position_origin;
    Eigen::Matrix3d rotation_origin;
    std::chrono::steady_clock::time_point started;
    std::ofstream log;
    std::uint64_t cycle_count {0};
    bool first_cycle {true};

    void Step()
    {
        try {
            if (robot.fault() || !robot.operational()) {
                throw std::runtime_error("robot fault/non-operational during RT loop");
            }
            const auto& states = robot.states();
            const Eigen::Map<const Eigen::VectorXd> q(states.q.data(), states.q.size());
            const Eigen::Map<const Eigen::VectorXd> dq(
                states.dtheta.data(), states.dtheta.size());
            model.Update(states.q, states.dtheta);
            const Eigen::MatrixXd mass = model.M();
            const Eigen::VectorXd coriolis = model.c();
            const Eigen::MatrixXd flange_jacobian = model.J("flange");
            Eigen::MatrixXd jacobian = flange_jacobian;
            if (options.endpoint == "tcp") {
                const Eigen::Vector3d offset_world
                    = PosePosition(states.tcp_pose) - PosePosition(states.flange_pose);
                jacobian.topRows<3>()
                    = flange_jacobian.topRows<3>()
                      - Skew(offset_world) * flange_jacobian.bottomRows<3>();
            } else if (options.endpoint == "pivot") {
                const Eigen::Vector3d offset_world
                    = PoseRotation(states.flange_pose) * options.pivot_offset_flange;
                jacobian.topRows<3>()
                    = flange_jacobian.topRows<3>()
                      - Skew(offset_world) * flange_jacobian.bottomRows<3>();
            }

            const auto now = std::chrono::steady_clock::now();
            // Scheduler::Start() itself takes about two seconds on this RDK.
            // Start the smooth trajectory clock on the first actual 1 kHz
            // callback, otherwise the controller skips the entire ramp and
            // receives an immediate 15-degree target step.
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
            } else if (options.trajectory == "spin") {
                target = ContinuousSpinTarget(position_origin, rotation_origin,
                    elapsed, 1.0 / options.duration_s,
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
            Eigen::Vector3d position;
            Eigen::Matrix3d rotation;
            if (options.endpoint == "tcp") {
                position = PosePosition(states.tcp_pose);
                rotation = PoseRotation(states.tcp_pose);
            } else {
                const Eigen::Vector3d offset = options.endpoint == "pivot"
                                                   ? options.pivot_offset_flange
                                                   : Eigen::Vector3d::Zero();
                position = PosePosition(states.flange_pose)
                           + PoseRotation(states.flange_pose) * offset;
                rotation = PoseRotation(states.flange_pose);
            }
            const Eigen::VectorXd twist = jacobian * dq;

            Eigen::VectorXd error(6), velocity_error(6), desired_acceleration(6);
            error.head<3>() = target.position - position;
            error.tail<3>() = RotationError(rotation, target.rotation);
            velocity_error.head<3>() = target.linear_velocity - twist.head<3>();
            velocity_error.tail<3>() = target.angular_velocity - twist.tail<3>();
            desired_acceleration.head<3>() = target.linear_acceleration;
            desired_acceleration.tail<3>() = target.angular_acceleration;

            // Restored from the known-working published controller (commit
            // 2f1e763). Only the target generator is newer; the controller
            // gains, dynamic consistency and torque path are unchanged.
            auto result = OperationalSpaceTorque(mass, jacobian, error,
                velocity_error, desired_acceleration, q_reference - q, dq,
                100.0, 20.0, 70.0, 16.0, 8.0, 2.0, 0.03);
            Eigen::VectorXd absolute_limit(7), rate_limit(7);
            absolute_limit << 20.0, 20.0, 15.0, 15.0, 8.0, 8.0, 5.0;
            rate_limit << 200.0, 200.0, 150.0, 150.0, 80.0, 80.0, 50.0;
            // Flexiv adds gravity internally (first true below); model.c()
            // supplies Coriolis/centripetal feed-forward for computed torque.
            const Eigen::VectorXd torque = RateAndMagnitudeLimit(
                result.torque + coriolis, previous_torque, absolute_limit, rate_limit);
            previous_torque = torque;
            robot.StreamJointTorque(
                std::vector<double>(torque.data(), torque.data() + torque.size()), true, true);
            if (log && cycle_count++ % 10 == 0) {
                log << std::setprecision(10) << elapsed << ',' << error.head<3>().norm()
                    << ',' << error.tail<3>().norm() << ',' << torque.norm() << '\n';
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
        rdk::Robot robot(options.robot_sn);
        if (robot.fault() || !robot.operational()) {
            throw std::runtime_error(
                "robot must already be enabled, operational and fault-free; no auto-enable/clear");
        }
        rdk::Model model(robot);
        const auto initial_states = robot.states();
        if (initial_states.q.size() != 7) {
            throw std::runtime_error("7-DoF demo requires exactly seven arm joints");
        }
        const Eigen::Vector3d initial_position
            = options.endpoint == "tcp"
                  ? PosePosition(initial_states.tcp_pose)
                  : PosePosition(initial_states.flange_pose)
                        + PoseRotation(initial_states.flange_pose)
                              * (options.endpoint == "pivot"
                                      ? options.pivot_offset_flange
                                      : Eigen::Vector3d::Zero());
        const Eigen::Matrix3d initial_rotation
            = options.endpoint == "tcp" ? PoseRotation(initial_states.tcp_pose)
                                         : PoseRotation(initial_states.flange_pose);
        Controller controller {robot, model, options,
            Eigen::Map<const Eigen::VectorXd>(
                initial_states.q.data(), initial_states.q.size()),
            Eigen::VectorXd::Zero(7), initial_position,
            initial_rotation, std::chrono::steady_clock::now(),
            std::ofstream("/tmp/flexiv_7dof_osc_rt.csv")};
        controller.log << "time_s,position_error_m,orientation_error_rad,torque_norm_nm\n";

        rdk::Scheduler scheduler;
        scheduler.AddTask(std::bind(&Controller::Step, &controller), "7dof_osc_1khz",
            1, scheduler.max_priority(), options.cpu_affinity);
        // Scheduler construction/configuration needs RT privilege.  Complete
        // it before changing the robot control mode so a permission failure
        // leaves the robot in its previous mode.
        robot.SetTimelinessFailureLimit(1.0);
        std::cout << "7DOF_CONTROLLER restored=published_commit_2f1e763"
                  << " trajectory=" << options.trajectory << std::endl;
        robot.SwitchMode(rdk::Mode::RT_JOINT_TORQUE);
        controller.started = std::chrono::steady_clock::now();
        scheduler.Start();
        while (!g_stop) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        scheduler.Stop();
        robot.Stop();
        std::cout << "7DOF_OSC_STOPPED log=/tmp/flexiv_7dof_osc_rt.csv" << std::endl;
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "ERROR " << error.what() << std::endl;
        return 1;
    }
}
