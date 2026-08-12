#include "chai3d.h"
#include "dhdc.h"
#include "drdc.h"

#include <cerrno>
#include <cmath>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>

namespace {

std::atomic<bool> g_running {true};

void HandleSignal(int)
{
    g_running = false;
}

struct Options {
    unsigned int device_index {0};
    unsigned int rate_hz {250};
    unsigned int max_samples {0};
    unsigned int feedback_watchdog_ms {100};
    double force_slew_rate_n_per_s {0.0};
    double teleop_damping_n_per_mps {0.0};
    bool gravity_compensation {true};
    bool hold_when_released {false};
    bool startup_center {false};
    unsigned int hold_switch {0};
    double hold_stiffness_n_per_m {150.0};
    double hold_damping_n_per_mps {5.0};
    double hold_max_force_n {4.0};
    double hold_slew_rate_n_per_s {20.0};
    bool probe_only {false};
    bool calibrate {false};
};

void PrintUsage(const char* program)
{
    std::cout << "用法: " << program
              << " [--probe] [--device INDEX] [--rate HZ] [--samples N] [--calibrate]\n"
              << "       [--feedback-watchdog-ms MS] [--force-slew-rate N_PER_S]\n"
              << "       [--teleop-damping N_PER_MPS]\n"
              << "       [--gravity-compensation|--no-gravity-compensation]\n"
              << "       [--hold-when-released] [--startup-center] [--hold-switch INDEX]\n"
              << "       [--hold-stiffness N_PER_M] [--hold-damping N_PER_MPS]\n"
              << "       [--hold-max-force N] [--hold-slew-rate N_PER_S]\n"
              << "\n"
              << "默认输出空格分隔的数据帧：\n"
              << "timestamp_ns px py pz r00 r01 r02 r10 r11 r12 r20 r21 r22 switches\n"
              << "stdin 力反馈命令：F fx fy fz（设备坐标系，单位 N）；Z 立即归零\n";
}

unsigned int ParseUnsigned(const std::string& text, const std::string& option)
{
    std::size_t consumed = 0;
    const unsigned long parsed = std::stoul(text, &consumed);
    if (consumed != text.size()) {
        throw std::invalid_argument(option + " 需要无符号整数");
    }
    return static_cast<unsigned int>(parsed);
}

double ParseNonnegativeDouble(const std::string& text, const std::string& option)
{
    std::size_t consumed = 0;
    const double parsed = std::stod(text, &consumed);
    if (consumed != text.size() || !std::isfinite(parsed) || parsed < 0.0) {
        throw std::invalid_argument(option + " 需要非负有限数值");
    }
    return parsed;
}

Options ParseOptions(int argc, char** argv)
{
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            PrintUsage(argv[0]);
            std::exit(EXIT_SUCCESS);
        }
        if (arg == "--probe") {
            options.probe_only = true;
            continue;
        }
        if (arg == "--calibrate") {
            options.calibrate = true;
            continue;
        }
        if (arg == "--gravity-compensation") {
            options.gravity_compensation = true;
            continue;
        }
        if (arg == "--no-gravity-compensation") {
            options.gravity_compensation = false;
            continue;
        }
        if (arg == "--hold-when-released") {
            options.hold_when_released = true;
            continue;
        }
        if (arg == "--startup-center") {
            options.startup_center = true;
            continue;
        }
        if ((arg == "--device" || arg == "--rate" || arg == "--samples"
                || arg == "--feedback-watchdog-ms" || arg == "--force-slew-rate"
                || arg == "--teleop-damping"
                || arg == "--hold-switch" || arg == "--hold-stiffness"
                || arg == "--hold-damping" || arg == "--hold-max-force"
                || arg == "--hold-slew-rate")
            && i + 1 >= argc) {
            throw std::invalid_argument(arg + " 缺少参数");
        }
        if (arg == "--device") {
            options.device_index = ParseUnsigned(argv[++i], arg);
            continue;
        }
        if (arg == "--rate") {
            options.rate_hz = ParseUnsigned(argv[++i], arg);
            if (options.rate_hz < 10 || options.rate_hz > 2000) {
                throw std::invalid_argument("--rate 必须在 10 到 2000 Hz 之间");
            }
            continue;
        }
        if (arg == "--samples") {
            options.max_samples = ParseUnsigned(argv[++i], arg);
            if (options.max_samples == 0) {
                throw std::invalid_argument("--samples 必须大于 0");
            }
            continue;
        }
        if (arg == "--feedback-watchdog-ms") {
            options.feedback_watchdog_ms = ParseUnsigned(argv[++i], arg);
            if (options.feedback_watchdog_ms < 20 || options.feedback_watchdog_ms > 1000) {
                throw std::invalid_argument("--feedback-watchdog-ms 必须在 20 到 1000 ms 之间");
            }
            continue;
        }
        if (arg == "--force-slew-rate") {
            options.force_slew_rate_n_per_s = ParseNonnegativeDouble(argv[++i], arg);
            continue;
        }
        if (arg == "--teleop-damping") {
            options.teleop_damping_n_per_mps =
                ParseNonnegativeDouble(argv[++i], arg);
            continue;
        }
        if (arg == "--hold-switch") {
            options.hold_switch = ParseUnsigned(argv[++i], arg);
            if (options.hold_switch >= 32) {
                throw std::invalid_argument("--hold-switch 必须在 0 到 31 之间");
            }
            continue;
        }
        if (arg == "--hold-stiffness") {
            options.hold_stiffness_n_per_m = ParseNonnegativeDouble(argv[++i], arg);
            if (options.hold_stiffness_n_per_m <= 0.0) {
                throw std::invalid_argument("--hold-stiffness 必须大于 0");
            }
            continue;
        }
        if (arg == "--hold-damping") {
            options.hold_damping_n_per_mps = ParseNonnegativeDouble(argv[++i], arg);
            continue;
        }
        if (arg == "--hold-max-force") {
            options.hold_max_force_n = ParseNonnegativeDouble(argv[++i], arg);
            if (options.hold_max_force_n <= 0.0) {
                throw std::invalid_argument("--hold-max-force 必须大于 0");
            }
            continue;
        }
        if (arg == "--hold-slew-rate") {
            options.hold_slew_rate_n_per_s = ParseNonnegativeDouble(argv[++i], arg);
            continue;
        }
        throw std::invalid_argument("未知参数: " + arg);
    }
    if (options.startup_center && !options.hold_when_released) {
        throw std::invalid_argument("--startup-center 必须与 --hold-when-released 一起使用");
    }
    return options;
}

void PrintDevice(unsigned int index, const chai3d::cHapticDeviceInfo& info)
{
    std::cout << "device=" << index << " model=\"" << info.m_modelName
              << "\" manufacturer=\"" << info.m_manufacturerName
              << "\" position=" << info.m_sensedPosition
              << " rotation=" << info.m_sensedRotation
              << " gripper=" << info.m_sensedGripper
              << " force=" << info.m_actuatedPosition
              << " torque=" << info.m_actuatedRotation
              << " workspace_radius_m=" << info.m_workspaceRadius << '\n';
}

class OpenDevice {
public:
    explicit OpenDevice(chai3d::cGenericHapticDevicePtr device)
        : device_(std::move(device))
    {
        if (!device_ || !device_->open()) {
            throw std::runtime_error("无法打开 CHAI3D 设备");
        }
    }

    ~OpenDevice()
    {
        if (device_) {
            // cDeltaDevice::close() delegates to drdClose()/dhdClose(), which
            // stops the device. Do not re-enable FORCE mode after the DRD
            // guard has deliberately returned it to BRAKE mode.
            device_->close();
        }
    }

    chai3d::cGenericHapticDevicePtr get() const { return device_; }

private:
    chai3d::cGenericHapticDevicePtr device_;
};

class DrdPositionHold {
public:
    explicit DrdPositionHold(int device_id)
        : device_id_(device_id)
    {
        if (!drdIsSupported(static_cast<char>(device_id_))) {
            throw std::runtime_error("Force Dimension DRD 不支持当前设备");
        }
        if (!drdIsInitialized(static_cast<char>(device_id_))) {
            throw std::runtime_error("omega.7 尚未初始化，不能启动原生位置保持");
        }
        if (drdStart(static_cast<char>(device_id_)) < 0) {
            throw std::runtime_error(
                std::string("启动 Force Dimension DRD 控制器失败: ")
                + dhdErrorGetLastStr());
        }
        running_ = true;
        try {
            // Only the translational base is locked on clutch release. An
            // omega.7 wrist is sensed but passive, so calling its unavailable
            // DRD rotation regulator would fail with "operation not
            // available". Configure only axes that the detected model can
            // actively drive.
            if (dhdHasActiveWrist(static_cast<char>(device_id_))) {
                Check(drdRegulateRot(false, static_cast<char>(device_id_)),
                    "关闭 DRD wrist regulation 失败");
            }
            if (dhdHasActiveGripper(static_cast<char>(device_id_))) {
                Check(drdRegulateGrip(false, static_cast<char>(device_id_)),
                    "关闭 DRD gripper regulation 失败");
            }
            SetHolding(true);
        } catch (...) {
            drdStop(false, static_cast<char>(device_id_));
            running_ = false;
            throw;
        }
    }

    ~DrdPositionHold()
    {
        if (running_) {
            // Leave the device in BRAKE mode when this process exits.
            drdStop(false, static_cast<char>(device_id_));
        }
    }

    DrdPositionHold(const DrdPositionHold&) = delete;
    DrdPositionHold& operator=(const DrdPositionHold&) = delete;

    void SetHolding(bool holding)
    {
        if (holding == holding_) {
            return;
        }
        if (holding) {
            // Update the target before re-enabling regulation so the first
            // control tick cannot pull toward a stale pre-clutch position.
            Check(drdHold(static_cast<char>(device_id_)),
                "预锁存 omega.7 当前保持位置失败");
            Check(drdRegulatePos(true, static_cast<char>(device_id_)),
                "启用 DRD position regulation 失败");
            Check(drdHold(static_cast<char>(device_id_)),
                "锁存 omega.7 当前保持位置失败");
        } else {
            Check(drdRegulatePos(false, static_cast<char>(device_id_)),
                "释放 DRD position regulation 失败");
        }
        holding_ = holding;
    }

    void SendUserForce(const chai3d::cVector3d& force)
    {
        double command[DHD_MAX_DOF] {
            force.x(), force.y(), force.z(), 0.0, 0.0, 0.0, 0.0, 0.0
        };
        Check(drdSetForceAndTorqueAndGripperForce(
                  command, static_cast<char>(device_id_)),
            "通过 DRD 发送触觉力失败");
    }

    bool holding() const { return holding_; }
    double controlFrequencyHz() const
    {
        return drdGetCtrlFreq(static_cast<char>(device_id_));
    }

private:
    static void Check(int result, const char* message)
    {
        if (result < 0) {
            throw std::runtime_error(
                std::string(message) + ": " + dhdErrorGetLastStr());
        }
    }

    int device_id_ {-1};
    bool running_ {false};
    bool holding_ {false};
};

chai3d::cVector3d LimitNorm(const chai3d::cVector3d& vector, double maximum)
{
    const double norm = vector.length();
    if (norm <= maximum || norm <= 1e-12) {
        return vector;
    }
    return (maximum / norm) * vector;
}

chai3d::cVector3d LimitStep(
    const chai3d::cVector3d& current,
    const chai3d::cVector3d& target,
    double maximum_step)
{
    const chai3d::cVector3d delta = target - current;
    const double norm = delta.length();
    if (norm <= maximum_step || norm <= 1e-12) {
        return target;
    }
    return current + (maximum_step / norm) * delta;
}

void ReadFeedbackCommands(
    std::string& input_buffer,
    chai3d::cVector3d& requested_force,
    bool& feedback_received,
    std::chrono::steady_clock::time_point& last_feedback)
{
    char buffer[512];
    while (true) {
        const ssize_t count = ::read(STDIN_FILENO, buffer, sizeof(buffer));
        if (count > 0) {
            input_buffer.append(buffer, static_cast<std::size_t>(count));
            continue;
        }
        if (count == 0) {
            break;
        }
        if (errno == EINTR) {
            continue;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            break;
        }
        throw std::runtime_error("读取 stdin 力反馈命令失败");
    }

    std::size_t newline = 0;
    while ((newline = input_buffer.find('\n')) != std::string::npos) {
        const std::string line = input_buffer.substr(0, newline);
        input_buffer.erase(0, newline + 1);
        if (line.empty()) {
            continue;
        }
        if (line == "Z") {
            requested_force.zero();
            feedback_received = false;
            last_feedback = std::chrono::steady_clock::now();
            continue;
        }
        std::istringstream stream(line);
        char command = '\0';
        double fx = 0.0;
        double fy = 0.0;
        double fz = 0.0;
        std::string extra;
        if (!(stream >> command >> fx >> fy >> fz) || command != 'F'
            || (stream >> extra) || !std::isfinite(fx) || !std::isfinite(fy)
            || !std::isfinite(fz)) {
            throw std::runtime_error("无效 stdin 力反馈命令: " + line);
        }
        requested_force.set(fx, fy, fz);
        feedback_received = true;
        last_feedback = std::chrono::steady_clock::now();
    }
}

} // namespace

int main(int argc, char** argv)
{
    try {
        const Options options = ParseOptions(argc, argv);
        chai3d::cHapticDeviceHandler handler;
        const unsigned int count = handler.getNumDevices();

        if (options.probe_only) {
            std::cout << "count=" << count << '\n';
            for (unsigned int i = 0; i < count; ++i) {
                chai3d::cHapticDeviceInfo info;
                if (handler.getDeviceSpecifications(info, i)) {
                    PrintDevice(i, info);
                }
            }
            return count == 0 ? EXIT_FAILURE : EXIT_SUCCESS;
        }

        if (count == 0) {
            throw std::runtime_error("CHAI3D 没有检测到触觉设备；请先运行 --probe");
        }
        if (options.device_index >= count) {
            throw std::runtime_error("--device 超出已检测设备范围");
        }

        chai3d::cGenericHapticDevicePtr device;
        if (!handler.getDevice(device, options.device_index)) {
            throw std::runtime_error("无法获取指定 CHAI3D 设备");
        }
        OpenDevice opened(device);
        const auto info = opened.get()->getSpecifications();
        if (!info.m_sensedPosition) {
            throw std::runtime_error("指定设备不支持位置读取");
        }
        if (options.hold_when_released && !info.m_actuatedPosition) {
            throw std::runtime_error("指定设备不支持位置保持所需的力输出");
        }
        if (options.hold_stiffness_n_per_m > info.m_maxLinearStiffness) {
            throw std::runtime_error("位置保持 stiffness 超过设备报告的最大值");
        }
        if (options.hold_damping_n_per_mps > info.m_maxLinearDamping) {
            throw std::runtime_error("位置保持 damping 超过设备报告的最大值");
        }
        if (options.teleop_damping_n_per_mps > info.m_maxLinearDamping) {
            throw std::runtime_error("遥操作 damping 超过设备报告的最大值");
        }
        if (options.hold_max_force_n > info.m_maxLinearForce) {
            throw std::runtime_error("位置保持力上限超过设备连续额定力");
        }
        const int dhd_device_id = dhdGetDeviceID();
        if (dhd_device_id < 0) {
            throw std::runtime_error("无法获取 Force Dimension DHD device ID");
        }
        const bool initialized_before =
            drdIsInitialized(static_cast<char>(dhd_device_id));
        if (options.calibrate || !initialized_before) {
            std::cerr << "CHAI3D_CALIBRATION starting forced=" << options.calibrate
                      << " 请勿触碰设备，等待自动初始化完成\n";
        }
        // Every Force Dimension application must ensure that the device has
        // been initialized once after power-on. --calibrate forces a repeat;
        // otherwise CHAI3D/DRD only runs auto-init when it is actually needed.
        if (!opened.get()->calibrate(options.calibrate)) {
            throw std::runtime_error("CHAI3D 设备校准失败");
        }
        const bool initialized_after =
            drdIsInitialized(static_cast<char>(dhd_device_id));
        if (!initialized_after) {
            throw std::runtime_error("omega.7 自动初始化后仍未 initialized");
        }
        if (options.calibrate || !initialized_before) {
            std::cerr << "CHAI3D_CALIBRATION complete\n";
        }

        opened.get()->setEnableGripperUserSwitch(true);

        // CHAI3D does not expose this DHD setting. Configure it explicitly so a
        // zero user-force command means a weightless handle, not disabled motors.
        const int requested_gravity = options.gravity_compensation ? DHD_ON : DHD_OFF;
        if (dhdSetGravityCompensation(requested_gravity, dhd_device_id) < 0) {
            throw std::runtime_error("设置 DHD 重力补偿失败");
        }
        // A startup-center trajectory still uses the legacy bounded spring.
        // Normal clutch release uses the manufacturer's native DRD regulator.
        const bool use_drd_position_hold =
            options.hold_when_released && !options.startup_center;
        std::unique_ptr<DrdPositionHold> drd_position_hold;
        if (use_drd_position_hold) {
            drd_position_hold = std::make_unique<DrdPositionHold>(dhd_device_id);
            drd_position_hold->SendUserForce(chai3d::cVector3d(0.0, 0.0, 0.0));
        } else if (!opened.get()->setForceAndTorqueAndGripperForce(
                       chai3d::cVector3d(0.0, 0.0, 0.0),
                       chai3d::cVector3d(0.0, 0.0, 0.0),
                       0.0)) {
            throw std::runtime_error("启用 CHAI3D force mode 失败");
        }
        int dhd_status[DHD_MAX_STATUS] {};
        if (dhdGetStatus(dhd_status, dhd_device_id) < 0) {
            throw std::runtime_error("读取 DHD 状态失败");
        }
        const bool gravity_active = dhd_status[DHD_STATUS_GRAVITY] == DHD_ON;
        if (gravity_active != options.gravity_compensation) {
            throw std::runtime_error("DHD 重力补偿状态与请求不一致");
        }
        double effector_mass_kg = 0.0;
        if (dhdGetEffectorMass(&effector_mass_kg, dhd_device_id) < 0) {
            throw std::runtime_error("读取 DHD effector mass 失败");
        }
        double device_angle_deg = std::numeric_limits<double>::quiet_NaN();
        if (dhdGetDeviceAngleDeg(&device_angle_deg, dhd_device_id) < 0) {
            throw std::runtime_error("读取 DHD device mounting angle 失败");
        }
        double gripper_angle_deg = std::numeric_limits<double>::quiet_NaN();
        if (dhdHasGripper(dhd_device_id)
            && dhdGetGripperAngleDeg(&gripper_angle_deg, dhd_device_id) < 0) {
            throw std::runtime_error("读取 DHD gripper angle 失败");
        }
        std::cerr << "CHAI3D_READY device=" << options.device_index
                  << " model=\"" << info.m_modelName << "\""
                  << " rotation=" << info.m_sensedRotation
                  << " force=" << info.m_actuatedPosition
                  << " torque=" << info.m_actuatedRotation
                  << " max_force_N=" << info.m_maxLinearForce
                  << " max_torque_Nm=" << info.m_maxAngularTorque
                  << " gravity=" << gravity_active
                  << " effector_mass_kg=" << effector_mass_kg
                  << " device_angle_deg=" << device_angle_deg
                  << " gripper_angle_deg=" << gripper_angle_deg
                  << " force_enabled=" << dhd_status[DHD_STATUS_FORCE]
                  << " brake=" << dhd_status[DHD_STATUS_BRAKE]
                  << " force_off_cause=" << dhd_status[DHD_STATUS_FORCEOFFCAUSE]
                  << " initialized=" << initialized_after
                  << " hold=" << options.hold_when_released
                  << " hold_controller="
                  << (!options.hold_when_released
                          ? "gravity"
                          : (use_drd_position_hold ? "drd" : "spring"))
                  << " drd_rate_hz="
                  << (drd_position_hold
                          ? drd_position_hold->controlFrequencyHz()
                          : 0.0)
                  << " startup_center=" << options.startup_center
                  << " hold_stiffness_N_per_m=" << options.hold_stiffness_n_per_m
                  << " hold_damping_N_per_mps=" << options.hold_damping_n_per_mps
                  << " hold_max_force_N=" << options.hold_max_force_n
                  << " force_slew_N_per_s=" << options.force_slew_rate_n_per_s
                  << " teleop_damping_N_per_mps="
                  << options.teleop_damping_n_per_mps
                  << " rate_hz=" << options.rate_hz << '\n';

        std::signal(SIGINT, HandleSignal);
        std::signal(SIGTERM, HandleSignal);

        using Clock = std::chrono::steady_clock;
        const int stdin_flags = ::fcntl(STDIN_FILENO, F_GETFL, 0);
        if (stdin_flags < 0 || ::fcntl(STDIN_FILENO, F_SETFL, stdin_flags | O_NONBLOCK) < 0) {
            throw std::runtime_error("无法把 stdin 设置为 non-blocking 模式");
        }
        const auto period = std::chrono::nanoseconds(1'000'000'000ULL / options.rate_hz);
        auto next_tick = Clock::now();
        auto next_status = Clock::now();
        auto last_feedback = Clock::now();
        std::string feedback_input_buffer;
        chai3d::cVector3d requested_force(0.0, 0.0, 0.0);
        chai3d::cVector3d applied_feedback_force(0.0, 0.0, 0.0);
        chai3d::cVector3d applied_damping_force(0.0, 0.0, 0.0);
        chai3d::cVector3d applied_hold_force(0.0, 0.0, 0.0);
        chai3d::cVector3d hold_target(0.0, 0.0, 0.0);
        bool feedback_received = false;
        bool first_device_sample = true;
        bool switch_was_pressed = false;
        unsigned int samples_written = 0;
        std::cout << std::setprecision(17);

        while (g_running) {
            next_tick += period;
            ReadFeedbackCommands(
                feedback_input_buffer, requested_force, feedback_received, last_feedback);
            chai3d::cVector3d position;
            chai3d::cVector3d linear_velocity;
            chai3d::cMatrix3d rotation;
            rotation.identity();
            unsigned int switches = 0;

            if (!opened.get()->getPosition(position)) {
                throw std::runtime_error("CHAI3D 位置读取失败");
            }
            if (!opened.get()->getLinearVelocity(linear_velocity)) {
                throw std::runtime_error("CHAI3D 线速度读取失败");
            }
            if (info.m_sensedRotation && !opened.get()->getRotation(rotation)) {
                throw std::runtime_error("CHAI3D 姿态读取失败");
            }
            opened.get()->getUserSwitches(switches);
            const bool switch_pressed =
                (switches & (1U << options.hold_switch)) != 0U;
            if (first_device_sample) {
                hold_target = options.startup_center
                    ? chai3d::cVector3d(0.0, 0.0, 0.0)
                    : position;
                if (drd_position_hold) {
                    drd_position_hold->SetHolding(!switch_pressed);
                }
                first_device_sample = false;
            } else if (!switch_pressed && switch_was_pressed) {
                // Latch the exact device position at clutch release.
                hold_target = position;
                applied_hold_force.zero();
                if (drd_position_hold) {
                    drd_position_hold->SetHolding(true);
                }
            } else if (switch_pressed && !switch_was_pressed
                && drd_position_hold) {
                // While clutching, release only translation regulation. DRD
                // continues sending gravity compensation and user feedback.
                drd_position_hold->SetHolding(false);
            }

            const auto feedback_age = std::chrono::duration_cast<std::chrono::milliseconds>(
                Clock::now() - last_feedback);
            const bool feedback_active = info.m_actuatedPosition && switch_pressed
                && feedback_received
                && feedback_age.count() <= options.feedback_watchdog_ms;
            if (feedback_active) {
                const chai3d::cVector3d safe_force =
                    LimitNorm(requested_force, info.m_maxLinearForce);
                if (options.force_slew_rate_n_per_s > 0.0) {
                    applied_feedback_force = LimitStep(
                        applied_feedback_force,
                        safe_force,
                        options.force_slew_rate_n_per_s
                            / static_cast<double>(options.rate_hz));
                } else {
                    applied_feedback_force = safe_force;
                }
            } else {
                applied_feedback_force.zero();
            }

            if (switch_pressed && options.teleop_damping_n_per_mps > 0.0) {
                // Local 1 kHz viscous damping dissipates oscillation energy
                // introduced by the slower/delayed bilateral force loop. It
                // is zero at rest, so static force-feedback gain is unchanged.
                applied_damping_force =
                    -options.teleop_damping_n_per_mps * linear_velocity;
            } else {
                applied_damping_force.zero();
            }

            const bool hold_active = options.hold_when_released && !switch_pressed;
            if (hold_active && !drd_position_hold) {
                const chai3d::cVector3d desired_hold_force = LimitNorm(
                    options.hold_stiffness_n_per_m * (hold_target - position)
                        - options.hold_damping_n_per_mps * linear_velocity,
                    options.hold_max_force_n);
                if (options.hold_slew_rate_n_per_s > 0.0) {
                    applied_hold_force = LimitStep(
                        applied_hold_force,
                        desired_hold_force,
                        options.hold_slew_rate_n_per_s
                            / static_cast<double>(options.rate_hz));
                } else {
                    applied_hold_force = desired_hold_force;
                }
            } else {
                // Never make the operator fight the hold spring while clutching.
                applied_hold_force.zero();
            }
            const chai3d::cVector3d total_user_force = LimitNorm(
                applied_feedback_force + applied_damping_force
                    + applied_hold_force,
                info.m_maxLinearForce);
            if (info.m_actuatedPosition || info.m_actuatedRotation || info.m_actuatedGripper) {
                if (drd_position_hold) {
                    drd_position_hold->SendUserForce(total_user_force);
                } else if (!opened.get()->setForceAndTorqueAndGripperForce(
                               total_user_force,
                               chai3d::cVector3d(0.0, 0.0, 0.0),
                               0.0)) {
                    throw std::runtime_error("CHAI3D 力反馈输出失败");
                }
            }
            const auto now = Clock::now();
            if (now >= next_status) {
                int runtime_status[DHD_MAX_STATUS] {};
                if (dhdGetStatus(runtime_status, dhd_device_id) < 0) {
                    throw std::runtime_error("读取 DHD 运行状态失败");
                }
                double runtime_gripper_angle_deg =
                    std::numeric_limits<double>::quiet_NaN();
                if (dhdHasGripper(dhd_device_id)
                    && dhdGetGripperAngleDeg(
                           &runtime_gripper_angle_deg, dhd_device_id)
                        < 0) {
                    throw std::runtime_error("读取 DHD gripper angle 失败");
                }
                const unsigned int raw_button_mask = dhdGetButtonMask(dhd_device_id);
                std::cerr << "CHAI3D_STATUS force_enabled="
                          << runtime_status[DHD_STATUS_FORCE]
                          << " gravity=" << runtime_status[DHD_STATUS_GRAVITY]
                          << " brake=" << runtime_status[DHD_STATUS_BRAKE]
                          << " error=" << runtime_status[DHD_STATUS_ERROR]
                          << " force_off_cause="
                          << runtime_status[DHD_STATUS_FORCEOFFCAUSE]
                          << " raw_button_mask=0x" << std::hex << raw_button_mask
                          << " switch_mask=0x" << switches << std::dec
                          << " gripper_angle_deg=" << runtime_gripper_angle_deg
                          << " hold_active=" << hold_active
                          << " hold_controller="
                          << (!options.hold_when_released
                                  ? "gravity"
                                  : (drd_position_hold ? "drd" : "spring"))
                          << " hold_error_mm="
                          << 1000.0 * (hold_target - position).length()
                          << " hold_force_N=" << applied_hold_force.length()
                          << " feedback_force_N=" << applied_feedback_force.length()
                          << " damping_force_N=" << applied_damping_force.length()
                          << '\n';
                next_status = now + std::chrono::seconds(1);
            }
            switch_was_pressed = switch_pressed;

            const auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                Clock::now().time_since_epoch())
                                       .count();
            std::cout << timestamp << ' ' << position.x() << ' ' << position.y() << ' '
                      << position.z();
            for (int row = 0; row < 3; ++row) {
                for (int col = 0; col < 3; ++col) {
                    std::cout << ' ' << rotation(row, col);
                }
            }
            std::cout << ' ' << switches << '\n' << std::flush;
            ++samples_written;
            if (options.max_samples > 0 && samples_written >= options.max_samples) {
                break;
            }
            std::this_thread::sleep_until(next_tick);
        }

        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "错误: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
