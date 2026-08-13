#pragma once

#include "wrist_shared_memory.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <unistd.h>

namespace teleop::osc {

constexpr std::uint64_t kTeleopSharedMagic = 0x54454c454f504f53ULL; // "TELEOPOS"

enum class TeleopControlMode : std::uint32_t {
    Hold = 0,
    Arm7Dof = 1,
    ArmWrist9Dof = 2,
    OrientationForce = 3,
};

struct TeleopCommand {
    TeleopControlMode mode {TeleopControlMode::Hold};
    bool enabled {false};
    std::array<double, 7> target_pose {};
    std::array<double, 3> target_linear_velocity {};
    std::array<double, 3> target_angular_velocity {};
    double target_force_z_n {0.0};
};

struct TeleopState {
    std::array<double, 7> probe_pose {};
    std::array<double, 6> external_wrench_world {};
    std::array<double, 2> wrist_q_rad {};
    TeleopControlMode mode {TeleopControlMode::Hold};
    std::uint32_t status_flags {0};
    double probe_force_z_n {0.0};
    double position_error_m {0.0};
    double orientation_error_rad {0.0};
    double force_error_n {0.0};
    std::array<double, 3> arm_orientation_error_world {};
    double arm_singularity_sigma_min {0.0};
    double arm_motion_scale {1.0};
};

struct alignas(8) TeleopSharedBlock {
    std::uint64_t magic;
    std::uint64_t command_seq;
    std::uint64_t command_time_ns;
    std::uint32_t command_mode;
    std::uint32_t command_enabled;
    double target_pose[7];
    double target_linear_velocity[3];
    double target_angular_velocity[3];
    double target_force_z_n;
    std::uint64_t state_seq;
    std::uint64_t state_time_ns;
    double probe_pose[7];
    double external_wrench_world[6];
    double wrist_q_rad[2];
    std::uint32_t controller_mode;
    std::uint32_t status_flags;
    double probe_force_z_n;
    double position_error_m;
    double orientation_error_rad;
    double force_error_n;
    double arm_orientation_error_world[3];
    double arm_singularity_sigma_min;
    double arm_motion_scale;
};
static_assert(sizeof(TeleopSharedBlock) == 360, "Python/C++ teleop ABI changed");
static_assert(offsetof(TeleopSharedBlock, command_mode) == 24);
static_assert(offsetof(TeleopSharedBlock, target_pose) == 32);
static_assert(offsetof(TeleopSharedBlock, target_force_z_n) == 136);
static_assert(offsetof(TeleopSharedBlock, state_seq) == 144);
static_assert(offsetof(TeleopSharedBlock, probe_pose) == 160);
static_assert(offsetof(TeleopSharedBlock, external_wrench_world) == 216);
static_assert(offsetof(TeleopSharedBlock, controller_mode) == 280);
static_assert(offsetof(TeleopSharedBlock, probe_force_z_n) == 288);
static_assert(offsetof(TeleopSharedBlock, arm_orientation_error_world) == 320);
static_assert(offsetof(TeleopSharedBlock, arm_motion_scale) == 352);

class TeleopSharedMemory {
public:
    explicit TeleopSharedMemory(const std::string& path)
    {
        fd_ = ::open(path.c_str(), O_RDWR);
        if (fd_ < 0) {
            throw std::runtime_error("cannot open teleop shared memory file: " + path);
        }
        void* mapping = ::mmap(nullptr, sizeof(TeleopSharedBlock),
            PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
        if (mapping == MAP_FAILED) {
            ::close(fd_);
            throw std::runtime_error("cannot mmap teleop shared memory file");
        }
        block_ = static_cast<TeleopSharedBlock*>(mapping);
        if (block_->magic != kTeleopSharedMagic) {
            throw std::runtime_error("teleop shared memory magic mismatch");
        }
    }

    ~TeleopSharedMemory()
    {
        if (block_) ::munmap(block_, sizeof(TeleopSharedBlock));
        if (fd_ >= 0) ::close(fd_);
    }

    TeleopSharedMemory(const TeleopSharedMemory&) = delete;
    TeleopSharedMemory& operator=(const TeleopSharedMemory&) = delete;

    bool ReadCommand(TeleopCommand& command, std::uint64_t& age_ns) const
    {
        for (int attempt = 0; attempt < 3; ++attempt) {
            const std::uint64_t first
                = __atomic_load_n(&block_->command_seq, __ATOMIC_ACQUIRE);
            if (first == 0 || (first & 1U) != 0U) continue;
            const std::uint32_t mode = block_->command_mode;
            const std::uint32_t enabled = block_->command_enabled;
            std::memcpy(command.target_pose.data(), block_->target_pose,
                sizeof(block_->target_pose));
            std::memcpy(command.target_linear_velocity.data(),
                block_->target_linear_velocity,
                sizeof(block_->target_linear_velocity));
            std::memcpy(command.target_angular_velocity.data(),
                block_->target_angular_velocity,
                sizeof(block_->target_angular_velocity));
            command.target_force_z_n = block_->target_force_z_n;
            const std::uint64_t timestamp = block_->command_time_ns;
            __atomic_thread_fence(__ATOMIC_ACQUIRE);
            const std::uint64_t second
                = __atomic_load_n(&block_->command_seq, __ATOMIC_RELAXED);
            if (first != second) continue;
            if (mode > static_cast<std::uint32_t>(TeleopControlMode::OrientationForce)) {
                return false;
            }
            command.mode = static_cast<TeleopControlMode>(mode);
            command.enabled = enabled != 0;
            const auto now = MonotonicNanoseconds();
            age_ns = now >= timestamp ? now - timestamp : 0;
            return true;
        }
        return false;
    }

    void WriteState(const TeleopState& state)
    {
        std::uint64_t sequence
            = __atomic_load_n(&block_->state_seq, __ATOMIC_RELAXED);
        if ((sequence & 1U) != 0U) ++sequence;
        __atomic_store_n(&block_->state_seq, sequence + 1, __ATOMIC_RELEASE);
        block_->state_time_ns = MonotonicNanoseconds();
        std::memcpy(block_->probe_pose, state.probe_pose.data(),
            sizeof(block_->probe_pose));
        std::memcpy(block_->external_wrench_world,
            state.external_wrench_world.data(),
            sizeof(block_->external_wrench_world));
        std::memcpy(block_->wrist_q_rad, state.wrist_q_rad.data(),
            sizeof(block_->wrist_q_rad));
        block_->controller_mode = static_cast<std::uint32_t>(state.mode);
        block_->status_flags = state.status_flags;
        block_->probe_force_z_n = state.probe_force_z_n;
        block_->position_error_m = state.position_error_m;
        block_->orientation_error_rad = state.orientation_error_rad;
        block_->force_error_n = state.force_error_n;
        std::memcpy(block_->arm_orientation_error_world,
            state.arm_orientation_error_world.data(),
            sizeof(block_->arm_orientation_error_world));
        block_->arm_singularity_sigma_min
            = state.arm_singularity_sigma_min;
        block_->arm_motion_scale = state.arm_motion_scale;
        __atomic_thread_fence(__ATOMIC_RELEASE);
        __atomic_store_n(&block_->state_seq, sequence + 2, __ATOMIC_RELEASE);
    }

private:
    int fd_ {-1};
    TeleopSharedBlock* block_ {nullptr};
};

} // namespace teleop::osc
