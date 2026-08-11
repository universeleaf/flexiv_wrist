#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace teleop::osc {

constexpr std::uint64_t kWristSharedMagic = 0x3957524953544f53ULL; // "9WRISTOS"

struct alignas(8) WristSharedBlock {
    std::uint64_t magic;
    std::uint64_t state_seq;
    std::uint64_t state_time_ns;
    double q_rad[2];
    double dq_rad_s[2];
    double measured_torque_nm[2];
    std::uint64_t command_seq;
    std::uint64_t command_time_ns;
    double command_torque_nm[2];
    std::uint32_t stop_requested;
    std::uint32_t reserved;
    double target_q_rad[2];
};
static_assert(sizeof(WristSharedBlock) == 128, "Python/C++ wrist ABI changed");

inline std::uint64_t MonotonicNanoseconds()
{
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

class WristSharedMemory {
public:
    explicit WristSharedMemory(const std::string& path)
    {
        fd_ = ::open(path.c_str(), O_RDWR);
        if (fd_ < 0) {
            throw std::runtime_error("cannot open wrist shared memory file: " + path);
        }
        void* mapping = ::mmap(
            nullptr, sizeof(WristSharedBlock), PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
        if (mapping == MAP_FAILED) {
            ::close(fd_);
            throw std::runtime_error("cannot mmap wrist shared memory file");
        }
        block_ = static_cast<WristSharedBlock*>(mapping);
        if (block_->magic != kWristSharedMagic) {
            throw std::runtime_error("wrist shared memory magic mismatch");
        }
    }

    ~WristSharedMemory()
    {
        if (block_) {
            block_->stop_requested = 1;
            ::munmap(block_, sizeof(WristSharedBlock));
        }
        if (fd_ >= 0) {
            ::close(fd_);
        }
    }

    WristSharedMemory(const WristSharedMemory&) = delete;
    WristSharedMemory& operator=(const WristSharedMemory&) = delete;

    bool ReadState(std::array<double, 2>& q, std::array<double, 2>& dq,
        std::array<double, 2>& torque, std::uint64_t& age_ns) const
    {
        for (int attempt = 0; attempt < 3; ++attempt) {
            const std::uint64_t first
                = __atomic_load_n(&block_->state_seq, __ATOMIC_ACQUIRE);
            if (first == 0) {
                return false;
            }
            if ((first & 1U) != 0U) {
                continue;
            }
            std::memcpy(q.data(), block_->q_rad, sizeof(block_->q_rad));
            std::memcpy(dq.data(), block_->dq_rad_s, sizeof(block_->dq_rad_s));
            std::memcpy(
                torque.data(), block_->measured_torque_nm, sizeof(block_->measured_torque_nm));
            const std::uint64_t timestamp = block_->state_time_ns;
            __atomic_thread_fence(__ATOMIC_ACQUIRE);
            const std::uint64_t second
                = __atomic_load_n(&block_->state_seq, __ATOMIC_RELAXED);
            if (first == second) {
                const auto now = MonotonicNanoseconds();
                age_ns = now >= timestamp ? now - timestamp : 0;
                cached_q_ = q;
                cached_dq_ = dq;
                cached_torque_ = torque;
                cached_timestamp_ns_ = timestamp;
                cache_valid_ = true;
                return true;
            }
        }
        // A state publication is only a few microseconds long. Reuse the last
        // coherent frame for this one 1 kHz tick instead of skipping a Flexiv
        // torque packet; the caller's age watchdog still applies.
        if (cache_valid_) {
            q = cached_q_;
            dq = cached_dq_;
            torque = cached_torque_;
            const auto now = MonotonicNanoseconds();
            age_ns = now >= cached_timestamp_ns_ ? now - cached_timestamp_ns_ : 0;
            return true;
        }
        return false;
    }

    void WriteTorque(const std::array<double, 2>& torque,
        const std::array<double, 2>& target_q = {
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN()})
    {
        std::uint64_t sequence
            = __atomic_load_n(&block_->command_seq, __ATOMIC_RELAXED);
        if ((sequence & 1U) != 0U) {
            ++sequence;
        }
        __atomic_store_n(&block_->command_seq, sequence + 1, __ATOMIC_RELEASE);
        block_->command_torque_nm[0] = torque[0];
        block_->command_torque_nm[1] = torque[1];
        block_->target_q_rad[0] = target_q[0];
        block_->target_q_rad[1] = target_q[1];
        block_->command_time_ns = MonotonicNanoseconds();
        __atomic_thread_fence(__ATOMIC_RELEASE);
        __atomic_store_n(&block_->command_seq, sequence + 2, __ATOMIC_RELEASE);
    }

    void RequestStop() { block_->stop_requested = 1; }

private:
    int fd_ {-1};
    WristSharedBlock* block_ {nullptr};
    mutable std::array<double, 2> cached_q_ {};
    mutable std::array<double, 2> cached_dq_ {};
    mutable std::array<double, 2> cached_torque_ {};
    mutable std::uint64_t cached_timestamp_ns_ {0};
    mutable bool cache_valid_ {false};
};

} // namespace teleop::osc
