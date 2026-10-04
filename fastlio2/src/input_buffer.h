#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// The receiver owns immutable payloads; only the worker mutates a claimed scan.
// A candidate stays in the queue through conversion and IMU waiting, so the
// capacity includes that work and overflow can invalidate it before claiming IMU.
template <typename Payload, typename Imu>
class LioInputBuffer {
public:
    using Clock = std::chrono::steady_clock;
    struct Candidate {
        uint64_t id;
        double stamp;
        Payload payload;
        Clock::time_point received;
    };
    struct Stats {
        uint64_t received_scans = 0, received_imus = 0;
        uint64_t duplicate_scans = 0, duplicate_imus = 0;
        uint64_t dropped_scans = 0, invalid_scans = 0, claimed_scans = 0;
        uint64_t consumed_imus = 0;
        size_t pending = 0, imu_pending = 0, max_pending = 0, max_imu_pending = 0;
    };
    enum class ClaimStatus { Taken, Superseded, Stopped, NoImu };
    struct Claim {
        ClaimStatus status;
        std::vector<Imu> imus;
    };

    LioInputBuffer(size_t scan_capacity, size_t imu_capacity, double maximum_imu_gap)
        : scan_capacity_(scan_capacity), imu_capacity_(imu_capacity),
          maximum_imu_gap_(maximum_imu_gap) {
        if (!scan_capacity || !imu_capacity || !std::isfinite(maximum_imu_gap) || maximum_imu_gap <= 0)
            throw std::invalid_argument("Invalid input queue capacity or IMU gap");
    }

    bool pushImu(const Imu &imu) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stopping_) return false;
            ++stats_.received_imus;
            checkStamp(imu.time, last_imu_, "IMU");
            if (last_imu_ && imu.time == *last_imu_) {
                ++stats_.duplicate_imus;
                return false;
            }
            if (last_imu_ && imu.time - *last_imu_ > maximum_imu_gap_ + 1e-9)
                throw std::runtime_error("IMU source-time gap exceeds imu_max_gap_sec");
            if (imus_.size() >= imu_capacity_)
                throw std::runtime_error("IMU buffer overflow: refusing to discard propagation history");
            imus_.push_back(imu);
            last_imu_ = imu.time;
            stats_.max_imu_pending = std::max(stats_.max_imu_pending, imus_.size());
        }
        cv_.notify_one();
        return true;
    }

    bool pushScan(double stamp, Payload payload, Clock::time_point received = Clock::now()) {
        // Retired messages can own large arrays: destroy them after unlocking.
        std::deque<Candidate> retired;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stopping_) return false;
            ++stats_.received_scans;
            checkStamp(stamp, last_scan_, "LiDAR");
            if (last_scan_ && stamp == *last_scan_) {
                ++stats_.duplicate_scans;
                return false;
            }
            if (scans_.size() >= scan_capacity_) {
                stats_.dropped_scans += scans_.size();
                retired.swap(scans_);
            }
            scans_.push_back({++next_id_, stamp, std::move(payload), received});
            last_scan_ = stamp;
            stats_.max_pending = std::max(stats_.max_pending, scans_.size());
        }
        cv_.notify_one();
        return true;
    }

    std::optional<Candidate> waitCandidate() {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [&] {return stopping_ || !scans_.empty();});
        if (stopping_) return std::nullopt;
        return scans_.front();
    }

    Claim claim(const Candidate &candidate, double scan_end) {
        if (!std::isfinite(scan_end) || scan_end < candidate.stamp)
            throw std::runtime_error("Invalid scan end timestamp");
        std::optional<Candidate> retired;
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [&] {
            return stopping_ || !current(candidate) || (last_imu_ && *last_imu_ >= scan_end);
        });
        if (stopping_) return {ClaimStatus::Stopped, {}};
        if (!current(candidate)) return {ClaimStatus::Superseded, {}};
        Claim result{ClaimStatus::Taken, {}};
        // Construct the whole package before removing history. Allocation/copy
        // failure must not leave a partially consumed propagation interval.
        for (const auto &imu : imus_) {
            if (imu.time >= scan_end) break;
            result.imus.push_back(imu);
        }
        for (size_t i = 0; i < result.imus.size(); ++i)
            imus_.pop_front();
        retired.emplace(std::move(scans_.front()));
        scans_.pop_front();
        if (result.imus.empty()) {
            result.status = ClaimStatus::NoImu;
            ++stats_.invalid_scans;
        } else {
            ++stats_.claimed_scans;
            stats_.consumed_imus += result.imus.size();
        }
        lock.unlock();
        return result;
    }

    void reject(const Candidate &candidate) {
        std::optional<Candidate> retired;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!current(candidate)) return;
            retired.emplace(std::move(scans_.front()));
            scans_.pop_front();
            ++stats_.invalid_scans;
        }
        cv_.notify_one();
    }

    void stop(const std::string &error = {}) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
            if (error_.empty()) error_ = error;
        }
        cv_.notify_all();
    }
    bool stopped() const {std::lock_guard<std::mutex> lock(mutex_); return stopping_;}
    std::string error() const {std::lock_guard<std::mutex> lock(mutex_); return error_;}
    Stats stats() const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto snapshot = stats_;
        snapshot.pending = scans_.size();
        snapshot.imu_pending = imus_.size();
        return snapshot;
    }

private:
    static void checkStamp(double stamp, const std::optional<double> &last, const char *source) {
        if (!std::isfinite(stamp) || (last && stamp < *last))
            throw std::runtime_error(std::string(source) + " source timestamp is non-finite or moved backwards");
    }
    bool current(const Candidate &candidate) const {
        return !scans_.empty() && scans_.front().id == candidate.id;
    }
    const size_t scan_capacity_, imu_capacity_;
    const double maximum_imu_gap_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<Candidate> scans_;
    std::deque<Imu> imus_;
    std::optional<double> last_scan_, last_imu_;
    uint64_t next_id_ = 0;
    Stats stats_;
    bool stopping_ = false;
    std::string error_;
};
