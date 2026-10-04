#pragma once
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

// Experimental tracing is bounded and performs no file I/O on receiver/worker
// paths. Flush only after those threads have stopped. Empty path disables it.
class TimingTrace {
public:
    struct Event {
        const char *kind;
        uint64_t id = 0;
        double stamp = 0;
        int64_t steady_ns = 0, ros_ns = 0;
        size_t pending = 0, imu_count = 0;
        double core_ms = 0, output_ms = 0;
        size_t iterations = 0, points = 0, map_points = 0;
    };
    explicit TimingTrace(const std::string &path, size_t capacity = 65536)
        : enabled_(!path.empty()), capacity_(capacity) {
        if (!enabled_) return;
        if (!capacity) throw std::invalid_argument("Trace capacity must be positive");
        events_.reserve(capacity);
        output_.open(path, std::ios::out | std::ios::trunc);
        if (!output_) throw std::runtime_error("Cannot open timing_trace_path: " + path);
    }
    bool enabled() const {return enabled_;}
    void record(const Event &event) {
        if (!enabled_) return;
        std::lock_guard<std::mutex> lock(mutex_);
        if (flushed_ || events_.size() == capacity_) {++dropped_; return;}
        events_.push_back(event);
    }
    uint64_t dropped() const {std::lock_guard<std::mutex> lock(mutex_); return dropped_;}
    bool flush() {
        if (!enabled_) return true;
        std::lock_guard<std::mutex> lock(mutex_);
        if (flushed_) return static_cast<bool>(output_);
        output_ << "event,id,stamp,steady_ns,ros_ns,pending,imu_count,core_ms,output_ms,iterations,points,map_points\n"
                << std::setprecision(17);
        for (const auto &e : events_)
            output_ << e.kind << ',' << e.id << ',' << e.stamp << ',' << e.steady_ns << ','
                    << e.ros_ns << ',' << e.pending << ',' << e.imu_count << ',' << e.core_ms << ','
                    << e.output_ms << ',' << e.iterations << ',' << e.points << ',' << e.map_points << '\n';
        output_ << "trace_dropped," << dropped_ << ",0,0,0,0,0,0,0,0,0,0\n";
        output_.flush();
        flushed_ = true;
        return static_cast<bool>(output_);
    }
private:
    const bool enabled_;
    const size_t capacity_;
    mutable std::mutex mutex_;
    std::vector<Event> events_;
    std::ofstream output_;
    uint64_t dropped_ = 0;
    bool flushed_ = false;
};
