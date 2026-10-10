#include "input_buffer.h"
#include <atomic>
#include <future>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <stdexcept>
#include <thread>

struct Imu { double time; };
using Buffer = LioInputBuffer<int, Imu>;
void require(bool value, const char *message) {
    if (!value) throw std::runtime_error(message);
}
template <typename F> void throws(F &&action) {
    bool caught = false;
    try { action(); } catch (const std::runtime_error &) { caught = true; }
    require(caught, "expected runtime error");
}
void overflow_preserves_imu() {
    Buffer buffer(2, 4096);
    for (int i = 0; i <= 60; ++i) buffer.pushImu({i * 0.005});
    buffer.pushScan(0.0, 1);
    auto old = buffer.waitCandidate();
    buffer.pushScan(0.1, 2);
    buffer.pushScan(0.2, 3);
    require(buffer.stats().pending == 1, "overflow must retain only newest");
    require(buffer.stats().dropped_scans == 2, "both old candidates counted once");
    require(buffer.claim(*old, 0.1).status == Buffer::ClaimStatus::Superseded,
            "converted old candidate must not claim IMU");
    auto newest = buffer.waitCandidate();
    auto claim = buffer.claim(*newest, 0.3);
    require(claim.status == Buffer::ClaimStatus::Taken, "newest complete scan taken");
    require(claim.imus.size() == 60, "all intervening IMU must be retained");
    require(claim.imus.front().time == 0.0, "first IMU retained across drops");
    require(buffer.stats().imu_pending == 1, "scan-end IMU belongs to next package");
}
void wait_releases_lock_and_stop_wakes() {
    Buffer buffer(2, 4096);
    buffer.pushScan(0.0, 1);
    auto candidate = buffer.waitCandidate();
    auto future = std::async(std::launch::async, [&] {return buffer.claim(*candidate, 0.1);});
    require(future.wait_for(std::chrono::milliseconds(20)) == std::future_status::timeout,
            "must wait for actual coverage");
    for (int i = 0; i <= 20; ++i) buffer.pushImu({i * 0.005});
    bool ready = future.wait_for(std::chrono::seconds(1)) == std::future_status::ready;
    if (!ready) buffer.stop();
    require(ready,
            "receiver must enqueue while worker waits");
    require(future.get().imus.size() == 20, "coverage package size");
    auto waiting = std::async(std::launch::async, [&] {return buffer.waitCandidate();});
    buffer.stop();
    require(waiting.wait_for(std::chrono::seconds(1)) == std::future_status::ready,
            "empty queue stop must wake worker");
    require(!waiting.get(), "stop must not return task");
}
void stale_candidate_wakes_without_consuming() {
    Buffer buffer(2, 4096);
    buffer.pushImu({0.0}); buffer.pushScan(0.0, 1);
    auto candidate = buffer.waitCandidate();
    auto future = std::async(std::launch::async, [&] {return buffer.claim(*candidate, 0.1);});
    require(future.wait_for(std::chrono::milliseconds(20)) == std::future_status::timeout,
            "claim should be blocked on IMU before superseding");
    buffer.pushScan(0.1, 2); buffer.pushScan(0.2, 3);
    bool ready = future.wait_for(std::chrono::seconds(1)) == std::future_status::ready;
    if (!ready) buffer.stop();
    require(ready,
            "superseding scan must wake pending claim");
    require(future.get().status == Buffer::ClaimStatus::Superseded, "stale claim result");
    require(buffer.stats().imu_pending == 1, "stale candidate cannot consume IMU");
}
void duplicates_and_faults() {
    Buffer buffer(2, 2);
    require(buffer.pushImu({0.0}), "first IMU");
    require(!buffer.pushImu({0.0}), "duplicate IMU rejected");
    require(buffer.pushScan(0.0, 1), "first scan");
    require(!buffer.pushScan(0.0, 2), "duplicate scan rejected");
    require(buffer.stats().duplicate_scans == 1 && buffer.stats().duplicate_imus == 1,
            "duplicate counters must distinguish sources");
    throws([&] {buffer.pushScan(-0.1, 3);});
    throws([&] {buffer.pushScan(std::numeric_limits<double>::quiet_NaN(), 3);});
    throws([&] {buffer.pushImu({std::numeric_limits<double>::quiet_NaN()});});
    Buffer backwards(2, 2);
    backwards.pushImu({0.1}); throws([&] {backwards.pushImu({0.09});});
    Buffer overflow(2, 2);
    overflow.pushImu({0.0}); overflow.pushImu({0.005});
    throws([&] {overflow.pushImu({0.01});});
    require(overflow.stats().imu_pending == 2, "overflow must never discard IMU");
    buffer.stop("explicit failure");
    require(buffer.error() == "explicit failure", "fault identity retained");
}
void gap_and_burst_preserve_history_and_wait_for_coverage() {
    Buffer buffer(2, 4096);
    const double stamps[] = {0.0, 0.005, 0.038572671, 0.038626202,
                             0.038642233, 0.038652888, 0.038663447, 0.038673879};
    for (double stamp : stamps) require(buffer.pushImu({stamp}), "gap/burst IMU accepted");
    buffer.pushScan(0.0, 1);
    auto candidate = buffer.waitCandidate();
    auto waiting = std::async(std::launch::async, [&] {return buffer.claim(*candidate, 0.1);});
    require(waiting.wait_for(std::chrono::milliseconds(20)) == std::future_status::timeout,
            "gap/burst must not bypass scan-end coverage");
    buffer.pushImu({0.1});
    bool ready = waiting.wait_for(std::chrono::seconds(1)) == std::future_status::ready;
    if (!ready) buffer.stop();
    require(ready, "scan-end coverage must wake claim after gap/burst");
    auto claim = waiting.get();
    require(claim.status == Buffer::ClaimStatus::Taken, "covered scan taken after gap/burst");
    require(claim.imus.size() == std::size(stamps), "gap/burst must retain every IMU");
    for (size_t i = 0; i < claim.imus.size(); ++i)
        require(claim.imus[i].time == stamps[i], "IMU order and timestamps preserved");
    require(buffer.stats().imu_pending == 1, "scan-end IMU remains for next package");
}
void stop_wakes_coverage_wait() {
    Buffer buffer(2, 4096);
    buffer.pushScan(1.0, 1);
    auto candidate = buffer.waitCandidate();
    auto waiting = std::async(std::launch::async, [&] {return buffer.claim(*candidate, 1.1);});
    require(waiting.wait_for(std::chrono::milliseconds(20)) == std::future_status::timeout,
            "claim initially waits for IMU");
    buffer.stop();
    require(waiting.wait_for(std::chrono::seconds(1)) == std::future_status::ready,
            "stop wakes coverage wait");
    require(waiting.get().status == Buffer::ClaimStatus::Stopped, "stopped claim status");
}
void retired_payload_destructs_outside_lock() {
    LioInputBuffer<std::shared_ptr<int>, Imu> buffer(1, 4096);
    int retired = 0;
    auto payload = std::shared_ptr<int>(new int(1), [&](int *value) {
        (void)buffer.stats();  // Would deadlock if the queue destroyed it under its lock.
        ++retired;
        delete value;
    });
    buffer.pushScan(1.0, std::move(payload));
    buffer.pushScan(1.1, std::make_shared<int>(2));
    require(retired == 1, "retired payload destructor completed outside buffer mutex");
}
void invalid_scan_retains_imu() {
    Buffer buffer(2, 4096);
    buffer.pushImu({0.0}); buffer.pushScan(0.0, 1);
    auto candidate = buffer.waitCandidate();
    buffer.reject(*candidate);
    require(buffer.stats().invalid_scans == 1, "invalid scan counted");
    require(buffer.stats().imu_pending == 1, "invalid scan retains IMU");
}
void invalid_scan_end_never_waits() {
    Buffer buffer(2, 4096);
    buffer.pushScan(1.0, 1);
    auto candidate = buffer.waitCandidate();
    auto waiting = std::async(std::launch::async, [&] {
        throws([&] {buffer.claim(*candidate, std::numeric_limits<double>::infinity());});
    });
    bool ready = waiting.wait_for(std::chrono::milliseconds(30)) == std::future_status::ready;
    buffer.stop();
    waiting.wait();
    require(ready, "invalid scan end must fail immediately rather than waiting forever");
    waiting.get();
    throws([&] {buffer.claim(*candidate, 0.5);});
}
void concurrent_receive_and_claim() {
    Buffer buffer(1000, 4096);
    std::atomic<int> claimed{0};
    std::thread worker([&] {
        while (auto candidate = buffer.waitCandidate()) {
            auto result = buffer.claim(*candidate, candidate->stamp + 0.025);
            if (result.status == Buffer::ClaimStatus::Taken) ++claimed;
        }
    });
    for (int i = 0; i <= 1000; ++i) {
        buffer.pushImu({i * 0.005});
        if (i % 20 == 0 && i < 1000) buffer.pushScan(i * 0.005, i);
    }
    for (int retry = 0; claimed < 50 && retry < 1000; ++retry)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    buffer.stop(); worker.join();
    require(claimed == 50, "all FIFO scans must be processed under concurrent receive");
    require(buffer.stats().dropped_scans == 0, "no non-overload drops");
}
int main() {
    const std::pair<const char *, void (*)()> tests[] = {
        {"overflow_preserves_imu", overflow_preserves_imu},
        {"wait_releases_lock_and_stop_wakes", wait_releases_lock_and_stop_wakes},
        {"stale_candidate_wakes_without_consuming", stale_candidate_wakes_without_consuming},
        {"duplicates_and_faults", duplicates_and_faults},
        {"gap_and_burst_preserve_history_and_wait_for_coverage", gap_and_burst_preserve_history_and_wait_for_coverage},
        {"invalid_scan_retains_imu", invalid_scan_retains_imu},
        {"invalid_scan_end_never_waits", invalid_scan_end_never_waits},
        {"stop_wakes_coverage_wait", stop_wakes_coverage_wait},
        {"retired_payload_destructs_outside_lock", retired_payload_destructs_outside_lock},
        {"concurrent_receive_and_claim", concurrent_receive_and_claim}};
    int failures = 0;
    for (const auto &test : tests) {
        try {test.second(); std::cout << "PASS " << test.first << '\n';}
        catch (const std::exception &error) {
            ++failures; std::cerr << "FAIL " << test.first << ": " << error.what() << '\n';
        }
    }
    return failures ? 1 : 0;
}
