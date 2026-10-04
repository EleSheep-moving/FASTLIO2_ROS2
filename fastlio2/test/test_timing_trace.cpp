#include "timing_trace.h"
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <unistd.h>

int main() {
    TimingTrace disabled("");
    for (int i = 0; i < 100000; ++i) disabled.record({"imu", 1, 2.0});
    if (disabled.dropped() != 0 || !disabled.flush()) return 1;
    const std::string path = "/tmp/fastlio-trace-" + std::to_string(getpid()) + ".csv";
    {
        TimingTrace trace(path, 2);
        trace.record({"received", 1, 2.0});
        trace.record({"published", 1, 2.1});
        trace.record({"overflow", 2, 2.2});
        if (trace.dropped() != 1 || !trace.flush()) return 2;
        if (!trace.flush()) return 3;
    }
    std::ifstream input(path);
    std::string line;
    int records = 0;
    while (std::getline(input, line)) {
        if (line.rfind("received,", 0) == 0 || line.rfind("published,", 0) == 0) ++records;
        if (line.rfind("overflow,", 0) == 0) return 4;
    }
    std::remove(path.c_str());
    if (records != 2) return 5;
    std::cout << "PASS disabled/bounded/idempotent timing trace\n";
    return 0;
}
