#ifndef TIMINGSTATS_H
#define TIMINGSTATS_H

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>

struct TimingSummary {
    unsigned samples = 0;
    double medianMs = 0, p95Ms = 0, p99Ms = 0, maximumMs = 0;
};

// Bounded recent history; recording never allocates or sorts. Summarize only
// when a consumer needs diagnostics, outside the drawing/stepping hot paths.
class TimingWindow {
public:
    void add(double milliseconds)
    {
        if (!std::isfinite(milliseconds) || milliseconds < 0) return;
        values[next] = milliseconds;
        next = (next + 1) % values.size();
        count = std::min(count + 1, unsigned(values.size()));
    }
    TimingSummary summary() const
    {
        TimingSummary result;
        result.samples = count;
        if (!count) return result;
        auto sorted = values;
        std::sort(sorted.begin(), sorted.begin() + count);
        result.medianMs = sorted[(count - 1) / 2];
        result.p95Ms = sorted[unsigned(std::ceil(count * 0.95)) - 1];
        result.p99Ms = sorted[unsigned(std::ceil(count * 0.99)) - 1];
        result.maximumMs = sorted[count - 1];
        return result;
    }
private:
    std::array<double, 240> values{};
    unsigned count = 0, next = 0;
};

struct FrameTimings {
    TimingSummary interval, scene, labels, paintAndSwap, gpu;
    bool gpuRequested = false, gpuSupported = false;
    std::uint64_t frames = 0;
};

#endif
