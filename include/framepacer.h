#ifndef FRAMEPACER_H
#define FRAMEPACER_H

#include <QtGlobal>
#include <algorithm>
#include <cmath>

// Deadlines stay on the original time line. Late frames are skipped rather
// than queued, and integer-millisecond timer rounding never accumulates.
class FramePacer {
public:
    void reset(qint64 now, double fps)
    {
        if (!std::isfinite(fps) || fps <= 0) fps = 60;
        period = 1e9 / std::max(1.0, std::min(1000.0, fps));
        next = double(now) + period;
    }
    bool due(qint64 now) const { return double(now) >= next; }
    void advance(qint64 now)
    {
        if (due(now)) next += (std::floor((double(now) - next) / period) + 1) * period;
    }
    int delayMilliseconds(qint64 now) const
    {
        return std::max(1, std::min(1000, int(std::ceil((next - double(now)) / 1e6))));
    }
private:
    double period = 1e9 / 60.0, next = 0;
};

#endif
