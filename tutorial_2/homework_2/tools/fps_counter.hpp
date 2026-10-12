#pragma once

#include <cstddef>
#include <deque>
#include <chrono>

namespace tools
{
class FpsCounter
{
public:
    explicit FpsCounter(std::size_t window = 30) : window_(window) {}

    void tick()
    {
        auto now = std::chrono::steady_clock::now();
        if (!last_.time_since_epoch().count()) { last_ = now; return; }
        double dt = std::chrono::duration<double>(now - last_).count();
        last_ = now;
        if (dt > 0.0) {
            frames_.push_back(dt);
            if (frames_.size() > window_) frames_.pop_front();
        }
    }

    double fps() const
    {
        if (frames_.size() < 2) return 0.0;
        double total = 0.0;
        for (double dt : frames_) total += dt;
        return static_cast<double>(frames_.size()) / total;
    }

private:
    std::size_t window_;
    std::deque<double> frames_;
    std::chrono::steady_clock::time_point last_{};
};
}  // namespace tools