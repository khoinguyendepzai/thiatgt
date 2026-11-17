#pragma once

#include <chrono>
#include <optional>

namespace utils {

// Giới hạn tần suất ghi log lỗi: allow() trả true lần đầu, sau đó tối đa một lần mỗi cooldown.
// KHÔNG thread-safe.
class RateLimiter {
public:
    using Clock = std::chrono::steady_clock;

    explicit RateLimiter(Clock::duration cooldown) : cooldown_(cooldown) {}

    bool allow() {
        const auto now = Clock::now();
        if (last_ && now - *last_ < cooldown_) return false;
        last_ = now;
        return true;
    }

private:
    Clock::duration                  cooldown_;
    std::optional<Clock::time_point> last_;
};

} // namespace utils
