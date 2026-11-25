#pragma once

#include <chrono>
#include <cstdint>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>

// Mọi thời gian / tần suất log và công cụ chống spam log gom về đây.
// Muốn đổi chu kỳ log chỉ sửa file này.
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

namespace logcfg {

using std::chrono::milliseconds;
using std::chrono::seconds;

// --- Thống kê định kỳ (đo theo thời gian) ---------------------------------------------------------
// main: dòng [Status] state/speed/...
inline constexpr seconds kMainStatusPeriod{1};

// --- Log debug theo số frame (chỉ khi debugging) --------------------------------------------------
// Khoảng 1 lần/giây ở 30 fps; đổi fps thì chu kỳ thật đổi theo.
inline constexpr std::uint64_t kCameraLogEveryFrames = 100;
inline constexpr std::uint64_t kUdpFrameLogEvery     = 100;

// --- Log debug đường lệnh lái PC -> TCP -> gateway -> ThreadWrite -> Nucleo ------------------------
// PC gửi #vcb 20-50 lần/s nên mỗi điểm log tối đa một dòng mỗi khoảng này (kèm số gói đã qua). Đặt 0 để in hết.
inline constexpr milliseconds kCommDebugPeriod{200};

// --- Chống spam log lỗi (utils::RateLimiter) -------------------------------------------------------
// Một lỗi cùng loại tối đa một lần mỗi khoảng này.
inline constexpr seconds kErrorCooldown{3};

// --- Hàm định dạng dùng khi log ---------------------------------------------------------------------
// Dùng ostringstream riêng: không đổi trạng thái định dạng của logger dùng chung (std::cout / std::cerr).

// fps -> "12.3fps", mặc định 1 chữ số thập phân. Dùng ở dòng [Status] trong main.
inline std::string format_fps(double fps, int precision = 1) {
    std::ostringstream s;
    s << std::fixed << std::setprecision(precision) << fps << "fps";
    return s.str();
}

} // namespace logcfg
} // namespace utils
