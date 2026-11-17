#pragma once

#include <optional>
#include <string>

namespace statemachine {

enum class SystemMode {
    Default,
    Auto,
    Manual,
    Stop
};

struct SerialHandlerConfig {
    bool enabledWrite;   // ThreadWrite chạy hay pause
    bool enabledRead;    // ThreadRead chạy hay pause
};

struct NetworkHandlerConfig {
    bool enabledUdp;   // ThreadUdpServer chạy (đăng ký + gửi) hay pause (hủy đăng ký)
    bool enabledTcp;   // ThreadTcpClient chạy (kết nối + nhận) hay pause (đóng kết nối)
};

// Không còn bám làn / đổi làn. ThreadControlHandler chỉ ghi nhận cờ này, phần điều khiển để trống (code sau).
struct ControlConfig {
    bool enabled;
};

// Cấu hình của một mode; mỗi module đọc trường của mình khi nhận StateChange.
// THÊM CẤU HÌNH MODULE MỚI Ở ĐÂY (và thêm giá trị cho từng mode trong systemMode.cpp)
struct ModeConfig {
    std::string         mode;           // "default", "auto", ...
    SerialHandlerConfig serialHandler;
    NetworkHandlerConfig networkHandler;
    ControlConfig       control;
};

const ModeConfig& get_mode_config(SystemMode mode);

// "DEFAULT", "AUTO", "MANUAL", "STOP"
std::string mode_to_string(SystemMode mode);

// Tên -> mode, nullopt nếu tên không hợp lệ
std::optional<SystemMode> mode_from_string(const std::string& name);

// Cấu hình theo tên mode (giá trị của StateChange), nullptr nếu tên không hợp lệ. Trỏ tới dữ liệu static.
const ModeConfig* find_mode_config(const std::string& modeName);

} // namespace statemachine
