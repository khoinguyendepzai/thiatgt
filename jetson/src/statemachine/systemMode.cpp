#include <statemachine/systemMode.hpp>

#include <stdexcept>

namespace statemachine {

const ModeConfig& get_mode_config(SystemMode mode) {
    // Không còn tự hành: dò làn và cảnh báo lệch làn chạy trên PC (main.py) từ ảnh cam bám làn gửi qua UDP.
    // DEFAULT: tắt ghi serial (xe đứng yên). Tắt ghi serial thì ThreadSerialHandler gửi lệnh dừng. Hai camera
    // (ThreadCameraHandler) luôn chạy, không có cờ. control.enabled: ThreadControlHandler (đang trống).
    // TCP BẬT ở DEFAULT / AUTO / MANUAL: nối sẵn tới main_master.py để nhận @vcb và @state. Ở DEFAULT lệnh @vcb vẫn
    // tới nhưng serial không ghi nên bánh không quay; bấm 3 (MANUAL) hoặc gửi @state:MANUAL;; thì xe mới chạy theo PC.
    // STOP tắt hết (chương trình sắp thoát).
    //                                    mode    serial{write, read}  network{udp, tcp}  control{enabled}
    static const ModeConfig defaultCfg{"default", {false, true},       {true, true},      {false}};
    static const ModeConfig autoCfg   {"auto",    {true,  true},       {true, true},      {true}};
    static const ModeConfig manualCfg {"manual",  {true,  true},       {true, true},      {false}};
    static const ModeConfig stopCfg   {"stop",    {false, true},       {false, false},    {false}};

    switch (mode) {
        case SystemMode::Default: return defaultCfg;
        case SystemMode::Auto:    return autoCfg;
        case SystemMode::Manual:  return manualCfg;
        case SystemMode::Stop:    return stopCfg;
    }
    throw std::invalid_argument("Unknown SystemMode");
}

std::string mode_to_string(SystemMode mode) {
    switch (mode) {
        case SystemMode::Default: return "DEFAULT";
        case SystemMode::Auto:    return "AUTO";
        case SystemMode::Manual:  return "MANUAL";
        case SystemMode::Stop:    return "STOP";
    }
    throw std::invalid_argument("Unknown SystemMode");
}

std::optional<SystemMode> mode_from_string(const std::string& name) {
    if (name == "DEFAULT") return SystemMode::Default;
    if (name == "AUTO")    return SystemMode::Auto;
    if (name == "MANUAL")  return SystemMode::Manual;
    if (name == "STOP")    return SystemMode::Stop;
    return std::nullopt;
}

const ModeConfig* find_mode_config(const std::string& modeName) {
    const auto mode = mode_from_string(modeName);
    return mode ? &get_mode_config(*mode) : nullptr;
}

} // namespace statemachine
