#pragma once

#include <any>
#include <cstdint>
#include <string>

namespace utils::messages {

// Tên các message trên gateway; thông tin định tuyến trong get_message_info()
enum class MessageName {
    // Camera
    CameraFrame,       // ThreadCameraHandler: ảnh cam người lái mới trong frame_buffer()
    LaneCameraFrame,   // ThreadCameraHandler: ảnh cam bám làn ĐÃ NẮN MÉO mới trong lane_frame_buffer()

    // Khoảng cách từ đầu xe tới mặt trước vật cản phía trước (float, m), < 0 = không có vật cản.
    // Nguồn: máy remote chạy YOLO rồi gửi qua TCP (#distance). Hiện chưa ai dùng, code sau (dành cho ThreadControlHandler)
    Distance,

    // To Nucleo: lệnh lái xe từ ThreadTcpClient (#vcb), ThreadWrite ghi xuống serial
    Speed,
    Steer,
    Directory,

    // From Nucleo: ThreadRead đọc từ serial
    ImuData,
    CurrentSpeed,

    // From StateMachine
    StateChange
};

// Struct làm value của message. Chỉ dùng kiểu chuẩn (không OpenCV, không header module) vì file này được include
// ở khắp nơi. msgType = "struct".

// CameraFrame, LaneCameraFrame: báo có ảnh mới; ảnh nằm trong hardware::camera::FrameBuffer
struct FrameInfo {
    std::uint64_t seq       = 0;     // tăng dần mỗi frame mới, bắt đầu từ 1
    double        timestamp = 0.0;   // giây tính từ epoch
    int           width     = 0;
    int           height    = 0;
};

// ImuData: từ "@imu:x;y;yaw;;" của Nucleo (đơn vị theo firmware), ThreadUdpServer gửi lại "@imu:x;y;yaw;;"
struct ImuData {
    float x   = 0.0f;
    float y   = 0.0f;
    float yaw = 0.0f;
};

struct MessageInfo {
    std::string Queue;
    // Nhãn định tuyến: cặp (Owner, msgID) phải duy nhất trong bảng
    std::string Owner;
    int         msgID;
    std::string msgType;
};

// Lấy thông tin của message
const MessageInfo& get_message_info(MessageName message);

// Lấy tên message dưới dạng string, dùng để debug/log
std::string message_name_to_string(MessageName message);

// Tên kiểu của value: "bool", "int", "float", "str", "dict", còn lại "struct"
std::string detect_any_type(const std::any& value);

} // namespace utils::messages
