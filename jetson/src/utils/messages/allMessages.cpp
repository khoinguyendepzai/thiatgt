#include <utils/messages/allMessages.hpp>

#include <cstdint>
#include <map>
#include <stdexcept>
#include <typeinfo>
#include <unordered_map>

namespace utils::messages {

const MessageInfo& get_message_info(MessageName message) {
    ////////// Camera ///////////
    static const MessageInfo CameraFrameInfo{
        "General", "threadCameraHandler", 2, "struct"
    };
    static const MessageInfo LaneCameraFrameInfo{
        "General", "threadCameraHandler", 1, "struct"
    };

    ////////// To control ///////////
    // Owner ghi nơi xử lý
    static const MessageInfo DistanceInfo{
        "Warning", "threadControlHandler", 1, "float"
    };

    ////////// To Nucleo ///////////
    // Owner ghi nơi xử lý
    static const MessageInfo SpeedInfo{
        "General", "threadWrite", 1, "float"
    };
    static const MessageInfo SteerInfo{
        "General", "threadWrite", 2, "float"
    };
    static const MessageInfo DirectoryInfo{
        "General", "threadWrite", 3, "int"
    };

    ////////// From Nucleo ///////////
    static const MessageInfo ImuDataInfo{
        "General", "threadRead", 1, "struct"
    };
    static const MessageInfo CurrentSpeedInfo{
        "General", "threadRead", 2, "float"
    };

    ////////// From StateMachine ///////////
    static const MessageInfo StateChangeInfo{
        "Critical", "stateMachine", 1, "str"
    };

    switch (message) {
        // Camera
        case MessageName::CameraFrame:
            return CameraFrameInfo;
        case MessageName::LaneCameraFrame:
            return LaneCameraFrameInfo;

        // To control
        case MessageName::Distance:
            return DistanceInfo;

        // To Nucleo
        case MessageName::Speed:
            return SpeedInfo;
        case MessageName::Steer:
            return SteerInfo;
        case MessageName::Directory:
            return DirectoryInfo;

        // From Nucleo
        case MessageName::ImuData:
            return ImuDataInfo;
        case MessageName::CurrentSpeed:
            return CurrentSpeedInfo;

        // From StateMachine
        case MessageName::StateChange:
            return StateChangeInfo;
    }

    throw std::invalid_argument("Unknown MessageName");
}

std::string message_name_to_string(MessageName message) {
    switch (message) {
        // Camera
        case MessageName::CameraFrame:
            return "CameraFrame";
        case MessageName::LaneCameraFrame:
            return "LaneCameraFrame";

        // To control
        case MessageName::Distance:
            return "Distance";

        // To Nucleo
        case MessageName::Speed:
            return "Speed";
        case MessageName::Steer:
            return "Steer";
        case MessageName::Directory:
            return "Directory";

        // From Nucleo
        case MessageName::ImuData:
            return "ImuData";
        case MessageName::CurrentSpeed:
            return "CurrentSpeed";

        // From StateMachine
        case MessageName::StateChange:
            return "StateChange";
    }

    throw std::invalid_argument("Unknown MessageName");
}

std::string detect_any_type(const std::any& value) {
    // Kiểm tra bool trước int vì sizeof(bool) == sizeof(int) trên vài hệ
    if (value.type() == typeid(bool))
        return "bool";

    // Integer variants
    if (value.type() == typeid(int))
        return "int";
    if (value.type() == typeid(std::int32_t))
        return "int";
    if (value.type() == typeid(std::int64_t))
        return "int";
    if (value.type() == typeid(unsigned int))
        return "int";
    if (value.type() == typeid(unsigned long))
        return "int";
    if (value.type() == typeid(unsigned long long))
        return "int";

    // Floating point
    if (value.type() == typeid(float))
        return "float";
    if (value.type() == typeid(double))
        return "float";

    // String
    if (value.type() == typeid(std::string))
        return "str";
    if (value.type() == typeid(const char*))
        return "str";

    // Dictionary (map variants)
    if (value.type() == typeid(std::map<std::string, std::any>))
        return "dict";
    if (value.type() == typeid(std::unordered_map<std::string, std::any>))
        return "dict";

    // Fallback: mọi kiểu khác là struct/class tự định nghĩa (ImuData, FrameInfo...)
    return "struct";
}

} // namespace utils::messages
