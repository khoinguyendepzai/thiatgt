#include <statemachine/stateMachine.hpp>

#include <exception>
#include <string>

namespace statemachine {

using utils::messages::MessageName;

StateMachine::StateMachine(utils::QueuesList& queuesList, std::ostream& logger)
    : logger_(logger)
    , stateChangeSender_(queuesList, MessageName::StateChange)
{}

bool StateMachine::request_mode(SystemMode newMode) {
    // Giữ mutex cả lúc gửi để thứ tự StateChange khớp thứ tự đổi mode
    // khi nhiều thread gọi cùng lúc.
    std::lock_guard<std::mutex> lock(mutex_);

    if (newMode == mode_) return false;

    logger_ << "[State Machine] Mode changed from " << mode_to_string(mode_)
            << " to " << mode_to_string(newMode) << '\n';

    mode_ = newMode;
    send_state_change(mode_);
    return true;
}

SystemMode StateMachine::get_mode() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return mode_;
}

void StateMachine::initialize_starting_mode() {
    std::lock_guard<std::mutex> lock(mutex_);
    send_state_change(mode_);
    logger_ << "[State Machine] Starting in " << mode_to_string(mode_) << " mode\n";
}

void StateMachine::send_state_change(SystemMode mode) {
    try {
        // Phải là std::string: ThreadSerialHandler dùng any_cast<std::string>
        stateChangeSender_.send(mode_to_string(mode));
    } catch (const std::exception& e) {
        logger_ << "[State Machine] WARNING: Failed to send state change: "
                << e.what() << '\n';
    }
}

} // namespace statemachine
