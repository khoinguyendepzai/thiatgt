#include <control/threadControlHandler.hpp>

#include <any>
#include <string>

#include <statemachine/systemMode.hpp>

namespace control {

using utils::messages::MessageName;

// ===================================== INIT =========================================
ThreadControlHandler::ThreadControlHandler(utils::QueuesList& queuesList,
                                           std::ostream&      logger,
                                           bool               debugging)
    : templates::ThreadWithStop(0.05)
    , logger_(logger)
    , debugging_(debugging)
    // receiver phải khác các handler khác: gateway khóa theo (Owner, msgID, receiver)
    , stateChangeSub_(queuesList, MessageName::StateChange, "lastonly", true, "ControlHandler")
{}

ThreadControlHandler::~ThreadControlHandler() {
    stop();
    join();
}

// ===================================== STATE CHANGE ==========================================
void ThreadControlHandler::state_change_handler() {
    const auto msg = stateChangeSub_.receive();
    if (!msg) return;

    if (const auto* mode = std::any_cast<std::string>(&*msg)) {
        // Tên mode không hợp lệ -> tắt
        const auto* cfg     = statemachine::find_mode_config(*mode);
        const bool  enabled = cfg && cfg->control.enabled;
        if (enabled == modeEnabled_) return;

        logger_ << "[ControlHandler] Control " << (enabled ? "enabled" : "disabled") << " (mode " << *mode << ")\n";
        modeEnabled_ = enabled;
    }
}

// ===================================== RUN ==========================================
void ThreadControlHandler::thread_work() {
    // Để trống: phần điều khiển code sau (đọc modeEnabled_ để biết mode hiện tại có bật control không)
}

} // namespace control
