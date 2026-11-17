#pragma once

#include <iostream>
#include <mutex>

#include <statemachine/systemMode.hpp>
#include <utils/messages/messageHandlerSender.hpp>
#include <utils/queues.hpp>

namespace statemachine {

// Giữ mode hiện tại của hệ thống và phát StateChange khi mode đổi. Thread-safe.
class StateMachine {
public:
    explicit StateMachine(utils::QueuesList& queuesList,
                          std::ostream&      logger = std::cout);

    StateMachine(const StateMachine&)            = delete;
    StateMachine& operator=(const StateMachine&) = delete;

    // true  = mode đã đổi và đã gửi StateChange
    // false = đã ở mode này rồi (không gửi lại)
    bool request_mode(SystemMode newMode);

    SystemMode get_mode() const;

    // Gửi StateChange với mode hiện tại. Gọi SAU KHI mọi subscriber đã đăng ký.
    void initialize_starting_mode();

private:
    // Phải gọi khi đang giữ mutex_
    void send_state_change(SystemMode mode);

    std::ostream&                         logger_;
    utils::messages::MessageHandlerSender stateChangeSender_;

    mutable std::mutex mutex_;
    SystemMode         mode_ = SystemMode::Default;
};

} // namespace statemachine
