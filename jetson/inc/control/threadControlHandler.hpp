#pragma once

#include <atomic>
#include <iostream>

#include <templates/threadWithStop.hpp>
#include <utils/messages/allMessages.hpp>
#include <utils/messages/messageHandlerSubscriber.hpp>
#include <utils/queues.hpp>

namespace control {

// Khung cho phần điều khiển. Không còn bám làn / đổi làn / dừng vì vật cản: hiện chỉ subscribe StateChange và ghi nhận
// mode hiện tại có bật control (ModeConfig::control.enabled) hay không. thread_work() để trống, code sau.
class ThreadControlHandler : public templates::ThreadWithStop {
public:
    ThreadControlHandler(utils::QueuesList& queuesList,
                         std::ostream&      logger    = std::cerr,
                         bool               debugging = false);
    ~ThreadControlHandler() override;

    // Mode hiện tại có bật control hay không
    bool is_enabled() const { return modeEnabled_.load(); }

protected:
    void state_change_handler() override;   // nhận StateChange, cập nhật modeEnabled_
    void thread_work() override;            // để trống, code sau

private:
    std::ostream& logger_;
    bool          debugging_;

    utils::messages::MessageHandlerSubscriber stateChangeSub_;

    // true nếu mode hiện tại bật control. Chỉ thread của handler ghi.
    std::atomic<bool> modeEnabled_{false};
};

} // namespace control
