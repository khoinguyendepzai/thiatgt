#pragma once

#include <any>

#include <utils/queues.hpp>
#include <utils/messages/allMessages.hpp>
#include <utils/messages/messagePacket.hpp>

namespace utils::messages {

// Gửi một loại message vào đúng queue. MessageInfo và queue đích tra một lần lúc tạo.
class MessageHandlerSender {
public:
    // Ném std::invalid_argument nếu message không có trong get_message_info().
    // Queue đích không tồn tại thì send() ném std::runtime_error.
    MessageHandlerSender(utils::QueuesList& queuesList, MessageName message);

    // Nhận theo giá trị để rvalue được move
    void send(std::any value);

private:
    const MessageInfo&     info_;    // trỏ tới dữ liệu static của get_message_info()
    utils::MessageQueuePtr queue_;   // nullptr nếu info_.Queue không tồn tại
};

} // namespace utils::messages
