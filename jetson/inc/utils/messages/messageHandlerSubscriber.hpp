#pragma once

#include <any>
#include <memory>
#include <optional>
#include <string>

#include <utils/queues.hpp>
#include <utils/messages/allMessages.hpp>
#include <utils/messages/messagePacket.hpp>

namespace utils::messages {

// Đăng ký nhận một loại message qua gateway, nhận vào pipe riêng (BlockingQueue).
// Gateway khóa theo (Owner, msgID, receiver): hai subscriber cùng message phải khác receiver.
class MessageHandlerSubscriber {
public:
    // deliveryMode: "fifo" hoặc "lastonly" (chỉ giữ giá trị mới nhất)
    MessageHandlerSubscriber(
        utils::QueuesList& queuesList,
        MessageName        message,
        std::string        deliveryMode = "fifo",
        bool               subscribeNow = false,
        std::string        receiver     = ""
    );

    ~MessageHandlerSubscriber();
    MessageHandlerSubscriber(const MessageHandlerSubscriber&)            = delete;
    MessageHandlerSubscriber& operator=(const MessageHandlerSubscriber&) = delete;

    // Không chặn: nullopt nếu pipe rỗng
    std::optional<std::any> receive();

    // Chặn tới khi có message
    std::any receive_with_block();

    // Bỏ hết message trong pipe
    void empty();

    void subscribe();
    void unsubscribe();

    bool is_data_in_pipe() const;

    void set_delivery_mode_to_fifo();
    void set_delivery_mode_to_last_only();

    // Pipe nội bộ, gateway ghi MessagePacket vào đây
    std::shared_ptr<utils::BlockingQueue<MessagePacket>> pipe() const;

private:
    // Cảnh báo nếu kiểu thực tế của value không khớp msgType đã khai báo
    void check_message_type(const std::any& value) const;

    utils::QueuesList& queuesList_;
    MessageName        message_;
    std::string        deliveryMode_;
    std::string        receiver_;

    std::shared_ptr<utils::BlockingQueue<MessagePacket>> pipe_;
    bool subscribed_ = false;
};

} // namespace utils::messages
