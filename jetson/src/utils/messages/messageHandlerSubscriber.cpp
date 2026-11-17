#include <utils/messages/messageHandlerSubscriber.hpp>

#include <iostream>
#include <utility>

#include <utils/stringUtils.hpp>

namespace utils::messages {

MessageHandlerSubscriber::MessageHandlerSubscriber(
    utils::QueuesList& queuesList,
    MessageName        message,
    std::string        deliveryMode,
    bool               subscribeNow,
    std::string        receiver
)
    : queuesList_(queuesList)
    , message_(message)
    , deliveryMode_(utils::to_lower(std::move(deliveryMode)))
    , receiver_(std::move(receiver))
    , pipe_(std::make_shared<utils::BlockingQueue<MessagePacket>>())
{
    if (receiver_.empty()) {
        receiver_ = "UnknownReceiver";
    }
    if (subscribeNow) {
        subscribe();
    }
    if (deliveryMode_ != "fifo" && deliveryMode_ != "lastonly") {
        std::cout
            << "WARNING! Wrong delivery mode supplied. "
            << deliveryMode_
            << " instead of FIFO or LastOnly. "
            << message_name_to_string(message_)
            << " "
            << receiver_
            << '\n'
            << "WARNING! Switching to FIFO\n";

        deliveryMode_ = "fifo";
    }
}

MessageHandlerSubscriber::~MessageHandlerSubscriber() {
    try {
        unsubscribe();
    } catch (...) {
        // destructor không được ném exception
    }
}

std::optional<std::any> MessageHandlerSubscriber::receive() {
    if (!pipe_->poll()) {
        return std::nullopt;
    }
    return receive_with_block();
}

std::any MessageHandlerSubscriber::receive_with_block() {
    MessagePacket message = pipe_->get();

    if (deliveryMode_ == "lastonly") {
        // try_get() chỉ ghi vào message khi lấy được, nên cuối vòng message là bản mới nhất
        while (pipe_->try_get(message)) {}
    }

    check_message_type(message.msgValue);
    return std::move(message.msgValue);
}

void MessageHandlerSubscriber::empty() {
    pipe_->clear();
}

void MessageHandlerSubscriber::subscribe() {
    if (subscribed_) return;

    const MessageInfo& info = get_message_info(message_);

    ConfigPacket packet;
    packet.SubscribeUnsubscribe = "subscribe";
    packet.Owner                = info.Owner;
    packet.msgID                = info.msgID;
    packet.receiver             = receiver_;
    packet.pipe                 = pipe_;

    queuesList_.configQueue->put(packet);
    subscribed_ = true;
}

void MessageHandlerSubscriber::unsubscribe() {
    if (!subscribed_) return;

    const MessageInfo& info = get_message_info(message_);

    ConfigPacket packet;
    packet.SubscribeUnsubscribe = "unsubscribe";
    packet.Owner                = info.Owner;
    packet.msgID                = info.msgID;
    packet.receiver             = receiver_;
    packet.pipe                 = nullptr;  // unsubscribe không cần pipe

    queuesList_.configQueue->put(packet);
    subscribed_ = false;
}

bool MessageHandlerSubscriber::is_data_in_pipe() const {
    return pipe_->poll();
}

void MessageHandlerSubscriber::set_delivery_mode_to_fifo() {
    deliveryMode_ = "fifo";
}

void MessageHandlerSubscriber::set_delivery_mode_to_last_only() {
    deliveryMode_ = "lastonly";
}

std::shared_ptr<utils::BlockingQueue<MessagePacket>> MessageHandlerSubscriber::pipe() const {
    return pipe_;
}

void MessageHandlerSubscriber::check_message_type(const std::any& value) const {
    const MessageInfo& info        = get_message_info(message_);
    const std::string  messageType = detect_any_type(value);

    if (messageType != info.msgType) {
        std::cout
            << "WARNING! Message type and value type are not matching. "
            << message_name_to_string(message_)
            << " received: "  << messageType
            << " expected: "  << info.msgType
            << '\n';
    }
}

} // namespace utils::messages
