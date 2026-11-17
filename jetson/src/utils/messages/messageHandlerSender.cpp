#include <utils/messages/messageHandlerSender.hpp>

#include <stdexcept>
#include <utility>

namespace utils::messages {

MessageHandlerSender::MessageHandlerSender(
    utils::QueuesList& queuesList,
    MessageName message
)
    : info_(get_message_info(message))
{
    const auto it = queuesList.messageQueues.find(info_.Queue);
    if (it != queuesList.messageQueues.end()) {
        queue_ = it->second;
    }
}

void MessageHandlerSender::send(std::any value) {
    if (!queue_) {
        throw std::runtime_error(
            "MessageHandlerSender: queue not found: " + info_.Queue
        );
    }

    MessagePacket packet;
    packet.Owner    = info_.Owner;
    packet.msgID    = info_.msgID;
    packet.msgType  = info_.msgType;
    packet.msgValue = std::move(value);

    queue_->put(std::move(packet));
}

} // namespace utils::messages
