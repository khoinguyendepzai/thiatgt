#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include <utils/blockingQueue.hpp>
#include <utils/messages/messagePacket.hpp>
namespace utils {

// MessageQueue: MessagePacket từ sender ("Critical", "Warning", "General").
// ConfigQueue: ConfigPacket từ subscriber (subscribe/unsubscribe).

using MessageQueue    = BlockingQueue<utils::messages::MessagePacket>;
using MessageQueuePtr = std::shared_ptr<MessageQueue>;

using ConfigQueue    = BlockingQueue<utils::messages::ConfigPacket>;
using ConfigQueuePtr = std::shared_ptr<ConfigQueue>;

// Các queue ưu tiên theo tên
using MessageQueuesList = std::unordered_map<std::string, MessageQueuePtr>;

// Toàn bộ queue của hệ thống
struct QueuesList {
    MessageQueuesList messageQueues; // "Critical", "Warning", "General"
    ConfigQueuePtr    configQueue;   // "Config"

    // MessageQueue theo tên, ném std::out_of_range nếu không có
    MessageQueuePtr message(const std::string& name) const {
        return messageQueues.at(name);
    }

    ConfigQueuePtr config() const {
        return configQueue;
    }
};

// QueuesList mặc định với 4 queue
inline QueuesList create_default_queues_list() {
    QueuesList ql;
    ql.messageQueues["Critical"] = std::make_shared<MessageQueue>();
    ql.messageQueues["Warning"]  = std::make_shared<MessageQueue>();
    ql.messageQueues["General"]  = std::make_shared<MessageQueue>();
    ql.configQueue               = std::make_shared<ConfigQueue>();
    return ql;
}

} // namespace utils
