#include <gateway/threadGateWay.hpp>

#include <utils/stringUtils.hpp>

namespace gateway {

ThreadGateway::ThreadGateway(utils::QueuesList& queuesList,
                             std::ostream& logger,
                             bool debugging)
    : templates::ThreadWithStop(0.001)
    , queuesList_(queuesList)
    , logger_(logger)
    , debugging_(debugging)
{}

// Dừng thread trước khi sendingList_ bị hủy
ThreadGateway::~ThreadGateway() {
    stop();
    join();
}

void ThreadGateway::subscribe(const ConfigPacket& message) {
    if (!message.pipe) {
        logger_ << "Subscribe error: pipe is null\n";
        return;
    }
    sendingList_[message.Owner][message.msgID][message.receiver] = message.pipe;

    if (debugging_) {
        print_list();
    }
}

void ThreadGateway::unsubscribe(const ConfigPacket& message) {
    auto ownerIt = sendingList_.find(message.Owner);
    if (ownerIt == sendingList_.end()) return;

    auto idIt = ownerIt->second.find(message.msgID);
    if (idIt == ownerIt->second.end()) return;

    idIt->second.erase(message.receiver);

    // Dọn map rỗng: nhờ vậy (Owner, msgID) còn trong sendingList_ nghĩa là còn người nhận
    if (idIt->second.empty())    ownerIt->second.erase(idIt);
    if (ownerIt->second.empty()) sendingList_.erase(ownerIt);

    if (debugging_) {
        print_list();
    }
}

void ThreadGateway::send(const MessagePacket& message) {
    auto ownerIt = sendingList_.find(message.Owner);
    if (ownerIt == sendingList_.end()) return;

    auto idIt = ownerIt->second.find(message.msgID);
    if (idIt == ownerIt->second.end()) return;

    for (auto& entry : idIt->second) {
        if (entry.second) {
            entry.second->put(message); // put vào pipe của receiver
        }
    }

    if (debugging_) {
        logger_ << "Gateway forwarded message: Owner=" << message.Owner
                << ", msgID=" << message.msgID
                << ", msgType=" << message.msgType << '\n';
    }
}

void ThreadGateway::print_list() const {
    logger_ << "Gateway sendingList:\n";

    for (const auto& [owner, ids] : sendingList_) {
        logger_ << "  Owner: " << owner << '\n';

        for (const auto& [id, receivers] : ids) {
            logger_ << "    msgID: " << id << " -> ";

            for (const auto& entry : receivers) {
                logger_ << entry.first << ' ';
            }
            logger_ << '\n';
        }
    }
}

// Xử lý hết Config (subscribe/unsubscribe), rồi chuyển hết data message theo ưu tiên
void ThreadGateway::thread_work() {
    if (queuesList_.configQueue) {
        ConfigPacket configMessage;
        while (queuesList_.configQueue->try_get(configMessage)) {
            const std::string action = utils::to_lower(configMessage.SubscribeUnsubscribe);

            if (action == "subscribe") {
                subscribe(configMessage);
            } else {
                unsubscribe(configMessage);
            }
        }
    }

    // Mỗi lần lấy một message, ưu tiên Critical > Warning > General, rồi kiểm tra lại từ Critical
    auto try_take = [this](const char* name, MessagePacket& out) {
        auto it = queuesList_.messageQueues.find(name);
        return it != queuesList_.messageQueues.end()
            && it->second
            && it->second->try_get(out);
    };

    MessagePacket message;
    while (try_take("Critical", message) ||
           try_take("Warning",  message) ||
           try_take("General",  message)) {
        send(message);
    }
}

} // namespace gateway
