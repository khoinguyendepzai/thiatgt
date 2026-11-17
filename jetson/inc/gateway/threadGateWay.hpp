#pragma once

#include <templates/threadWithStop.hpp>
#include <utils/queues.hpp>
#include <utils/messages/messagePacket.hpp>

#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>

namespace gateway {
class ThreadGateway : public templates::ThreadWithStop {
public:
    explicit ThreadGateway(utils::QueuesList& queuesList,
                           std::ostream& logger = std::cout,
                           bool debugging = false);
    ~ThreadGateway() override;

protected:
    void thread_work() override;

private:
    using MessagePacket = utils::messages::MessagePacket;
    using ConfigPacket  = utils::messages::ConfigPacket;
    using PipePtr       = std::shared_ptr<utils::BlockingQueue<MessagePacket>>;

    void subscribe(const ConfigPacket& message);
    void unsubscribe(const ConfigPacket& message);
    void send(const MessagePacket& message);
    void print_list() const;

    utils::QueuesList& queuesList_;
    std::ostream&      logger_;
    bool               debugging_;
    // sendingList_[Owner][msgID][receiver] = pipe của receiver
    std::unordered_map<std::string, std::unordered_map<int, std::unordered_map<std::string, PipePtr>>> sendingList_;
};

} // namespace gateway
