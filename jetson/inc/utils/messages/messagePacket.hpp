#pragma once

#include <any>
#include <memory>
#include <string>

// Forward declaration để tránh circular include với queues.hpp
namespace utils { template<typename T> class BlockingQueue; }
namespace utils::messages {

// Packet sender đưa vào MessageQueue ("Critical", "Warning", "General")
struct MessagePacket {
    std::string Owner;
    int         msgID   = 0;
    std::string msgType;
    std::any    msgValue;
};

// Packet subscriber gửi vào ConfigQueue khi đăng ký / hủy đăng ký (pipe = nullptr khi hủy)
struct ConfigPacket {
    std::string SubscribeUnsubscribe; // "subscribe" hoặc "unsubscribe"
    std::string Owner;
    int         msgID    = 0;
    std::string receiver;
    std::shared_ptr<utils::BlockingQueue<MessagePacket>> pipe;
};
} // namespace utils::messages
