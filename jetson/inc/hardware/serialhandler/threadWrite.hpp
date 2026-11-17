#pragma once

#include <chrono>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

#include <hardware/serialhandler/messageConverter.hpp>
#include <hardware/serialhandler/serialPort.hpp>
#include <templates/threadWithStop.hpp>
#include <utils/messages/allMessages.hpp>
#include <utils/messages/messageHandlerSubscriber.hpp>
#include <utils/queues.hpp>
#include <utils/logConfig.hpp>

namespace hardware::serialhandler {

// Nhận Speed/Steer/Directory từ gateway, ghi lệnh "#action:...;;\r\n" xuống Nucleo.
// Phần gửi text giống hệt ThreadUdpServer, chỉ khác bước gửi.
class ThreadWrite : public templates::ThreadWithStop {
public:
    ThreadWrite(utils::QueuesList& queuesList,
                SerialPort&        port,
                std::ostream&      logger    = std::cerr,
                bool               debugging = false);
    ~ThreadWrite() override;

    // Gửi thẳng speed=0, steer=0, dir=0 xuống cổng. Gọi sau khi thread đã pause hoặc stop.
    void send_stop();
    // Đăng ký / hủy đăng ký cả 3 subscriber. Gọi set_subscribed(false) TRƯỚC pause(), set_subscribed(true)
    // TRƯỚC resume(). Chỉ gọi từ thread của ThreadSerialHandler.
    void set_subscribed(bool on);

protected:
    void thread_work() override;

private:
    bool send_command(const std::string& action, const std::vector<CommandArg>& args);
    void send_speed(float speed);
    void send_steer(float steer);
    void send_dir(int dir);
    // THÊM BUILDER MỚI Ở ĐÂY

    SerialPort&        port_;
    std::ostream&      logger_;
    bool               debugging_;
    MessageConverter   converter_;
    bool               subscribed_ = true;   // constructor đã đăng ký sẵn (subscribeNow = true)
    // send_stop() (thread của ThreadSerialHandler) và thread_work() không gửi chen nhau
    std::mutex         sendMutex_;

    utils::messages::MessageHandlerSubscriber speedSubscriber_;
    utils::messages::MessageHandlerSubscriber steerSubscriber_;
    utils::messages::MessageHandlerSubscriber directorySubscriber_;
    // THÊM SUBSCRIBER MỚI Ở ĐÂY (và khởi tạo trong constructor, thêm vào set_subscribed())

    utils::RateLimiter errorLimiter_{utils::logcfg::kErrorCooldown};
};

} // namespace hardware::serialhandler
