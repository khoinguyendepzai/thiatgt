#pragma once

#include <chrono>
#include <cstddef>
#include <iostream>
#include <string>

#include <hardware/serialhandler/serialPort.hpp>
#include <templates/threadWithStop.hpp>
#include <utils/messages/allMessages.hpp>
#include <utils/messages/messageHandlerSender.hpp>
#include <utils/queues.hpp>
#include <utils/logConfig.hpp>

namespace hardware::serialhandler {

// Đọc "@action:giá trị;;\r\n" từ Nucleo và gửi lên gateway:
//   @imu:x;y;yaw                             -> ImuData
//   @hall:speed;...                          -> CurrentSpeed (float)
// Thiếu field hoặc sai kiểu thì bỏ cả message. process_data/send_queue giống hệt ThreadTcpClient.
class ThreadRead : public templates::ThreadWithStop {
public:
    ThreadRead(utils::QueuesList& queuesList,
               SerialPort&        port,
               std::ostream&      logger    = std::cerr,
               bool               debugging = false);
    ~ThreadRead() override;

protected:
    void thread_work() override;

private:
    void process_data(const std::string& data);
    void send_queue(const std::string& buff);

    SerialPort&        port_;
    std::ostream&      logger_;
    bool               debugging_;
    std::string        buffer_;

    utils::messages::MessageHandlerSender currentSpeedSender_;
    utils::messages::MessageHandlerSender imuDataSender_;
    // THÊM SENDER MỚI Ở ĐÂY (và khởi tạo trong constructor)

    utils::RateLimiter errorLimiter_{utils::logcfg::kErrorCooldown};

    static constexpr std::size_t kMaxBufferSize = 4096;
};

} // namespace hardware::serialhandler
