#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <string>

#include <network/tcpClient.hpp>
#include <statemachine/stateMachine.hpp>
#include <templates/threadWithStop.hpp>
#include <utils/messages/allMessages.hpp>
#include <utils/messages/messageHandlerSender.hpp>
#include <utils/queues.hpp>
#include <utils/logConfig.hpp>

namespace network {

// Kết nối tới server TCP, nhận "#action:giá trị;;\r\n" và xử lý (main_master.py gửi '#'; '@' vẫn được nhận):
//   #vcb:speed;steer;dir -> Speed, Steer (float), Directory (int); thiếu field hoặc sai kiểu thì bỏ cả gói
//   #state:TÊN / #mode:TÊN -> sm.request_mode() (không phân biệt hoa thường; tên sai thì bỏ qua)
//   #distance:d -> Distance (float, m; < 0 = không có vật cản)
// process_data/send_queue giống hệt ThreadRead. Mất kết nối thì tự nối lại mỗi kReconnectInterval.
class ThreadTcpClient : public templates::ThreadWithStop {
public:
    // sm: nhận yêu cầu đổi state từ server (phải sống lâu hơn thread này).
    ThreadTcpClient(utils::QueuesList& queuesList,
                    statemachine::StateMachine& sm,
                    std::string host,
                    std::uint16_t port,
                    std::ostream& logger = std::cerr,
                    bool debugging = false);
    ~ThreadTcpClient() override;

    bool is_connected() const { return connected_.load(); }

    // Đóng kết nối ngay. Handler gọi SAU pause(). Thread-safe (chờ thread_work() đang chạy xong).
    void disconnect_now();

protected:
    void thread_work() override;

private:
    using Clock = std::chrono::steady_clock;

    void try_connect();
    void disconnect(const char* reason);
    void read_available();
    void process_data(const std::string& data);
    void send_queue(const std::string& buff);
    void send_stop_command(const char* reason);   // tốc độ 0, lái thẳng, dir 0 (như pi_bridge khi mất lệnh)

    std::string host_;
    std::uint16_t port_;
    std::ostream& logger_;
    bool debugging_;

    utils::messages::MessageHandlerSender steerSender_;
    utils::messages::MessageHandlerSender speedSender_;
    utils::messages::MessageHandlerSender directorySender_;
    utils::messages::MessageHandlerSender distanceSender_;
    statemachine::StateMachine& sm_;

    std::mutex ioMutex_;   // bảo vệ client_, buffer_, nextAttempt_... (TcpClient không thread-safe)
    TcpClient client_;
    std::string buffer_;

    std::atomic<bool> connected_{false};
    Clock::time_point nextAttempt_{};

    // An toàn: NUCLEO giữ lệnh cuối cùng nhận được. Mất kết nối hoặc PC ngừng gửi #vcb quá kCommandTimeout thì gửi lệnh
    // dừng một lần (main_master gửi 50 lần/s, nên 300 ms = ~15 lệnh liên tiếp bị mất).
    Clock::time_point lastVcb_{};
    bool              driving_ = false;   // đã nhận #vcb và chưa gửi lệnh dừng

    utils::RateLimiter errorLimiter_{utils::logcfg::kErrorCooldown};

    static constexpr std::chrono::milliseconds kCommandTimeout{300};
    static constexpr std::chrono::seconds kReconnectInterval{1};
    static constexpr std::chrono::milliseconds kConnectTimeout{1000};
    static constexpr std::size_t kMaxBufferSize = 4096;
    static constexpr int kMaxReadsPerLoop = 16;
};

} // namespace network
