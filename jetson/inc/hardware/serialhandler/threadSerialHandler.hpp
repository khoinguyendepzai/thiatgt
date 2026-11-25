#pragma once

#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include <hardware/serialhandler/serialPort.hpp>
#include <hardware/serialhandler/threadRead.hpp>
#include <hardware/serialhandler/threadWrite.hpp>
#include <templates/threadWithStop.hpp>
#include <utils/messages/allMessages.hpp>
#include <utils/messages/messageHandlerSubscriber.hpp>
#include <utils/queues.hpp>

namespace hardware::serialhandler {

// Sở hữu SerialPort (Nucleo), ThreadRead và ThreadWrite; bật/tắt chúng theo StateChange và tự kết nối lại.
// Mode tắt ghi khi cổng đang mở: pause ThreadWrite rồi gửi lệnh dừng.
class ThreadSerialHandler : public templates::ThreadWithStop {
public:
    // serialParams: cổng và baudrate của Nucleo. Constructor thử kết nối ngay.
    ThreadSerialHandler(utils::QueuesList& queuesList,
                        const SerialParams& serialParams,
                        std::ostream&      logger    = std::cerr,
                        bool               debugging = false);
    ~ThreadSerialHandler() override;

    // Kiểm tra cổng serial có đang mở hay không (ThreadSerialHandler đã kết nối thành công)
    bool is_connected() const { return connected_.load(); }
    // ThreadWrite đang chạy (cổng mở + mode cho ghi). false thì lệnh từ gateway KHÔNG xuống Nucleo.
    bool is_write_enabled() const { return !writeTh_->is_paused(); }
    // Log debug của ThreadWrite (lệnh ghi xuống serial), bật/tắt lúc đang chạy. Không đụng tới log của ThreadRead.
    void set_write_debugging(bool on) { writeTh_->set_debugging(on); }

protected:
    void thread_work() override;
    void state_change_handler() override;   // nhận StateChange, bật/tắt thread con

private:
    void handle_disconnection();
    void try_connect();
    void apply_thread_state();   // pause/resume ThreadRead + ThreadWrite, gửi lệnh dừng khi mode tắt ghi

    // port_ PHẢI khai báo trước readTh_/writeTh_: được hủy sau hai thread dùng nó
    SerialPort         port_;
    std::ostream&      logger_;

    utils::messages::MessageHandlerSubscriber stateChangeSub_;

    std::unique_ptr<ThreadRead>  readTh_;
    std::unique_ptr<ThreadWrite> writeTh_;

    std::atomic<bool> connected_{false}; // true nếu cổng đang mở
    bool              modeEnabledWrite_ = true; // true nếu mode hiện tại cho phép ThreadWrite chạy
    bool              modeEnabledRead_  = true; // true nếu mode hiện tại cho phép ThreadRead chạy
    // Thời điểm tiếp theo được phép thử kết nối lại
    std::chrono::steady_clock::time_point nextAttempt_{};
    // Thời gian chờ giữa các lần thử kết nối lại khi mất kết nối
    static constexpr std::chrono::seconds kReconnectInterval{1};
};

} // namespace hardware::serialhandler
