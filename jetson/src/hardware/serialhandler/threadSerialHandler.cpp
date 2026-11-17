#include <hardware/serialhandler/threadSerialHandler.hpp>

#include <any>
#include <utility>

#include <statemachine/systemMode.hpp>

namespace hardware::serialhandler {

using utils::messages::MessageName;
using Clock = std::chrono::steady_clock;

// ===================================== INIT =========================================
ThreadSerialHandler::ThreadSerialHandler(utils::QueuesList& queuesList,
                                         const SerialParams& serialParams,
                                         std::ostream&      logger,
                                         bool               debugging)
    : templates::ThreadWithStop(0.05)
    , port_(serialParams)
    , logger_(logger)
    // receiver phải khác các handler khác: gateway khóa theo (Owner, msgID, receiver)
    , stateChangeSub_(queuesList, MessageName::StateChange, "lastonly", true, "SerialHandler")
    , readTh_(std::make_unique<ThreadRead>(queuesList, port_, logger, debugging))
    , writeTh_(std::make_unique<ThreadWrite>(queuesList, port_, logger, debugging))
{
    // start() luôn đặt lại trạng thái Running, nên phải start trước rồi mới pause
    readTh_->start();
    writeTh_->start();

    // Kết nối ngay với port đã truyền vào; thất bại thì thread_work() sẽ thử lại mỗi kReconnectInterval
    try_connect();
    apply_thread_state();      // chưa kết nối được -> pause cả hai
}

// Thứ tự: dừng vòng quản lý trước (không còn ai pause/resume), rồi dừng write,
// gửi lệnh dừng xe, dừng read, cuối cùng đóng cổng.
ThreadSerialHandler::~ThreadSerialHandler() {
    stop();
    join();

    writeTh_->stop();          // dừng write trước để không ghi đè lệnh dừng
    writeTh_->join();
    if (port_.is_open()) writeTh_->send_stop();

    // readTh_ tự stop + join trong destructor của nó
    port_.close();
}

// ===================================== RUN ==========================================
void ThreadSerialHandler::thread_work() {
    if (connected_ && port_.connection_lost()) {
        handle_disconnection();
    }
    if (!connected_ && Clock::now() >= nextAttempt_) {
        try_connect();
    }
}

// ===================================== STATE CHANGE ==========================================
void ThreadSerialHandler::state_change_handler() {
    const auto msg = stateChangeSub_.receive();
    if (!msg) return;

    if (const auto* mode = std::any_cast<std::string>(&*msg)) {
        // Tên mode không hợp lệ (cfg == nullptr) -> tắt serial cho an toàn
        const auto* cfg = statemachine::find_mode_config(*mode);
        modeEnabledWrite_ = cfg && cfg->serialHandler.enabledWrite;
        modeEnabledRead_  = cfg && cfg->serialHandler.enabledRead;
        apply_thread_state();
    }
}

// dừng thread con và đóng cổng khi mất kết nối
void ThreadSerialHandler::handle_disconnection() {
    logger_ << "[SerialHandler] Serial device disconnected\n";
    connected_ = false;
    apply_thread_state();      // pause trước khi đóng
    port_.close();
    nextAttempt_ = Clock::now() + kReconnectInterval;
}

// Cố gắng mở lại đúng port đã truyền vào constructor của SerialPort.
// nextAttempt_ được đặt trước, nên mọi nhánh lỗi đều chờ đủ kReconnectInterval.
void ThreadSerialHandler::try_connect() {
    nextAttempt_ = Clock::now() + kReconnectInterval;

    port_.close();
    if (!port_.open()) return;

    connected_ = true;
    logger_ << "[SerialHandler] Connected to " << port_.device() << '\n';
    apply_thread_state();
}

// Chưa kết nối hoặc mode tắt -> pause. ThreadWrite: hủy đăng ký TRƯỚC khi pause,
// đăng ký lại TRƯỚC khi resume, để gateway không đẩy tin vào pipe của thread đang ngủ.
void ThreadSerialHandler::apply_thread_state() {
    const bool writeOn = connected_ && modeEnabledWrite_;
    const bool readOn  = connected_ && modeEnabledRead_;

    if (writeOn) {
        writeTh_->set_subscribed(true);
        writeTh_->resume();
    } else {
        writeTh_->set_subscribed(false);
        writeTh_->pause();
        // Mode tắt ghi mà cổng vẫn mở: gửi lệnh dừng, nếu không xe giữ lệnh cuối cùng
        if (connected_ && !modeEnabledWrite_) writeTh_->send_stop();
    }

    if (readOn) readTh_->resume();
    else        readTh_->pause();
}

} // namespace hardware::serialhandler
