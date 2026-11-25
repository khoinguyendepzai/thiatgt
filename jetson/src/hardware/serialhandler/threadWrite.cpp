#include <hardware/serialhandler/threadWrite.hpp>

#include <any>
#include <optional>

#include <utils/stringUtils.hpp>

namespace hardware::serialhandler {

using utils::messages::MessageHandlerSubscriber;
using utils::messages::MessageName;

namespace {

// Giá trị mới nhất kiểu T, nullopt nếu sai kiểu hoặc không có tin. Giống hệt receive_as() trong threadUdpServer.cpp.
template <typename T>
std::optional<T> receive_as(MessageHandlerSubscriber& sub) {
    const auto msg = sub.receive();
    if (!msg) return std::nullopt;

    if (const auto* p = std::any_cast<T>(&*msg)) return *p;
    return std::nullopt;
}

} // namespace

// ===================================== INIT =========================================
ThreadWrite::ThreadWrite(utils::QueuesList& queuesList, SerialPort& port,
                         std::ostream& logger, bool debugging)
    : templates::ThreadWithStop(0.001)
    , port_(port)
    , logger_(logger)
    , debugging_(debugging)
    , converter_(logger)
    , speedSubscriber_    (queuesList, MessageName::Speed,     "lastonly", true, "SerialWriter")
    , steerSubscriber_    (queuesList, MessageName::Steer,     "lastonly", true, "SerialWriter")
    , directorySubscriber_(queuesList, MessageName::Directory, "lastonly", true, "SerialWriter")
    // THÊM SUBSCRIBER MỚI Ở ĐÂY
{}

ThreadWrite::~ThreadWrite() {
    stop();
    join();
}

// ==================================== SENDING =======================================
// Đổi (action, args) thành lệnh text rồi gửi. Giống hệt ThreadUdpServer::send_command() trừ bước gửi.
bool ThreadWrite::send_command(const std::string& action, const std::vector<CommandArg>& args) {
    const auto cmd = converter_.get_command(action, args);   // sai kiểu/thiếu tham số: converter đã log
    if (!cmd) return false;

    // Gửi xuống Nucleo qua dây serial
    if (!port_.write_all(*cmd)) {
        if (errorLimiter_.allow()) {
            logger_ << "[ThreadWrite] ERROR: Failed to write to serial\n";
        }
        return false;
    }

    if (debugging_) {
        if (!debugLine_.empty()) debugLine_ += "  ";
        debugLine_ += utils::strip(*cmd);
    }
    return true;
}

// In các lệnh đã gom trong vòng này. force: in ngay, không qua debugLimiter_ (lệnh dừng).
void ThreadWrite::flush_debug(const char* tag, bool force) {
    if (debugLine_.empty()) return;
    ++debugLoops_;
    if (force || debugLimiter_.allow()) {
        logger_ << "[ThreadWrite][debug] " << tag << " -> serial: " << debugLine_ << "  (" << debugLoops_
                << " vong ghi tu lan log truoc)\n";
        debugLoops_ = 0;
    }
    debugLine_.clear();
}

// Tên action và tên tham số phải khớp đúng bảng commands() trong messageConverter.cpp
void ThreadWrite::send_speed(float speed) { send_command("speed", {{"speed", speed}}); }
void ThreadWrite::send_steer(float steer) { send_command("steer", {{"steer", steer}}); }
void ThreadWrite::send_dir(int dir)       { send_command("dir",   {{"dir",   dir}});   }

// sendMutex_ bảo đảm lệnh dừng là lệnh cuối cùng Nucleo nhận (thread_work không gửi sau khi pause)
void ThreadWrite::send_stop() {
    std::lock_guard<std::mutex> lock(sendMutex_);
    send_speed(0.0f);
    send_steer(0.0f);
    send_dir(0);
    flush_debug("STOP", true);
}

void ThreadWrite::set_subscribed(bool on) {
    if (on == subscribed_) return;
    subscribed_ = on;

    // THÊM SUBSCRIBER MỚI vào danh sách này
    for (auto* sub : {&speedSubscriber_, &steerSubscriber_, &directorySubscriber_}) {
        if (on) {
            sub->empty();        // bỏ tin còn sót từ trước lúc hủy đăng ký
            sub->subscribe();
        } else {
            sub->unsubscribe();
        }
    }
}

// ===================================== RUN ==========================================
void ThreadWrite::thread_work() {
    std::lock_guard<std::mutex> lock(sendMutex_);
    // Đã bị pause trong lúc vòng này chạy: không gửi, để không ghi đè lệnh dừng của send_stop()
    if (is_paused()) return;

    // lastonly: mỗi vòng chỉ lấy giá trị mới nhất
    if (const auto v = receive_as<float>(speedSubscriber_))   send_speed(*v);
    if (const auto v = receive_as<float>(steerSubscriber_))   send_steer(*v);
    if (const auto v = receive_as<int>(directorySubscriber_)) send_dir(*v);
    flush_debug("gateway", false);

    // THÊM KHỐI MỚI Ở ĐÂY
}

} // namespace hardware::serialhandler
