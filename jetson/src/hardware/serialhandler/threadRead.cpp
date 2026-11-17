#include <hardware/serialhandler/threadRead.hpp>

#include <cctype>
#include <exception>
#include <vector>

#include <utils/stringUtils.hpp>

namespace hardware::serialhandler {

using utils::messages::ImuData;
using utils::messages::MessageName;

// ===================================== INIT =========================================
ThreadRead::ThreadRead(utils::QueuesList& queuesList, SerialPort& port,
                       std::ostream& logger, bool debugging)
    : templates::ThreadWithStop(0.01)
    , port_(port)
    , logger_(logger)
    , debugging_(debugging)
    , currentSpeedSender_(queuesList, MessageName::CurrentSpeed)
    , imuDataSender_(queuesList, MessageName::ImuData)
    // THÊM SENDER MỚI Ở ĐÂY
{}

ThreadRead::~ThreadRead() {
    stop();
    join();
}

// ====================================== RUN ==========================================
void ThreadRead::thread_work() {
    try {
        std::string data;
        switch (port_.read_some(data)) {
            case ReadStatus::Closed:
                return;

            case ReadStatus::NoData:
                return;

            case ReadStatus::Error:
                if (errorLimiter_.allow()) {
                    logger_ << "[ThreadRead] ERROR: Reading from serial\n";
                }
                buffer_.clear();
                return;

            case ReadStatus::Data:
                process_data(data);
                return;
        }
    } catch (const std::exception& e) {
        if (errorLimiter_.allow()) {
            logger_ << "[ThreadRead] ERROR: Thread run method (" << e.what() << ")\n";
        }
    }
}

// Dữ liệu vừa đọc -> buffer -> từng message kết thúc ";;" -> send_queue(). Giống hệt ThreadTcpClient::process_data()
void ThreadRead::process_data(const std::string& data) {
    if (!utils::is_ascii(data)) {
        if (errorLimiter_.allow()) {
            logger_ << "[ThreadRead] ERROR: Non-ascii data dropped\n";
        }
        return;
    }
    buffer_ += data;

    std::size_t pos;
    while ((pos = buffer_.find(";;")) != std::string::npos) {
        const std::string msg = utils::strip(buffer_.substr(0, pos));
        buffer_.erase(0, pos + 2);
        if (msg.empty()) continue;

        try {
            send_queue(msg);
        } catch (const std::exception& e) {
            logger_ << "[ThreadRead] ERROR: Processing message " << msg
                    << " (" << e.what() << ")\n";
        }
    }

    if (buffer_.size() > kMaxBufferSize) {
        buffer_.clear();
        if (errorLimiter_.allow()) {
            logger_ << "[ThreadRead] WARNING: message too long; buffer cleared\n";
        }
    }
}

// ==================================== SENDING =======================================
// Bóc tách một message và gửi lên gateway. Giống hệt ThreadTcpClient::send_queue() trừ các action.
void ThreadRead::send_queue(const std::string& buff) {
    const auto colon = buff.find(':');
    if (buff.find('@') == std::string::npos || colon == std::string::npos) return;

    // action chỉ giữ chữ và số
    std::string action;
    for (char c : buff.substr(0, colon)) {
        if (std::isalnum(static_cast<unsigned char>(c))) action += c;
    }
    const std::vector<std::string> splittedValue = utils::split(buff.substr(colon + 1), ';');

    if (debugging_) logger_ << "[ThreadRead] " << buff << '\n';

    // Thiếu field hoặc sai kiểu thì ném exception, process_data() bắt và log
    if (action == "imu") {
        // x;y;yaw
        utils::require_fields(splittedValue, 3);
        ImuData imu;
        imu.x   = utils::to_float(splittedValue.at(0));
        imu.y   = utils::to_float(splittedValue.at(1));
        imu.yaw = utils::to_float(splittedValue.at(2));
        imuDataSender_.send(imu);
    }

    else if (action == "hall") {
        utils::require_fields(splittedValue, 1);
        const float speed = utils::to_float(splittedValue.at(0));
        currentSpeedSender_.send(speed);
    }

    // THÊM ACTION MỚI Ở ĐÂY: else if (action == "battery") { ... }
}

} // namespace hardware::serialhandler
