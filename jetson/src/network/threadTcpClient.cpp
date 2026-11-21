#include <network/threadTcpClient.hpp>

#include <cerrno>
#include <cctype>
#include <cstring>
#include <exception>
#include <utility>
#include <vector>

#include <statemachine/systemMode.hpp>
#include <utils/stringUtils.hpp>

namespace network {

using utils::messages::MessageName;

// ===================================== INIT =========================================
ThreadTcpClient::ThreadTcpClient(utils::QueuesList& queuesList, statemachine::StateMachine& sm,
                                 std::string host, std::uint16_t port,
                                 std::ostream& logger, bool debugging)
    : templates::ThreadWithStop(0.01),
      host_(std::move(host)),
      port_(port),
      logger_(logger),
      debugging_(debugging),
      steerSender_(queuesList, MessageName::Steer),
      speedSender_(queuesList, MessageName::Speed),
      directorySender_(queuesList, MessageName::Directory),
      distanceSender_(queuesList, MessageName::Distance),
      sm_(sm) {}

ThreadTcpClient::~ThreadTcpClient() {
    stop();
    join();
}

// ====================================== RUN ==========================================
void ThreadTcpClient::thread_work() {
    std::lock_guard<std::mutex> lock(ioMutex_);

    // pause() có thể xảy ra ngay sau khi thread qua cửa kiểm tra của ThreadWithStop,
    // nên một vòng cuối vẫn lọt vào đây. Đã bị pause thì không được kết nối.
    if (is_paused()) return;

    if (!client_.is_open()) {
        if (Clock::now() >= nextAttempt_) try_connect();
        return;
    }
    read_available();

    // Vẫn nối nhưng PC ngừng gửi lệnh (main_master treo, Wi-Fi nghẽn): dừng xe
    if (driving_ && Clock::now() - lastVcb_ > kCommandTimeout) send_stop_command("no #vcb for 300 ms");
}

void ThreadTcpClient::send_stop_command(const char* reason) {
    driving_ = false;
    steerSender_.send(0.0f);
    speedSender_.send(0.0f);
    directorySender_.send(0);
    logger_ << "[TcpClient] STOP sent (speed 0, steer 0, dir 0): " << reason << '\n';
}

// Gọi từ thread khác (handler), SAU pause(). Mọi thứ đụng vào client_ đều nằm dưới ioMutex_.
void ThreadTcpClient::disconnect_now() {
    std::lock_guard<std::mutex> lock(ioMutex_);
    if (client_.is_open()) disconnect("disabled by mode");
}

void ThreadTcpClient::try_connect() {
    nextAttempt_ = Clock::now() + kReconnectInterval;
    buffer_.clear();
    if (!client_.connect(host_, port_, kConnectTimeout)) {
        const int err = errno;   // lấy ngay, trước khi ghi log có thể làm đổi errno
        if (errorLimiter_.allow()) {
            logger_ << "[TcpClient] ERROR: Cannot connect to " << host_ << ':'
                    << port_ << " (" << std::strerror(err) << "), will retry\n";
        }
        return;
    }
    connected_ = true;
    logger_ << "[TcpClient] Connected to " << host_ << ':' << port_ << '\n';
}

void ThreadTcpClient::disconnect(const char* reason) {
    client_.close();
    connected_ = false;
    buffer_.clear();
    nextAttempt_ = Clock::now() + kReconnectInterval;
    logger_ << "[TcpClient] Disconnected (" << reason << ")\n";
    if (driving_) send_stop_command(reason);
}

// TCP là stream: process_data() cắt message theo ";;", không theo ranh giới packet
void ThreadTcpClient::read_available() {
    std::string data;
    for (int i = 0; i < kMaxReadsPerLoop; ++i) {
        const auto status = client_.read_some(data);
        if (status == TcpReadStatus::NoData) return;
        if (status == TcpReadStatus::Disconnected) {
            disconnect("connection closed by server");
            return;
        }
        process_data(data);
    }
}

// Dữ liệu vừa đọc -> buffer -> từng message kết thúc ";;" -> send_queue(). Giống hệt ThreadRead::process_data().
void ThreadTcpClient::process_data(const std::string& data) {
    if (!utils::is_ascii(data)) {
        if (errorLimiter_.allow()) {
            logger_ << "[TcpClient] ERROR: Non-ascii data dropped\n";
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
            logger_ << "[TcpClient] ERROR: Processing message " << msg
                    << " (" << e.what() << ")\n";
        }
    }

    if (buffer_.size() > kMaxBufferSize) {
        buffer_.clear();
        if (errorLimiter_.allow()) {
            logger_ << "[TcpClient] WARNING: message too long; buffer cleared\n";
        }
    }
}

// ==================================== SENDING =======================================
// Bóc tách một message và xử lý. Giống ThreadRead::send_queue() trừ các action và ký tự đầu: main_master.py gửi
// "#vcb:...;;" (CMD_PREFIX = "#"); vẫn nhận '@' cho bản main_master / công cụ cũ.
void ThreadTcpClient::send_queue(const std::string& buff) {
    const auto colon = buff.find(':');
    if (buff.find_first_of("#@") == std::string::npos || colon == std::string::npos) return;

    // action chỉ giữ chữ và số
    std::string action;
    for (char c : buff.substr(0, colon)) {
        if (std::isalnum(static_cast<unsigned char>(c))) action += c;
    }
    const std::vector<std::string> splittedValue = utils::split(buff.substr(colon + 1), ';');

    if (debugging_) logger_ << "[TcpClient] " << buff << '\n';

    // Thiếu field hoặc sai kiểu thì ném exception, process_data() bắt và log
    if (action == "vcb") {
        // speed;steer;dir: đổi đủ 3 giá trị rồi mới gửi
        utils::require_fields(splittedValue, 3);
        const float speed = utils::to_float(splittedValue.at(0));
        const float steer = utils::to_float(splittedValue.at(1));
        const int   dir   = utils::to_int(splittedValue.at(2));
        steerSender_.send(steer);
        speedSender_.send(speed);
        directorySender_.send(dir);
        lastVcb_ = Clock::now();
        driving_ = true;
    }

    else if (action == "state" || action == "mode") {
        // Đổi state hệ thống. Tên sai thì không gửi, vì StateChange tên lạ sẽ tắt hết thread con, kể cả TCP.
        utils::require_fields(splittedValue, 1);
        const std::string name  = utils::strip(splittedValue.at(0));
        const std::string upper = utils::to_upper(name);
        if (const auto mode = statemachine::mode_from_string(upper)) {
            logger_ << "[TcpClient] Remote state request: " << upper << '\n';
            sm_.request_mode(*mode);   // false nếu đã ở state này rồi: không làm gì
        } else if (errorLimiter_.allow()) {
            logger_ << "[TcpClient] WARNING: Unknown state requested remotely: '" << name << "'\n";
        }
    }

    else if (action == "distance") {
        // Khoảng cách tới vật cản (m) do YOLO trên máy remote tính; < 0 = không thấy vật cản.
        // Gửi trong mọi mode; hiện chưa ai subscribe (phần điều khiển code sau).
        utils::require_fields(splittedValue, 1);
        const float distance = utils::to_float(splittedValue.at(0));
        distanceSender_.send(distance);
    }

    // THÊM ACTION MỚI Ở ĐÂY: else if (action == "xxx") { ... }
}

} // namespace network
