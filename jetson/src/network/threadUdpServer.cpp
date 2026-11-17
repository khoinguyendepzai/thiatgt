#include <network/threadUdpServer.hpp>

#include <algorithm>
#include <any>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <optional>
#include <utility>

#include <arpa/inet.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <utils/stringUtils.hpp>

namespace network {

using utils::messages::FrameInfo;
using utils::messages::ImuData;
using utils::messages::MessageHandlerSubscriber;
using utils::messages::MessageName;

namespace {

// Chất lượng JPEG và kích thước payload mỗi gói
constexpr int         kJpegQuality  = 85;
constexpr std::size_t kChunkPayload = 1300;
constexpr std::size_t kHeaderSize   = 8;

// "ip:port" để ghi log
std::string endpoint_to_string(const sockaddr_in& a) {
    char ip[INET_ADDRSTRLEN] = {};
    inet_ntop(AF_INET, &a.sin_addr, ip, sizeof(ip));
    return std::string(ip) + ":" + std::to_string(ntohs(a.sin_port));
}

bool same_endpoint(const sockaddr_in& a, const sockaddr_in& b) {
    return a.sin_addr.s_addr == b.sin_addr.s_addr && a.sin_port == b.sin_port;
}

void put_u16(unsigned char* p, std::uint16_t v) {
    p[0] = static_cast<unsigned char>((v >> 8) & 0xFF);
    p[1] = static_cast<unsigned char>(v & 0xFF);
}

void put_u32(unsigned char* p, std::uint32_t v) {
    p[0] = static_cast<unsigned char>((v >> 24) & 0xFF);
    p[1] = static_cast<unsigned char>((v >> 16) & 0xFF);
    p[2] = static_cast<unsigned char>((v >> 8) & 0xFF);
    p[3] = static_cast<unsigned char>(v & 0xFF);
}

// Giá trị mới nhất kiểu T, nullopt nếu sai kiểu hoặc không có tin. Giống hệt receive_as() trong threadWrite.cpp.
template <typename T>
std::optional<T> receive_as(MessageHandlerSubscriber& sub) {
    const auto msg = sub.receive();
    if (!msg) return std::nullopt;

    if (const auto* p = std::any_cast<T>(&*msg)) return *p;
    return std::nullopt;
}

} // namespace

// ===================================== INIT =========================================
ThreadUdpServer::ThreadUdpServer(utils::QueuesList&                             queuesList,
                                 std::shared_ptr<hardware::camera::FrameBuffer> frames,
                                 std::shared_ptr<hardware::camera::FrameBuffer> laneFrames,
                                 std::ostream&                                  logger,
                                 bool                                           debugging)
    : templates::ThreadWithStop(0.005)
    , logger_(logger)
    , debugging_(debugging)
    , converter_(logger)
    // receiver phải khác các subscriber khác của cùng message (SerialWriter, CameraViewer)
    , frameSub_       (queuesList, MessageName::CameraFrame,     "lastonly", true, "UdpServer")
    , laneFrameSub_   (queuesList, MessageName::LaneCameraFrame, "lastonly", true, "UdpServer")
    , currentSpeedSub_(queuesList, MessageName::CurrentSpeed,    "lastonly", true, "UdpServer")
    , speedCmdSub_    (queuesList, MessageName::Speed,           "lastonly", true, "UdpServer")
    , imuSub_         (queuesList, MessageName::ImuData,         "lastonly", true, "UdpServer")
    , steerSub_       (queuesList, MessageName::Steer,           "lastonly", true, "UdpServer")
    , camera_{"camera",      std::move(frames),     {}}
    , lane_  {"lane camera", std::move(laneFrames), {}}
{
    try_open();
}

// Dừng thread trước khi server_, camera_/lane_, subscriber... bị hủy.
ThreadUdpServer::~ThreadUdpServer() {
    stop();
    join();
}

// nextBind_ được đặt trước, nên mọi nhánh lỗi đều chờ đủ kRebindInterval.
void ThreadUdpServer::try_open() {
    nextBind_ = Clock::now() + kRebindInterval;

    if (!server_.open()) {
        const int err = errno;   // lấy ngay, trước khi ghi log có thể làm đổi errno
        if (errorLimiter_.allow()) {
            logger_ << "[UdpServer] ERROR: Cannot open UDP socket (" << std::strerror(err) << "), will retry\n";
        }
        return;
    }

    open_ = true;
    logger_ << "[UdpServer] UDP socket opened (source port assigned by OS)\n";
}

// ===================================== CLIENTS ======================================
bool ThreadUdpServer::add_client(const std::string& ip, std::uint16_t port, std::uint16_t lanePort,
                                 std::uint16_t textPort) {
    if (port == 0 || lanePort == port) return false;

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(port);
    // "any" hoặc "0.0.0.0": gửi tới mọi máy trong LAN (broadcast). Linux hiểu 0.0.0.0 là chính máy này nên phải đổi.
    const bool toAll = (ip == "any" || ip == "0.0.0.0");
    if (inet_pton(AF_INET, toAll ? "255.255.255.255" : ip.c_str(), &addr.sin_addr) != 1) return false;

    std::lock_guard<std::mutex> lock(clientsMutex_);
    add_endpoint(camera_, addr);
    if (textPort != 0) {
        sockaddr_in textAddr = addr;
        textAddr.sin_port    = htons(textPort);
        const bool exists = std::any_of(textClients_.begin(), textClients_.end(),
            [&](const sockaddr_in& c) { return same_endpoint(c, textAddr); });
        if (!exists) {
            textClients_.push_back(textAddr);
            logger_ << "[UdpServer] Client added: " << endpoint_to_string(textAddr) << " (text)\n";
        }
    }
    if (lanePort != 0) {
        sockaddr_in laneAddr = addr;
        laneAddr.sin_port    = htons(lanePort);
        add_endpoint(lane_, laneAddr);
    }
    return true;
}

void ThreadUdpServer::add_endpoint(ImageStream& stream, const sockaddr_in& addr) {
    const bool exists = std::any_of(stream.clients.begin(), stream.clients.end(),
        [&](const sockaddr_in& c) { return same_endpoint(c, addr); });
    if (!exists) {
        stream.clients.push_back(addr);
        logger_ << "[UdpServer] Client added: " << endpoint_to_string(addr) << " (" << stream.name << ")\n";
    }
}

bool ThreadUdpServer::has_clients(ImageStream& stream) {
    std::lock_guard<std::mutex> lock(clientsMutex_);
    return !stream.clients.empty();
}

void ThreadUdpServer::set_subscribed(bool on) {
    if (on == subscribed_) return;
    subscribed_ = on;

    for (auto* sub : {&frameSub_, &laneFrameSub_, &currentSpeedSub_, &speedCmdSub_, &imuSub_, &steerSub_}) {
        if (on) {
            sub->empty();           // bỏ frame/speed/steer cũ còn sót
            sub->subscribe();
        } else {
            sub->unsubscribe();
        }
    }
}

// ===================================== RUN ==========================================
void ThreadUdpServer::thread_work() {
    // Luôn rút cạn subscriber, kể cả khi chưa bind hoặc chưa có client, để pipe không phình
    relay_subscribed_values();
    const bool newFrame     = receive_as<FrameInfo>(frameSub_).has_value();
    const bool newLaneFrame = receive_as<FrameInfo>(laneFrameSub_).has_value();

    if (!server_.is_open()) {
        if (Clock::now() >= nextBind_) try_open();
        return;
    }

    if (newFrame)     send_latest_frame(camera_);
    if (newLaneFrame) send_latest_frame(lane_);
}

// Mỗi vòng lấy giá trị mới nhất, đổi thành gói text rồi gửi (giống ThreadWrite::thread_work())
void ThreadUdpServer::relay_subscribed_values() {
    if (const auto v = receive_as<float>(currentSpeedSub_)) send_speed(*v);
    if (const auto v = receive_as<float>(speedCmdSub_))     send_ref(*v);
    if (const auto v = receive_as<ImuData>(imuSub_))        send_imu(*v);
    if (const auto v = receive_as<float>(steerSub_))        send_steer(*v);

    // THÊM KHỐI MỚI Ở ĐÂY
}

// ==================================== SENDING =======================================
// Đổi (action, args) thành lệnh text rồi gửi. Giống hệt ThreadWrite::send_command() trừ bước gửi.
bool ThreadUdpServer::send_command(const std::string& action, const std::vector<CommandArg>& args) {
    const auto cmd = converter_.get_command(action, args);   // sai kiểu/thiếu tham số: converter đã log
    if (!cmd) return false;

    // Gửi ra mạng: mỗi lệnh là một gói UDP cho mọi client
    if (!send_text(*cmd)) {
        if (errorLimiter_.allow()) {
            logger_ << "[UdpServer] WARNING: Text packet dropped (send buffer full or network error)\n";
        }
        return false;
    }

    if (debugging_) logger_ << "[UdpServer] " << utils::strip(*cmd) << '\n';
    return true;
}

// Tên action và tên tham số phải khớp đúng bảng commands() trong messageConverter.cpp
void ThreadUdpServer::send_speed(float speed) { send_command("speed", {{"speed", speed}}); }
void ThreadUdpServer::send_ref(float speed)   { send_command("ref",   {{"speed", speed}}); }
// "@imu:x;y;yaw;;\r\n"
void ThreadUdpServer::send_imu(const ImuData& imu) {
    send_command("imu", {{"x", imu.x}, {"y", imu.y}, {"yaw", imu.yaw}});
}
void ThreadUdpServer::send_steer(float steer) { send_command("steer", {{"steer", steer}}); }

void ThreadUdpServer::send_latest_frame(ImageStream& stream) {
    if (!has_clients(stream)) return;      // không ai nghe: khỏi vẽ/nén cho tốn CPU

    // lastonly: ảnh trong buffer có thể mới hơn seq của thông báo; nếu đã gửi rồi thì bỏ qua.
    // image == nullptr: buffer vừa bị xóa (mất camera) -> không gửi ảnh cũ.
    const hardware::camera::Frame frame = stream.frames->latest();
    if (!frame.image || frame.info.seq <= stream.lastSentSeq) return;
    stream.lastSentSeq = frame.info.seq;

    // Gửi đúng kích thước ảnh trong buffer, không resize. frame.image chỉ đọc (dùng chung)
    const std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, kJpegQuality};
    if (!cv::imencode(".jpg", *frame.image, jpegBuf_, params)) return;

    send_jpeg(stream, jpegBuf_);
}

// Cắt JPEG thành các gói kChunkPayload byte kèm header 8 byte, gửi cho từng client của luồng ảnh.
// Giữ clientsMutex_ suốt lúc gửi: sendto không chặn nên rất ngắn, chỉ add_client() phải chờ.
void ThreadUdpServer::send_jpeg(ImageStream& stream, const std::vector<unsigned char>& jpeg) {
    if (jpeg.empty()) return;

    std::lock_guard<std::mutex> lock(clientsMutex_);

    const std::size_t total = (jpeg.size() + kChunkPayload - 1) / kChunkPayload;
    if (total > 0xFFFF) {                  // header chỉ có 16 bit cho totalChunks
        if (errorLimiter_.allow()) logger_ << "[UdpServer] ERROR: JPEG too large to send\n";
        return;
    }

    const std::uint32_t fid         = ++stream.frameId;   // mỗi luồng ảnh đếm riêng
    const auto          totalChunks = static_cast<std::uint16_t>(total);
    std::size_t         dropped     = 0;

    for (std::uint16_t i = 0; i < totalChunks; ++i) {
        const std::size_t offset  = static_cast<std::size_t>(i) * kChunkPayload;
        const std::size_t payload = std::min(jpeg.size() - offset, kChunkPayload);

        packet_.resize(kHeaderSize + payload);
        put_u32(packet_.data(),     fid);
        put_u16(packet_.data() + 4, i);
        put_u16(packet_.data() + 6, totalChunks);
        std::memcpy(packet_.data() + kHeaderSize, jpeg.data() + offset, payload);

        for (const auto& to : stream.clients) {
            if (!server_.send_to(to, packet_.data(), packet_.size())) ++dropped;
        }
    }

    if (dropped > 0 && errorLimiter_.allow()) {
        logger_ << "[UdpServer] WARNING: " << dropped
                << " packet(s) dropped (send buffer full or network error)\n";
    }
    if (debugging_ && fid % utils::logcfg::kUdpFrameLogEvery == 0) {
        logger_ << "[UdpServer] " << stream.name << " frame=" << fid << " bytes=" << jpeg.size()
                << " chunks=" << totalChunks << " clients=" << stream.clients.size() << '\n';
    }
}

// Gửi một gói text tới cổng text của mọi client. false nếu có gói bị rớt; chưa mở socket thì không gửi và không tính
// là rớt.
bool ThreadUdpServer::send_text(const std::string& text) {
    if (!server_.is_open()) return true;

    std::lock_guard<std::mutex> lock(clientsMutex_);
    bool sent = true;
    for (const auto& to : textClients_) {
        if (!server_.send_to(to, text.data(), text.size())) sent = false;
    }
    return sent;
}

} // namespace network
