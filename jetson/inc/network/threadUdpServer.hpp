#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <netinet/in.h>

#include <hardware/camera/frameBuffer.hpp>
#include <network/messageConverter.hpp>
#include <network/udpServer.hpp>
#include <templates/threadWithStop.hpp>
#include <utils/messages/allMessages.hpp>
#include <utils/messages/messageHandlerSubscriber.hpp>
#include <utils/queues.hpp>
#include <utils/logConfig.hpp>

namespace network {

// Gửi ra UDP hai luồng ảnh và các giá trị telemetry dạng text.
// Subscribe (receiver "UdpServer"): CameraFrame, LaneCameraFrame (ảnh lấy từ FrameBuffer), CurrentSpeed, Speed,
// ImuData, Steer. Bật/tắt do ThreadNetworkHandler gọi set_subscribed() + pause()/resume().
//
// Ảnh: giữ nguyên kích thước (cam người lái 960x540, cam bám làn ĐÃ NẮN MÉO 640x480), nén JPEG, cắt gói rồi gửi. Mỗi
// luồng ảnh tới một cổng riêng của client, có frameId riêng. main.py trên PC ghép gói, dò làn trên ảnh cam bám làn
// (không khử méo lại) và hiện cả hai ảnh.
// Định dạng gói (big-endian): [frameId u32][chunkIndex u16][totalChunks u16][payload ≤ 1300 byte].
// Text: mỗi giá trị mới là một gói "@action:...;;\r\n" tới cổng text RIÊNG (không chung cổng với ảnh camera),
// giống hệt ThreadWrite, chỉ khác bước gửi:
//   @speed:v (đo)  @ref:v (lệnh #speed)  @steer:độ (+ = phải)  @imu:x;y;yaw
// Socket không bind (cổng nguồn do hệ điều hành cấp). Chỉ gửi tới client đã add_client(); server không đọc datagram.
class ThreadUdpServer : public templates::ThreadWithStop {
public:
    // frames: ảnh cam người lái; laneFrames: ảnh cam bám làn đã nắn méo (buffer của ThreadCameraHandler).
    // Mở socket thất bại thì thử lại mỗi kRebindInterval.
    ThreadUdpServer(utils::QueuesList&                             queuesList,
                    std::shared_ptr<hardware::camera::FrameBuffer> frames,
                    std::shared_ptr<hardware::camera::FrameBuffer> laneFrames,
                    std::ostream&                                  logger    = std::cerr,
                    bool                                           debugging = false);
    ~ThreadUdpServer() override;

    // true nếu socket đã mở
    bool is_open() const { return open_.load(); }

    // Tắt: set_subscribed(false) rồi pause(); bật: set_subscribed(true) rồi resume(). Chỉ gọi từ thread của handler.
    void set_subscribed(bool on);

    // Thêm client: cam người lái tới ip:port, cam bám làn tới lanePort, text tới textPort (0: không nhận).
    // ip = "any" hoặc "0.0.0.0": broadcast 255.255.255.255, máy nào trong LAN đang nghe các cổng đều nhận được.
    // false nếu ip không hợp lệ, port = 0 hoặc hai cổng ảnh trùng nhau. textPort được phép trùng cổng khác. Thread-safe.
    bool add_client(const std::string& ip, std::uint16_t port, std::uint16_t lanePort = 0,
                    std::uint16_t textPort = 0);

protected:
    void thread_work() override;

private:
    using Clock = std::chrono::steady_clock;

    // Một luồng ảnh gửi đi. clients được bảo vệ bởi clientsMutex_.
    struct ImageStream {
        const char*                                    name;
        std::shared_ptr<hardware::camera::FrameBuffer> frames;
        std::vector<sockaddr_in>                       clients;
        std::uint64_t                                  lastSentSeq = 0;   // seq ảnh đã gửi gần nhất
        std::uint32_t                                  frameId     = 0;   // id trong header gói UDP
    };

    void try_open();
    void relay_subscribed_values();        // giá trị mới -> gói text
    void add_endpoint(ImageStream& stream, const sockaddr_in& addr);   // gọi khi đang giữ clientsMutex_
    bool has_clients(ImageStream& stream);
    void send_latest_frame(ImageStream& stream);
    void send_jpeg(ImageStream& stream, const std::vector<unsigned char>& jpeg);

    bool send_command(const std::string& action, const std::vector<CommandArg>& args);
    void send_speed(float speed);
    void send_ref(float speed);
    void send_imu(const utils::messages::ImuData& imu);
    void send_steer(float steer);
    // THÊM BUILDER MỚI Ở ĐÂY
    bool send_text(const std::string& text);

    std::ostream&    logger_;
    bool             debugging_;
    MessageConverter converter_;

    // Hủy sau khi thread đã dừng (destructor gọi stop() + join() trước)
    utils::messages::MessageHandlerSubscriber frameSub_;
    utils::messages::MessageHandlerSubscriber laneFrameSub_;
    utils::messages::MessageHandlerSubscriber currentSpeedSub_;
    utils::messages::MessageHandlerSubscriber speedCmdSub_;
    utils::messages::MessageHandlerSubscriber imuSub_;
    utils::messages::MessageHandlerSubscriber steerSub_;

    UdpServer         server_;                 // chỉ thread của lớp này được đụng sau constructor
    std::atomic<bool> open_{false};
    bool              subscribed_ = true;      // constructor đã subscribeNow = true; chỉ thread của handler đụng vào
    Clock::time_point nextBind_{};

    std::mutex  clientsMutex_;                 // bảo vệ clients của hai luồng ảnh + textClients_ (add_client() gọi từ thread khác)
    ImageStream camera_;                       // ảnh cam người lái
    ImageStream lane_;                         // ảnh cam bám làn đã nắn méo
    std::vector<sockaddr_in> textClients_;     // nơi nhận gói text (cổng riêng)

    // Chỉ thread của lớp này đụng vào các biến dưới đây
    std::vector<unsigned char> jpegBuf_;
    std::vector<unsigned char> packet_;

    utils::RateLimiter errorLimiter_{utils::logcfg::kErrorCooldown};

    static constexpr std::chrono::seconds kRebindInterval{1};
};

} // namespace network
