#pragma once

#include <cstdint>
#include <iostream>
#include <memory>
#include <string>

#include <hardware/camera/frameBuffer.hpp>
#include <network/threadTcpClient.hpp>
#include <network/threadUdpServer.hpp>
#include <statemachine/stateMachine.hpp>
#include <templates/threadWithStop.hpp>
#include <utils/messages/allMessages.hpp>
#include <utils/messages/messageHandlerSubscriber.hpp>
#include <utils/queues.hpp>

namespace network {

// Sở hữu ThreadUdpServer (gửi ảnh + telemetry) và ThreadTcpClient (nhận @vcb, @state, @mode); bật/tắt chúng
// theo StateChange:
//   - UDP: tắt = set_subscribed(false) rồi pause(); bật = set_subscribed(true) rồi resume().
//   - TCP: tắt = pause() rồi disconnect_now(); bật = resume().
// Trước StateChange đầu tiên: UDP bật, TCP tắt. Tên state không hợp lệ -> tắt cả hai.
class ThreadNetworkHandler : public templates::ThreadWithStop {
public:
    // sm: để ThreadTcpClient đổi state (phải sống lâu hơn handler). frames / laneFrames: frame_buffer() và
    // lane_frame_buffer() của ThreadCameraHandler. tcpHost: IPv4 dạng số của server TCP.
    ThreadNetworkHandler(utils::QueuesList&                             queuesList,
                         statemachine::StateMachine&                    sm,
                         std::shared_ptr<hardware::camera::FrameBuffer> frames,
                         std::shared_ptr<hardware::camera::FrameBuffer> laneFrames,
                         std::string                                    tcpHost,
                         std::uint16_t                                  tcpPort,
                         std::ostream&                                  logger    = std::cerr,
                         bool                                           debugging = false);
    ~ThreadNetworkHandler() override;

    // Chuyển tiếp sang ThreadUdpServer
    bool is_udp_open() const { return udpTh_->is_open(); }
    bool is_udp_enabled() const { return !udpTh_->is_paused(); }
    bool add_udp_client(const std::string& ip, std::uint16_t port, std::uint16_t lanePort = 0,
                        std::uint16_t textPort = 0) {
        return udpTh_->add_client(ip, port, lanePort, textPort);
    }

    // Chuyển tiếp sang ThreadTcpClient
    bool is_tcp_connected() const { return tcpTh_->is_connected(); }
    bool is_tcp_enabled() const { return !tcpTh_->is_paused(); }

protected:
    void state_change_handler() override;   // nhận StateChange, bật/tắt thread con

private:
    void apply_thread_state();   // set_subscribed + pause/resume UDP, pause/resume TCP

    std::ostream& logger_;

    // Khai báo trước thread con để được hủy sau chúng
    utils::messages::MessageHandlerSubscriber stateChangeSub_;

    std::unique_ptr<ThreadUdpServer> udpTh_;
    std::unique_ptr<ThreadTcpClient> tcpTh_;

    // State hiện tại cho phép UDP gửi frame / TCP nhận message (chỉ thread của handler đụng vào).
    // Trước StateChange đầu tiên: UDP bật, TCP tắt (TCP bật sẵn thì lúc khởi động xe thử nối tới server một lần,
    // chặn tới kConnectTimeout, dù mode nào cũng tắt TCP)
    bool modeEnabledUdp_ = true;
    bool modeEnabledTcp_ = false;
};

} // namespace network
