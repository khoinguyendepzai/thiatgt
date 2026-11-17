#include <network/threadNetworkHandler.hpp>

#include <any>
#include <utility>

#include <statemachine/systemMode.hpp>

namespace network {

using utils::messages::MessageName;

// ===================================== INIT =========================================
ThreadNetworkHandler::ThreadNetworkHandler(utils::QueuesList&                             queuesList,
                                           statemachine::StateMachine&                    sm,
                                           std::shared_ptr<hardware::camera::FrameBuffer> frames,
                                           std::shared_ptr<hardware::camera::FrameBuffer> laneFrames,
                                           std::string                                    tcpHost,
                                           std::uint16_t                                  tcpPort,
                                           std::ostream&                                  logger,
                                           bool                                           debugging)
    : templates::ThreadWithStop(0.05)
    , logger_(logger)
    // receiver phải khác các handler khác: gateway khóa theo (Owner, msgID, receiver)
    , stateChangeSub_(queuesList, MessageName::StateChange, "lastonly", true, "NetworkHandler")
    , udpTh_(std::make_unique<ThreadUdpServer>(queuesList, std::move(frames), std::move(laneFrames), logger,
                                               debugging))
    , tcpTh_(std::make_unique<ThreadTcpClient>(queuesList, sm, std::move(tcpHost), tcpPort, logger, debugging))
{
    udpTh_->start();
    tcpTh_->start();
    apply_thread_state();   // áp mặc định: UDP bật, TCP tắt (pause + đóng kết nối nếu lỡ nối)
}

// Dừng vòng quản lý trước; hai thread con tự stop + join trong destructor
ThreadNetworkHandler::~ThreadNetworkHandler() {
    stop();
    join();
}

// ===================================== STATE CHANGE ==========================================
void ThreadNetworkHandler::state_change_handler() {
    const auto msg = stateChangeSub_.receive();
    if (!msg) return;

    if (const auto* state = std::any_cast<std::string>(&*msg)) {
        // Tên state không hợp lệ -> tắt cả hai
        const auto* cfg = statemachine::find_mode_config(*state);
        const bool  udp = cfg && cfg->networkHandler.enabledUdp;
        const bool  tcp = cfg && cfg->networkHandler.enabledTcp;

        if (udp != modeEnabledUdp_ || tcp != modeEnabledTcp_) {
            logger_ << "[NetworkHandler] UDP " << (udp ? "enabled" : "disabled")
                    << ", TCP " << (tcp ? "enabled" : "disabled")
                    << " (state " << *state << ")\n";
        }
        modeEnabledUdp_ = udp;
        modeEnabledTcp_ = tcp;
        apply_thread_state();
    }
}

// UDP: set_subscribed trước pause()/resume(). TCP: tắt thì pause() rồi disconnect_now(), bật thì resume().
void ThreadNetworkHandler::apply_thread_state() {
    if (modeEnabledUdp_) {
        udpTh_->set_subscribed(true);
        udpTh_->resume();
    } else {
        udpTh_->set_subscribed(false);
        udpTh_->pause();
    }

    if (modeEnabledTcp_) {
        tcpTh_->resume();
    } else {
        tcpTh_->pause();
        tcpTh_->disconnect_now();
    }
}

} // namespace network
