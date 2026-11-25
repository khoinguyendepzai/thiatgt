#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <initializer_list>
#include <iostream>
#include <memory>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include <hardware/camera/frameBuffer.hpp>
#include <perception/cameraUndistorter.hpp>
#include <templates/threadWithStop.hpp>
#include <utils/messages/allMessages.hpp>
#include <utils/messages/messageHandlerSender.hpp>
#include <utils/queues.hpp>
#include <utils/logConfig.hpp>

namespace hardware::camera {

// Một thread đọc cả hai camera (không có thread con), luôn chạy, không phụ thuộc mode (không subscribe StateChange):
//   - Camera bám làn (laneSource): nắn méo quang sai (perception::CameraUndistorter). Mỗi frame: ảnh thô vào
//     raw_buffer() (không thông báo, main lưu test.jpg để calib), ảnh đã nắn méo vào lane_frame_buffer() +
//     LaneCameraFrame (ThreadUdpServer gửi về PC; main.py dò làn trên ảnh này, không khử méo lại). Frame khác kích thước
//     calib: không nắn méo (map chỉ đúng ở cỡ calib), gửi nguyên ảnh thô và log lỗi.
//   - Camera nhìn đường (source): ảnh người lái vào frame_buffer() + CameraFrame (ThreadUdpServer gửi về PC).
// Mỗi camera tự mở lại khi mất kết nối, mỗi kReconnectInterval. Mỗi vòng đọc lần lượt từng camera; một camera mất thì
// camera kia vẫn chạy (chỉ bị chặn trong lúc open() camera mất). Dò làn, bird's-eye và mask không còn ở Jetson.
class ThreadCameraHandler : public templates::ThreadWithStop {
public:
    // source: camera nhìn đường. laneSource: camera bám làn (bird's-eye).
    // calib: thông số calib ống kính (nắn méo), phải khớp laneSource.
    ThreadCameraHandler(utils::QueuesList&                   queuesList,
                        std::string                          source,
                        std::string                          laneSource,
                        const perception::CameraCalibration& calib,
                        int                                  apiPreference = cv::CAP_GSTREAMER,
                        std::ostream&                        logger        = std::cerr,
                        bool                                 debugging     = false);
    ~ThreadCameraHandler() override;

    // Camera nhìn đường đang mở
    bool is_opened() const { return road_.opened.load(); }

    // Camera bám làn đang mở
    bool is_lane_camera_opened() const { return lane_.opened.load(); }

    // Frame mới nhất của camera nhìn đường (thông báo CameraFrame)
    std::shared_ptr<FrameBuffer> frame_buffer() const { return frameBuffer_; }

    // Frame đã nắn méo mới nhất của camera bám làn (thông báo LaneCameraFrame)
    std::shared_ptr<FrameBuffer> lane_frame_buffer() const { return laneFrameBuffer_; }

    // Frame thô (chưa nắn méo) mới nhất của camera bám làn, không có thông báo. Dùng lưu test.jpg: calib/pick_marks.py
    // tự nắn méo ảnh này bằng bộ số trong main.cpp, nên phải là ảnh thô.
    std::shared_ptr<FrameBuffer> raw_buffer() const { return rawBuffer_; }

protected:
    void thread_work() override;   // đọc hai camera, publish buffer và gửi thông báo

private:
    using Clock = std::chrono::steady_clock;

    // Trạng thái mở / đọc / mở lại của một camera. Chỉ thread của handler đụng vào (trừ opened).
    struct CameraSource {
        CameraSource(const char* name, std::string source) : name(name), source(std::move(source)) {}

        const char*       name;      // để log: "Lane camera" / "Road camera"
        std::string       source;    // chuỗi nguồn cho cv::VideoCapture
        cv::VideoCapture  capture;
        cv::Mat           raw;       // chuyển quyền sở hữu ra ngoài sau mỗi frame, read() kế tiếp cấp phát mới
        std::atomic<bool> opened{false};
        int               failures         = 0;       // số lần read() thất bại liên tiếp
        bool              firstFrameLogged = false;
        Clock::time_point nextAttempt{};             // thời điểm tiếp theo được phép thử mở
        utils::RateLimiter errorLimiter{utils::logcfg::kErrorCooldown};
    };

    // Đọc một frame của cam. Chưa mở thì thử mở (đến hạn); mất cam thì đóng và bỏ ảnh cũ trong các buffer.
    // Trả về ảnh mới (cv::Mat riêng, không bị read() sau ghi đè) hoặc nullptr. Không ngủ.
    std::shared_ptr<cv::Mat> read_frame(CameraSource& cam, std::initializer_list<FrameBuffer*> buffers);
    void try_open(CameraSource& cam);
    void handle_lost(CameraSource& cam, std::initializer_list<FrameBuffer*> buffers);
    void log_frame(CameraSource& cam, const utils::messages::FrameInfo& info);

    void process_lane(std::shared_ptr<cv::Mat> raw);   // nắn méo, publish, gửi LaneCameraFrame
    void process_road(std::shared_ptr<cv::Mat> raw);   // publish, gửi CameraFrame

    int           apiPreference_;
    std::ostream& logger_;
    bool          debugging_;

    // Thư viện nắn méo quang sai thấu kính
    perception::CameraUndistorter undistorter_;

    CameraSource lane_;   // camera bám làn
    CameraSource road_;   // camera nhìn đường

    // Buffer và Sender
    std::shared_ptr<FrameBuffer>          laneFrameBuffer_;   // cam bám làn đã nắn méo, ThreadUdpServer gửi về PC
    std::shared_ptr<FrameBuffer>          rawBuffer_;         // cam bám làn thô, không thông báo
    std::shared_ptr<FrameBuffer>          frameBuffer_;       // cam nhìn đường
    utils::messages::MessageHandlerSender laneFrameSender_;   // MessageName::LaneCameraFrame
    utils::messages::MessageHandlerSender frameSender_;       // MessageName::CameraFrame

    // Thời gian chờ giữa các lần thử mở lại camera
    static constexpr std::chrono::seconds      kReconnectInterval{1};
    // Nghỉ khi cả hai camera chưa mở (tránh quay vòng liên tục)
    static constexpr std::chrono::milliseconds kIdleSleep{100};
    // Nghỉ ngắn khi vòng này không đọc được frame nào
    static constexpr std::chrono::milliseconds kReadRetrySleep{5};
    // Quá số lần read() thất bại liên tiếp này thì coi như mất camera
    static constexpr int                       kMaxReadFailures = 200;

    utils::RateLimiter sizeErrorLimiter_{utils::logcfg::kErrorCooldown};   // frame khác kích thước calib
};

} // namespace hardware::camera
