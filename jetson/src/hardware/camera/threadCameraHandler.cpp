#include <hardware/camera/threadCameraHandler.hpp>

#include <thread>
#include <utility>

namespace hardware::camera {

using utils::messages::FrameInfo;
using utils::messages::MessageName;

// ===================================== INIT =========================================
ThreadCameraHandler::ThreadCameraHandler(utils::QueuesList&                   queuesList,
                                         std::string                          source,
                                         std::string                          laneSource,
                                         const perception::CameraCalibration& calib,
                                         int                                  apiPreference,
                                         std::ostream&                        logger,
                                         bool                                 debugging)
    : templates::ThreadWithStop(0.0)       // capture.read() tự chặn theo tốc độ camera
    , apiPreference_(apiPreference)
    , logger_(logger)
    , debugging_(debugging)
    , undistorter_(calib)
    , lane_("Lane camera", std::move(laneSource))
    , road_("Road camera", std::move(source))
    , laneFrameBuffer_(std::make_shared<FrameBuffer>())
    , rawBuffer_(std::make_shared<FrameBuffer>())
    , frameBuffer_(std::make_shared<FrameBuffer>())
    , laneFrameSender_(queuesList, MessageName::LaneCameraFrame)
    , frameSender_(queuesList, MessageName::CameraFrame)
{}

// Lớp con PHẢI stop() + join() trước khi member (capture, buffer...) bị hủy.
ThreadCameraHandler::~ThreadCameraHandler() {
    stop();
    join();
    lane_.capture.release();
    road_.capture.release();
}

// ===================================== RUN ==========================================
// Mỗi vòng đọc lần lượt cam bám làn rồi cam nhìn đường. read() chặn tới khi camera có frame, nên vòng lặp tự chạy
// theo tốc độ camera; không đọc được frame nào thì nghỉ ngắn (cả hai chưa mở: nghỉ lâu hơn).
void ThreadCameraHandler::thread_work() {
    bool gotFrame = false;

    if (auto raw = read_frame(lane_, {laneFrameBuffer_.get(), rawBuffer_.get()})) {
        process_lane(std::move(raw));
        gotFrame = true;
    }
    if (auto raw = read_frame(road_, {frameBuffer_.get()})) {
        process_road(std::move(raw));
        gotFrame = true;
    }

    if (!gotFrame) {
        const bool anyOpen = lane_.opened || road_.opened;
        std::this_thread::sleep_for(anyOpen ? kReadRetrySleep : kIdleSleep);
    }
}

std::shared_ptr<cv::Mat> ThreadCameraHandler::read_frame(CameraSource& cam,
                                                         std::initializer_list<FrameBuffer*> buffers) {
    // Chưa mở được (hoặc đã mất) -> thử mở lại mỗi kReconnectInterval
    if (!cam.opened) {
        if (Clock::now() >= cam.nextAttempt) try_open(cam);
        return nullptr;
    }

    if (!cam.capture.read(cam.raw) || cam.raw.empty()) {
        if (++cam.failures >= kMaxReadFailures) handle_lost(cam, buffers);
        return nullptr;
    }
    cam.failures = 0;

    // Chuyển cam.raw vào shared_ptr (không copy pixel): cam.raw rỗng, read() kế tiếp cấp phát ảnh mới, nên consumer
    // giữ ảnh bao lâu cũng không bị ghi đè.
    auto raw = std::make_shared<cv::Mat>(std::move(cam.raw));
    cam.raw  = cv::Mat();
    return raw;
}

void ThreadCameraHandler::process_lane(std::shared_ptr<cv::Mat> raw) {
    // Map nắn méo tính cho đúng cỡ calib; remap() luôn trả ảnh cỡ của map, nên frame sai cỡ mà vẫn remap thì ra ảnh
    // đúng cỡ nhưng sai hình, và PC không còn phát hiện được. Sai cỡ: gửi nguyên ảnh thô (main.py thấy sai cỡ).
    std::shared_ptr<cv::Mat> output = raw;
    if (raw->size() == undistorter_.image_size()) {
        output = std::make_shared<cv::Mat>();
        undistorter_.undistort(*raw, *output);   // ảnh mới mỗi frame: không cần clone()
    } else if (sizeErrorLimiter_.allow()) {
        logger_ << "[CameraHandler] ERROR: Lane camera frame is " << raw->cols << 'x' << raw->rows
                << " but calibration is " << undistorter_.image_size().width << 'x'
                << undistorter_.image_size().height
                << ", not undistorted. Fix the pipeline (kLaneCameraSource) or recalibrate\n";
    }

    rawBuffer_->publish(std::move(raw));
    const FrameInfo info = laneFrameBuffer_->publish(std::move(output));

    // Chỉ gửi thông báo, pixel nằm trong laneFrameBuffer_ (ThreadUdpServer gửi ảnh về PC)
    laneFrameSender_.send(info);
    log_frame(lane_, info);
}

void ThreadCameraHandler::process_road(std::shared_ptr<cv::Mat> raw) {
    const FrameInfo info = frameBuffer_->publish(std::move(raw));

    // Chỉ gửi thông báo, pixel nằm trong frameBuffer_
    frameSender_.send(info);
    log_frame(road_, info);
}

void ThreadCameraHandler::log_frame(CameraSource& cam, const FrameInfo& info) {
    if (!cam.firstFrameLogged) {
        cam.firstFrameLogged = true;
        logger_ << "[CameraHandler] " << cam.name << " first frame: " << info.width << "x" << info.height << '\n';
    }
    if (debugging_ && info.seq % utils::logcfg::kCameraLogEveryFrames == 0) {
        logger_ << "[CameraHandler] " << cam.name << " seq=" << info.seq << " size=" << info.width << "x"
                << info.height << '\n';
    }
}

// Mở lại đúng source đã truyền vào constructor.
// nextAttempt được đặt trước, nên mọi nhánh lỗi đều chờ đủ kReconnectInterval.
void ThreadCameraHandler::try_open(CameraSource& cam) {
    // Nguồn rỗng = camera bị tắt trong main.cpp (sensor-id -1): không bao giờ thử mở
    if (cam.source.empty()) {
        if (cam.openFailures == 0) logger_ << "[CameraHandler] " << cam.name << " disabled (sensor id < 0)\n";
        cam.openFailures = 1;
        cam.nextAttempt  = Clock::time_point::max();
        return;
    }

    cam.nextAttempt = Clock::now() + (cam.openFailures >= kSlowRetryAfter ? kSlowReconnectInterval : kReconnectInterval);

    cam.capture.release();
    if (!cam.capture.open(cam.source, apiPreference_) || !cam.capture.isOpened()) {
        ++cam.openFailures;
        if (cam.errorLimiter.allow()) {
            logger_ << "[CameraHandler] ERROR: Cannot open " << cam.name << " source '" << cam.source
                    << "'. Check on the Jetson: ls /dev/video* (no device = cable/port problem), "
                       "gst-launch-1.0 nvarguscamerasrc sensor-id=N num-buffers=30 ! fakesink, OpenCV built with "
                       "GStreamer, no other program using the camera (sudo systemctl restart nvargus-daemon)\n";
        }
        return;
    }

    cam.openFailures     = 0;
    cam.failures         = 0;
    cam.firstFrameLogged = false;
    cam.opened           = true;
    logger_ << "[CameraHandler] " << cam.name << " opened\n";
}

// Đóng camera, bỏ ảnh cũ để consumer không dùng ảnh đã cũ, chờ thử mở lại.
void ThreadCameraHandler::handle_lost(CameraSource& cam, std::initializer_list<FrameBuffer*> buffers) {
    logger_ << "[CameraHandler] " << cam.name << " lost, will retry\n";
    cam.opened = false;
    cam.capture.release();
    cam.failures = 0;
    for (auto* buffer : buffers) buffer->clear();
    cam.nextAttempt = Clock::now() + kReconnectInterval;
}

} // namespace hardware::camera
