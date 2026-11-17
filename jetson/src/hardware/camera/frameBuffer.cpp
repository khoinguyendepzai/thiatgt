#include <hardware/camera/frameBuffer.hpp>

#include <chrono>
#include <utility>

namespace hardware::camera {

using utils::messages::FrameInfo;

namespace {

// Giây tính từ epoch
double now_seconds() {
    using namespace std::chrono;
    return duration<double>(system_clock::now().time_since_epoch()).count();
}

} // namespace

FrameInfo FrameBuffer::publish(std::shared_ptr<const cv::Mat> image) {
    if (!image || image->empty()) return {};

    const double timestamp = now_seconds();

    std::lock_guard<std::mutex> lock(mutex_);

    FrameInfo info;
    info.seq       = ++seq_;
    info.timestamp = timestamp;
    info.width     = image->cols;
    info.height    = image->rows;

    frame_.info  = info;
    frame_.image = std::move(image);
    return info;
}

Frame FrameBuffer::latest() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return frame_;
}

void FrameBuffer::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    frame_.image.reset();
}

} // namespace hardware::camera
