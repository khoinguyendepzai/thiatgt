#pragma once

#include <cstdint>
#include <memory>
#include <mutex>

#include <opencv2/core.hpp>

#include <utils/messages/allMessages.hpp>

namespace hardware::camera {

// Một frame kèm FrameInfo. image == nullptr: chưa có frame (hoặc camera vừa mất).
struct Frame {
    std::shared_ptr<const cv::Mat> image;
    utils::messages::FrameInfo     info;
};

// Ô "frame mới nhất" dùng chung giữa thread ghi và các consumer. Thread-safe.
// Mỗi frame là một cv::Mat riêng (shared_ptr<const>) nên frame sau không làm hỏng ảnh consumer đang giữ.
// Ảnh chỉ để đọc; muốn sửa thì clone() trước.
class FrameBuffer {
public:
    FrameBuffer() = default;

    FrameBuffer(const FrameBuffer&)            = delete;
    FrameBuffer& operator=(const FrameBuffer&) = delete;

    // Đặt frame mới, trả về FrameInfo để gửi thông báo. image rỗng -> không làm gì, trả seq = 0.
    utils::messages::FrameInfo publish(std::shared_ptr<const cv::Mat> image);

    // Lấy frame mới nhất (copy shared_ptr, không copy pixel).
    Frame latest() const;

    // Bỏ ảnh hiện tại để consumer không dùng ảnh cũ; seq không lùi
    void clear();

private:
    mutable std::mutex mutex_;
    Frame              frame_;
    std::uint64_t      seq_ = 0;
};

} // namespace hardware::camera
