#pragma once

#include <condition_variable>
#include <mutex>
#include <queue>
#include <utility>

namespace utils {

template <typename T>
class BlockingQueue {
public:
    BlockingQueue() = default;
    ~BlockingQueue() = default;

    BlockingQueue(const BlockingQueue&) = delete;
    BlockingQueue& operator=(const BlockingQueue&) = delete;
    // Thêm một phần tử vào cuối queue bằng cách copy.
    void put(const T& value) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.push(value);
        }
        cv_.notify_one();
    }
    // Thêm một phần tử vào cuối queue bằng cách move.
    void put(T&& value) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.push(std::move(value));
        }
        cv_.notify_one();
    }
    // Lấy phần tử đầu queue; chờ (chặn) nếu queue đang rỗng.
    T get() {
        std::unique_lock<std::mutex> lock(mutex_);

        cv_.wait(lock, [this] {
            return !queue_.empty();
        });

        T value = std::move(queue_.front());
        queue_.pop();

        return value;
    }
    // Thử lấy phần tử đầu queue mà không chờ; trả về false nếu queue rỗng.
    bool try_get(T& value) {
        std::lock_guard<std::mutex> lock(mutex_);

        if (queue_.empty()) {
            return false;
        }

        value = std::move(queue_.front());
        queue_.pop();

        return true;
    }
    // Kiểm tra queue có phần tử hay không.
    bool poll() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return !queue_.empty();
    }
    // Kiểm tra queue có rỗng hay không.
    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }
    // Xóa toàn bộ phần tử đang có trong queue.
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);

        while (!queue_.empty()) {
            queue_.pop();
        }
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::queue<T> queue_;
};

} // namespace utils
