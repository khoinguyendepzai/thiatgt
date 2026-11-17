#pragma once

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace templates {

// Lớp cơ sở cho thread có start/stop/pause/resume: lớp con override thread_work() (gọi lặp, nghỉ pause giây mỗi vòng).
// Lớp con PHẢI gọi stop() + join() trong destructor của chính nó, vì member của lớp con bị hủy trước lớp base.
class ThreadWithStop {
public:
    // pause: thời gian nghỉ giữa mỗi vòng thread_work (tính bằng giây)
    explicit ThreadWithStop(double pause = 0.001);

    ThreadWithStop(const ThreadWithStop&)            = delete;
    ThreadWithStop& operator=(const ThreadWithStop&) = delete;

    virtual ~ThreadWithStop();

    // Bắt đầu thread (gọi lại sau stop() thì chạy lại từ đầu)
    void start();

    // Yêu cầu thread kết thúc (không chờ)
    void stop();

    // Tạm dừng thread (thread vẫn sống, chỉ không chạy thread_work).
    // Không có tác dụng nếu đã stop().
    void pause();

    // Tiếp tục chạy sau khi pause. Không có tác dụng nếu đã stop().
    void resume();

    // Chờ thread kết thúc
    void join();

    // Chờ tối đa timeout giây, trả về true nếu thread đã kết thúc
    bool join_for(std::chrono::seconds timeout);

    bool is_paused() const;
    bool is_alive() const;

protected:
    // Override để thực hiện công việc chính của thread
    virtual void thread_work() {}

    // Override nếu cần xử lý state change (được gọi ngay trước thread_work)
    virtual void state_change_handler() {}

private:
    enum class State { Running, Paused, Stopped };

    void run();

    const std::chrono::milliseconds pause_;

    mutable std::mutex      mutex_;     // bảo vệ state_ và running_
    std::condition_variable cv_;        // đánh thức: pause/resume/stop và thread kết thúc
    State                   state_{State::Running};
    bool                    running_{false};   // true từ start() đến khi run() thoát

    std::thread worker_;
};

} // namespace templates
