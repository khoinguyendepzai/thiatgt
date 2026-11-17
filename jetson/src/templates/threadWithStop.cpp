#include <templates/threadWithStop.hpp>

#include <cmath>
#include <exception>
#include <iostream>

namespace templates {

namespace {
std::chrono::milliseconds to_milliseconds(double seconds) {
    if (seconds < 0.0) seconds = 0.0;
    return std::chrono::milliseconds(std::llround(seconds * 1000.0));
}
} // namespace

ThreadWithStop::ThreadWithStop(double pause)
    : pause_(to_milliseconds(pause))
{}

ThreadWithStop::~ThreadWithStop() {
    stop();
    join();
}

void ThreadWithStop::start() {
    if (is_alive()) return;
    join();                                   // dọn thread cũ đã kết thúc (nếu có)
    {
        std::lock_guard<std::mutex> l(mutex_);
        state_   = State::Running;
        running_ = true;
    }
    worker_ = std::thread(&ThreadWithStop::run, this);
}

void ThreadWithStop::run() {
    std::unique_lock<std::mutex> l(mutex_);

    while (state_ != State::Stopped) {
        // Đang Paused thì ngủ ở đây cho đến khi resume() hoặc stop()
        cv_.wait(l, [this] { return state_ != State::Paused; });
        if (state_ == State::Stopped) break;

        // KHÔNG giữ mutex_ khi làm việc: thread_work() có thể gọi pause()/stop()
        l.unlock();
        try {
            state_change_handler();
            thread_work();
        } catch (const std::exception& e) {
            std::cerr << "ThreadWithStop error: " << e.what() << '\n';
        } catch (...) {
            std::cerr << "ThreadWithStop: unknown exception\n";
        }
        l.lock();

        // Nghỉ pause_ giữa các vòng; bị đánh thức sớm nếu pause()/stop()
        if (state_ == State::Running) {
            cv_.wait_for(l, pause_, [this] { return state_ != State::Running; });
        }
    }

    running_ = false;
    l.unlock();
    cv_.notify_all();                         // đánh thức join_for() đang chờ
}

void ThreadWithStop::pause() {
    std::lock_guard<std::mutex> l(mutex_);
    if (state_ == State::Running) state_ = State::Paused;
    // Không cần notify: thread tự kiểm tra state_ ở vòng kế tiếp
}

void ThreadWithStop::resume() {
    {
        std::lock_guard<std::mutex> l(mutex_);
        if (state_ != State::Paused) return;
        state_ = State::Running;
    }
    cv_.notify_all();
}

void ThreadWithStop::stop() {
    {
        std::lock_guard<std::mutex> l(mutex_);
        state_ = State::Stopped;
    }
    cv_.notify_all();
}

void ThreadWithStop::join() {
    if (worker_.joinable()) worker_.join();
}

bool ThreadWithStop::join_for(std::chrono::seconds timeout) {
    {
        std::unique_lock<std::mutex> l(mutex_);
        if (!cv_.wait_for(l, timeout, [this] { return !running_; })) return false;
    }
    join();
    return true;
}

bool ThreadWithStop::is_paused() const {
    std::lock_guard<std::mutex> l(mutex_);
    return state_ == State::Paused;
}

bool ThreadWithStop::is_alive() const {
    std::lock_guard<std::mutex> l(mutex_);
    return worker_.joinable() && running_;
}

} // namespace templates
