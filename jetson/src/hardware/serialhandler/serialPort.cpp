#include <hardware/serialhandler/serialPort.hpp>

#include <cerrno>
#include <cstddef>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace hardware::serialhandler {

namespace {

// Tốc độ baud -> hằng termios; false nếu không hỗ trợ
bool baud_to_speed(unsigned baudrate, speed_t& speed) {
    switch (baudrate) {
        case 9600:   speed = B9600;   return true;
        case 19200:  speed = B19200;  return true;
        case 38400:  speed = B38400;  return true;
        case 57600:  speed = B57600;  return true;
        case 115200: speed = B115200; return true;
        case 230400: speed = B230400; return true;
        case 460800: speed = B460800; return true;
        case 921600: speed = B921600; return true;
        default:     return false;
    }
}

} // namespace

SerialPort::SerialPort(const SerialParams& params) : params_(params) {}

SerialPort::~SerialPort() { close(); }

bool SerialPort::open() {
    std::unique_lock<std::shared_mutex> lock(fdMutex_);
    if (fd_ >= 0) return true;

    speed_t speed;
    if (!baud_to_speed(params_.baudrate, speed)) return false;

    const int fd = ::open(params_.device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return false;

    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        ::close(fd);
        return false;
    }

    cfmakeraw(&tty);                       // 8 bit, không xử lý ký tự đặc biệt
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(CSTOPB | CRTSCTS);    // 1 stop bit, không flow control
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        ::close(fd);
        return false;
    }

    tcflush(fd, TCIOFLUSH);                // xóa buffer vào/ra

    fd_ = fd;
    lost_.store(false);
    return true;
}

// Đợi read_some()/write_all() đang chạy xong (write_all() kẹt tối đa ~200ms) rồi mới đóng
void SerialPort::close() {
    std::unique_lock<std::shared_mutex> lock(fdMutex_);
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    lost_.store(false);
}

bool SerialPort::is_open() const {
    std::shared_lock<std::shared_mutex> lock(fdMutex_);
    return fd_ >= 0;
}

bool SerialPort::write_all(const std::string& data) {
    std::shared_lock<std::shared_mutex> fdLock(fdMutex_);   // giữ fd_ không bị đóng giữa chừng
    if (fd_ < 0) return false;

    std::lock_guard<std::mutex> writeLock(writeMutex_);     // không để hai lệnh chen byte nhau

    std::size_t offset = 0;
    int stalls = 0;

    while (offset < data.size()) {
        const ssize_t n = ::write(fd_, data.data() + offset, data.size() - offset);
        if (n >= 0) {
            offset += static_cast<std::size_t>(n);
            stalls = 0;
            continue;
        }
        if (errno == EINTR) continue;
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            pollfd p{fd_, POLLOUT, 0};
            ::poll(&p, 1, 10);
            if (++stalls > 20) {           // ~200ms không ghi được
                lost_.store(true);
                return false;
            }
            continue;
        }
        lost_.store(true);
        return false;
    }
    return true;
}

// Chỉ giữ khóa shared: write_all() vẫn chạy song song. poll() timeout 0 nên không chặn;
// ThreadRead tự nghỉ giữa các vòng.
ReadStatus SerialPort::read_some(std::string& out) {
    std::shared_lock<std::shared_mutex> lock(fdMutex_);
    if (fd_ < 0) return ReadStatus::Closed;

    pollfd p{fd_, POLLIN, 0};
    const int r = ::poll(&p, 1, 0);

    if (r < 0) {
        if (errno == EINTR) return ReadStatus::NoData;
        lost_.store(true);
        return ReadStatus::Error;
    }
    if (r == 0) return ReadStatus::NoData;

    if (p.revents & POLLIN) {
        char buf[256];
        const ssize_t n = ::read(fd_, buf, sizeof(buf));
        if (n > 0) {
            out.append(buf, static_cast<std::size_t>(n));
            return ReadStatus::Data;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
            return ReadStatus::NoData;
        }
        lost_.store(true);                 // n == 0 hoặc lỗi khác: rút cáp
        return ReadStatus::Error;
    }

    if (p.revents & (POLLERR | POLLHUP | POLLNVAL)) {
        lost_.store(true);
        return ReadStatus::Error;
    }
    return ReadStatus::NoData;
}

} // namespace hardware::serialhandler
