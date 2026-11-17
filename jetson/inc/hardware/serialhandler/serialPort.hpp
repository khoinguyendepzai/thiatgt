#pragma once

#include <atomic>
#include <mutex>
#include <shared_mutex>
#include <string>

namespace hardware::serialhandler {

enum class ReadStatus { Data, NoData, Closed, Error };

// Tham số cổng serial của Nucleo (main.cpp: kSerialParams). Khung truyền cố định 8N1, không flow control.
struct SerialParams {
    std::string device;                // ví dụ "/dev/ttyACM0"
    unsigned    baudrate = 115200;     // 9600, 19200, 38400, 57600, 115200, 230400, 460800 hoặc 921600; số khác thì open() thất bại
};

// Cổng serial 8N1, không flow control, tốc độ theo SerialParams. Không tự kết nối lại (ThreadSerialHandler lo).
// Khóa: fdMutex_ (shared khi đọc/ghi, unique khi open/close), writeMutex_ để hai lệnh ghi không chen byte.
// Thứ tự lấy khóa: fdMutex_ rồi writeMutex_.
class SerialPort {
public:
    // params.device: đường dẫn cố định, ví dụ "/dev/ttyACM0". Constructor chưa mở cổng.
    explicit SerialPort(const SerialParams& params);
    ~SerialPort();

    SerialPort(const SerialPort&)            = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    bool open();
    void close();
    bool is_open() const;

    // Ghi hết chuỗi; false nếu lỗi (đồng thời đánh dấu mất kết nối)
    bool write_all(const std::string& data);

    // Đọc những gì đang có (không chờ), NỐI vào out.
    // NoData nếu chưa có gì; Closed nếu cổng chưa mở; Error nếu lỗi I/O (đồng thời đánh dấu mất kết nối).
    ReadStatus read_some(std::string& out);

    // Thread đọc/ghi gặp lỗi I/O thì bật cờ này; ThreadSerialHandler sẽ xử lý
    bool connection_lost() const { return lost_.load(); }

    // device_ là const nên đọc từ nhiều thread không cần khóa
    const std::string& device() const { return params_.device; }
    unsigned baudrate() const { return params_.baudrate; }

private:
    const SerialParams     params_;
    int                    fd_ = -1;
    mutable std::shared_mutex fdMutex_;
    std::mutex             writeMutex_;
    std::atomic<bool>      lost_{false};
};

} // namespace hardware::serialhandler
