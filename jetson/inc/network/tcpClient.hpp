#pragma once

#include <chrono>
#include <cstdint>
#include <string>

namespace network {

enum class TcpReadStatus {
    Data,           // đã đọc được dữ liệu
    NoData,         // chưa có gì (không chặn)
    Disconnected    // server đóng kết nối hoặc lỗi mạng -> caller nên close()
};

// Kết nối TCP client IPv4, chỉ nhận dữ liệu, đọc không chặn. Caller tự cắt message (xem ThreadTcpClient).
// KHÔNG thread-safe.
class TcpClient {
public:
    TcpClient() = default;
    ~TcpClient();

    TcpClient(const TcpClient&)            = delete;
    TcpClient& operator=(const TcpClient&) = delete;

    // Kết nối tới host:port (IPv4 dạng số), chờ tối đa timeout; đang mở thì đóng kết nối cũ trước.
    // Lỗi -> false, errno giữ nguyên nhân. Bật keepalive để phát hiện server mất sau ~11s.
    bool connect(const std::string& host, std::uint16_t port, std::chrono::milliseconds timeout);

    void close();
    bool is_open() const { return fd_ >= 0; }

    // Đọc những gì đang có (tối đa 1024 byte, không chờ). Dữ liệu ghi ĐÈ vào out.
    TcpReadStatus read_some(std::string& out);

private:
    int fd_ = -1;
};

} // namespace network
