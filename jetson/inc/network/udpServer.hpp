#pragma once

#include <cstddef>
#include <cstdint>

#include <netinet/in.h>

namespace network {

// Socket UDP IPv4 không chặn, KHÔNG bind (cổng nguồn do hệ điều hành cấp), chỉ gửi. KHÔNG thread-safe.
class UdpServer {
public:
    UdpServer() = default;
    ~UdpServer();

    UdpServer(const UdpServer&)            = delete;
    UdpServer& operator=(const UdpServer&) = delete;

    // Mở socket. false nếu lỗi (hết file descriptor...). Gọi lại khi đang mở thì đóng socket cũ trước.
    bool open();

    void close();
    bool is_open() const { return fd_ >= 0; }

    // Gửi một datagram (không chờ). false nếu lỗi hoặc buffer gửi đang đầy (gói bị bỏ).
    bool send_to(const sockaddr_in& to, const void* data, std::size_t size);

private:
    int fd_ = -1;
};

} // namespace network
