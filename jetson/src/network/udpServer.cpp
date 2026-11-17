#include <network/udpServer.hpp>

#include <cerrno>

#include <sys/socket.h>
#include <unistd.h>

namespace network {

UdpServer::~UdpServer() { close(); }

bool UdpServer::open() {
    close();

    const int fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (fd < 0) return false;

    // Tăng buffer gửi để cả frame JPEG (nhiều gói liền nhau) lọt vào, không rớt gói giữa chừng
    const int sndbuf = 1 << 20;
    ::setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

    // Cho phép gửi tới địa chỉ broadcast (255.255.255.255 hoặc dạng 192.168.1.255); thiếu thì sendto báo EACCES
    const int enable = 1;
    ::setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &enable, sizeof(enable));

    // Không bind: hệ điều hành tự cấp cổng nguồn ở lần sendto đầu tiên (máy nhận chỉ cần quan tâm cổng đích)
    fd_ = fd;
    return true;
}

void UdpServer::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool UdpServer::send_to(const sockaddr_in& to, const void* data, std::size_t size) {
    if (fd_ < 0) return false;

    while (true) {
        const ssize_t n = ::sendto(fd_, data, size, MSG_NOSIGNAL,
                                   reinterpret_cast<const sockaddr*>(&to), sizeof(to));
        if (n >= 0) return static_cast<std::size_t>(n) == size;
        if (errno == EINTR) continue;
        return false;              // EAGAIN (buffer đầy) hoặc lỗi mạng -> bỏ gói, UDP không đảm bảo
    }
}

} // namespace network
