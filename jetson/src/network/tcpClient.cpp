#include <network/tcpClient.hpp>

#include <cerrno>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace network {

TcpClient::~TcpClient() { close(); }

bool TcpClient::connect(const std::string& host, std::uint16_t port,
                        std::chrono::milliseconds timeout) {
    close();

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(port);
    if (port == 0 || inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
        errno = EINVAL;
        return false;
    }

    const int fd = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (fd < 0) return false;

    // Hàm trợ giúp: đóng fd nhưng giữ errno cho caller
    auto fail = [fd](int err) {
        ::close(fd);
        errno = err;
        return false;
    };

    if (::connect(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0) {
        if (errno != EINPROGRESS) return fail(errno);

        // Kết nối không chặn: chờ socket ghi được (= xong hoặc lỗi) tối đa timeout
        pollfd p{fd, POLLOUT, 0};
        int r;
        do {
            r = ::poll(&p, 1, static_cast<int>(timeout.count()));
        } while (r < 0 && errno == EINTR);
        if (r < 0)  return fail(errno);
        if (r == 0) return fail(ETIMEDOUT);

        int       soError = 0;
        socklen_t len     = sizeof(soError);
        if (::getsockopt(fd, SOL_SOCKET, SO_ERROR, &soError, &len) != 0) return fail(errno);
        if (soError != 0) return fail(soError);
    }

    // Keepalive: sau 5s im lặng, thăm dò mỗi 2s, 3 lần không đáp -> mất kết nối
    const int on = 1, idle = 5, interval = 2, count = 3;
    ::setsockopt(fd, SOL_SOCKET,  SO_KEEPALIVE,  &on,       sizeof(on));
    ::setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE,  &idle,     sizeof(idle));
    ::setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &interval, sizeof(interval));
    ::setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT,   &count,    sizeof(count));

    fd_ = fd;
    return true;
}

void TcpClient::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

TcpReadStatus TcpClient::read_some(std::string& out) {
    if (fd_ < 0) return TcpReadStatus::Disconnected;

    char          buf[1024];
    const ssize_t n = ::recv(fd_, buf, sizeof(buf), 0);
    if (n > 0) {
        out.assign(buf, static_cast<std::size_t>(n));
        return TcpReadStatus::Data;
    }
    if (n == 0) return TcpReadStatus::Disconnected;     // server đóng kết nối
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return TcpReadStatus::NoData;
    return TcpReadStatus::Disconnected;                 // ECONNRESET, ETIMEDOUT (keepalive)...
}

} // namespace network
