// cam_udp_sender.cpp - chạy trên Jetson: mở camera CSI bằng GStreamer, nén JPEG, gửi UDP về PC (main.py, cổng 7000).
//
// Giữ cách làm của BrainUp (main.cpp, udpServer.cpp, threadUdpServer.cpp):
//   - pipeline kLaneCameraSource (1920x1080 -> 640x480, flip-method=2) như main.cpp;
//   - IP và cổng khai báo bằng hằng số đầu file như kUdpClientIp / kUdpClientPort / kUdpPort của main.cpp:
//     socket bind cổng nguồn kUdpPort (9000), gửi tới kUdpClientIp:kUdpClientPort (192.168.1.104:7000);
//   - socket UDP không chặn, SO_SNDBUF 1 MB, không SO_REUSEADDR (chạy hai bản cùng lúc thì báo "port đang dùng"),
//     sendto với MSG_NOSIGNAL, buffer đầy thì bỏ gói; JPEG chất lượng 85 (kJpegQuality của BrainUp).
// Mỗi gói UDP là MỘT ảnh JPEG nguyên vẹn (< 65 507 byte). Gửi ảnh THÔ chưa khử méo: PC tự khử méo bằng K và hệ số
// méo (main.py, GroundCalibration), nên calib dưới đây chỉ để đối chiếu, code gửi không dùng.
//
// Build trên Jetson:
//   g++ -O2 -std=c++17 cam_udp_sender.cpp -o cam_udp_sender $(pkg-config --cflags --libs opencv4)
// Chạy (tắt braincar trước vì camera chỉ mở được một lần):
//   ./cam_udp_sender ["<pipeline đường>"]
// Đổi IP PC / cổng thì sửa hằng số bên dưới, giống main.cpp của BrainUp. Ctrl+C để thoát.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

// Camera bird's-eye (cùng pipeline với kLaneCameraSource trong main.cpp của BrainUp). K và hệ số méo gắn với độ
// phân giải, sensor mode và flip-method của pipeline này; đổi một bên thì phải calib lại.
static const std::string kLaneCameraSource =
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM), width=1920, height=1080, format=NV12, framerate=30/1 ! "
    "nvvidconv flip-method=2 ! "
    "video/x-raw, width=640, height=480, format=BGRx ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! "
    "appsink drop=true max-buffers=1 sync=false";

// Calib 08/11/2026 ở 640x480: chép từ kGroundCalibration trong main.cpp của BrainUp, trùng GroundCalibration của
// main.py. Jetson không dùng (PC khử méo và tính homography).
struct GroundCalibration {
    cv::Size imageSize;
    cv::Matx33d homography;   // ảnh đã chỉnh méo -> mặt đường (m)
    cv::Matx33d cameraMatrix;
    cv::Vec<double, 5> distCoeffs;
};
static const GroundCalibration kGroundCalibration{
    cv::Size(640, 480),
    cv::Matx33d( 1.38382102e-05,  0.000345778475,  1.08880963,
                -0.00153570693,   3.85623778e-05,  0.442112946,
                 2.11989362e-05,  0.00410399309,   1),
    cv::Matx33d(512.492640, 0.0, 287.186733,
                0.0, 687.516488, 284.552142,
                0.0, 0.0, 1.0),
    cv::Vec<double, 5>(-0.35453198, 0.18076870, -0.00151746, 0.00140839, -0.06370173),
};

// Cổng nguồn trên Jetson (kUdpPort của main.cpp)
static constexpr std::uint16_t kUdpPort = 9000;

// Client nhận stream UDP: main.py trên PC (kUdpClientIp / kUdpClientPort của main.cpp)
static const std::string       kUdpClientIp   = "192.168.1.100";
static constexpr std::uint16_t kUdpClientPort = 7000;

static constexpr int kJpegQuality = 85;                 // như threadUdpServer.cpp
static constexpr std::size_t kMaxUdpPayload = 65507;    // giới hạn một gói UDP/IPv4

static volatile std::sig_atomic_t g_run = 1;
static void onSignal(int) { g_run = 0; }

int main(int argc, char** argv) {
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    const std::string laneSource = (argc >= 2) ? argv[1] : kLaneCameraSource;

    cv::VideoCapture cap(laneSource, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        std::cerr << "Khong mo duoc camera. Da tat braincar chua? OpenCV co GStreamer khong?\n";
        return 1;
    }

    // Socket giống UdpServer::bind: không chặn, buffer gửi 1 MB, bind cổng nguồn, không SO_REUSEADDR
    const int fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (fd < 0) {
        std::cerr << "Khong tao duoc socket UDP\n";
        return 1;
    }
    const int sndbuf = 1 << 20;
    ::setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = htons(kUdpPort);
    if (::bind(fd, reinterpret_cast<const sockaddr*>(&local), sizeof(local)) != 0) {
        std::cerr << "Khong bind duoc cong UDP " << kUdpPort << " (port dang dung?)\n";
        ::close(fd);
        return 1;
    }

    // Địa chỉ client giống add_udp_client: IPv4 dạng số
    sockaddr_in client{};
    client.sin_family = AF_INET;
    client.sin_port = htons(kUdpClientPort);
    if (::inet_pton(AF_INET, kUdpClientIp.c_str(), &client.sin_addr) != 1) {
        std::cerr << "UDP client khong hop le: " << kUdpClientIp << ':' << kUdpClientPort << "\n";
        ::close(fd);
        return 1;
    }

    std::cout << "UDP stream: cong nguon " << kUdpPort << " -> " << kUdpClientIp << ':' << kUdpClientPort
              << " (camera tho " << kGroundCalibration.imageSize.width << 'x' << kGroundCalibration.imageSize.height
              << ", JPEG q" << kJpegQuality << "). Ctrl+C de thoat\n";

    const std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, kJpegQuality};
    std::vector<uchar> jpeg;
    cv::Mat frame;
    long sent = 0, dropped = 0;
    auto tReport = std::chrono::steady_clock::now();

    while (g_run) {
        if (!cap.read(frame) || frame.empty()) {
            usleep(10000);
            continue;
        }
        if (frame.size() != kGroundCalibration.imageSize) cv::resize(frame, frame, kGroundCalibration.imageSize);

        if (!cv::imencode(".jpg", frame, jpeg, params) || jpeg.size() > kMaxUdpPayload) {
            ++dropped;  // ảnh quá lớn cho một gói: bỏ frame, nên hạ chất lượng
            continue;
        }

        ssize_t n;
        do {
            n = ::sendto(fd, jpeg.data(), jpeg.size(), MSG_NOSIGNAL, reinterpret_cast<const sockaddr*>(&client),
                         sizeof(client));
        } while (n < 0 && errno == EINTR);
        if (n == static_cast<ssize_t>(jpeg.size())) ++sent;
        else ++dropped;  // EAGAIN (buffer đầy) hoặc lỗi mạng -> bỏ gói, UDP không đảm bảo

        const auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now - tReport).count() >= 5.0) {
            std::cout << "Da gui " << sent << " anh, bo " << dropped << ", goi cuoi " << jpeg.size() << " byte\n";
            tReport = now;
        }
    }

    cap.release();
    ::close(fd);
    std::cout << "Dung. Tong gui " << sent << " anh, bo " << dropped << "\n";
    return 0;
}
