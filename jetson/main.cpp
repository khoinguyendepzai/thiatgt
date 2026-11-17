#include <any>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <termios.h>
#include <unistd.h>

#include <control/threadControlHandler.hpp>
#include <gateway/threadGateWay.hpp>
#include <hardware/camera/frameBuffer.hpp>
#include <hardware/camera/threadCameraHandler.hpp>
#include <hardware/serialhandler/threadSerialHandler.hpp>
#include <network/threadNetworkHandler.hpp>
#include <perception/cameraUndistorter.hpp>
#include <statemachine/stateMachine.hpp>
#include <statemachine/systemMode.hpp>
#include <utils/messages/allMessages.hpp>
#include <utils/messages/messageHandlerSubscriber.hpp>
#include <utils/logConfig.hpp>
#include <utils/queues.hpp>

using namespace std::chrono_literals;
using utils::messages::FrameInfo;
using utils::messages::MessageHandlerSubscriber;
using utils::messages::MessageName;

// Cờ dừng chương trình dùng chung (bấm q, ESC hoặc Ctrl+C)
static std::atomic<bool> g_shutdownRequested{false};

static void shutdown_signal_handler(int) {
    g_shutdownRequested.store(true);
}

// Chương trình chính: 2 camera (nhìn đường + bám làn có nắn méo) + UDP + TCP + serial + state machine. Không còn dò làn
// hay tự hành: ảnh cam bám làn đã nắn méo gửi về PC qua UDP, main.py trên PC dò làn và cảnh báo lệch làn.
// ThreadControlHandler chỉ subscribe StateChange (phần điều khiển code sau).
// Phím (terminal hoặc cửa sổ OpenCV):
//   q / ESC: chuyển về STOP rồi thoát (Ctrl+C cũng vậy)
//   s: lưu ảnh THÔ (chưa nắn méo) của camera bám làn ra test.jpg (để calib bằng calib/pick_marks.py) và ảnh camera
//      nhìn đường ra road.jpg
//   1..4: đổi state (1 DEFAULT, 2 AUTO, 3 MANUAL, 4 STOP)
// Chạy: ./braincar [<ip nhận UDP | any | 0.0.0.0> [<ip server TCP>]]
//   tham số 1 ghi đè kUdpClientIp: IP một máy nhận, hoặc "any" hay "0.0.0.0" = gửi tới 255.255.255.255 (broadcast), máy
//   nào trong LAN đang nghe cổng thì nhận được. Tham số 2 ghi đè kTcpServerHost (bỏ trống thì dùng giá trị đặt cứng).
//   Pipeline hai camera (kRoadCameraSource, kLaneCameraSource) và cổng serial (kSerialParams) chỉ đổi được trong code.
// Cửa sổ chỉ mở khi DISPLAY khác rỗng.

// Pipeline GStreamer hai camera CSI trên Raspberry Pi 5 (libcamerasrc).
// Không có camera-name thì libcamerasrc mở camera đầu tiên libcamera liệt kê (camera 0), nên cam bám làn để trống như
// Brain, còn cam người lái chỉ rõ camera kia bằng camera-name. Xem tên và thứ tự: rpicam-hello --list-cameras
// (dòng "1 : imx219 ... (/base/axi/...)"). Nếu cửa sổ Camera ra ảnh bird's-eye: đổi đường dẫn dưới đây sang camera 0
// và thêm camera-name của camera 1 vào kLaneCameraSource (và LANE_CAMERA_SOURCE trong calib/calib_camera.py).
static const std::string kRoadCameraName = "/base/axi/pcie@120000/rp1/i2c@80000/imx219@10";

// Cam người lái (ThreadCameraHandler đọc): ảnh cho UDP và cửa sổ Camera, 960x540 (16:9, libcamera chọn sensor mode 1920x1080
// và ISP thu nhỏ bằng phần cứng). Không cần videoflip: libcamera tự lật theo "rotation" của device tree.
static const std::string kRoadCameraSource =
    "libcamerasrc camera-name=\"" + kRoadCameraName + "\" ! "
    "video/x-raw, width=960, height=540, format=NV12, framerate=30/1 ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! "
    "appsink drop=true max-buffers=1 sync=false";

// Cam bám làn (ThreadCameraHandler đọc và nắn méo, gửi về PC). PHẢI cho cùng hình học ảnh với lúc
// calib kCameraCalibration, tức pipeline cũ trên Jetson: sensor mode 1920x1080 (vùng cắt 16:9 giữa cảm biến IMX219),
// xoay 180°, kéo giãn 16:9 -> 640x480 BGR (vì vậy fx ≠ fy trong cameraMatrix). Ở đây:
//   - xin 960x540 (16:9): libcamera chọn sensor mode 1920x1080 (cùng vùng cắt), ISP thu nhỏ bằng phần cứng.
//     Xin thẳng 640x480 (4:3) thì libcamera chọn mode 640x480 = vùng cắt 1280x960 ở giữa, hẹp hơn hẳn: sai calib.
//   - không cần videoflip: libcamera tự lật cảm biến theo "rotation" của device tree (Jetson phải flip-method=2).
//   - videoscale add-borders=false: kéo giãn 960x540 -> 640x480 như nvvidconv, không thêm viền đen.
// Camera khác IMX219 hoặc đổi chỗ gắn: calib lại (README mục 2). Giữ giống LANE_CAMERA_SOURCE trong
// calib/calib_camera.py.
static const std::string kLaneCameraSource =
    "libcamerasrc ! "
    "video/x-raw, width=960, height=540, format=NV12, framerate=30/1 ! "
    "videoscale add-borders=false ! "
    "video/x-raw, width=640, height=480 ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! "
    "appsink drop=true max-buffers=1 sync=false";

// Calib ống kính cam bám làn để nắn méo trên xe. Bắt buộc có đủ số.
// Homography (ảnh đã nắn méo -> mặt đường) nằm ở PC (GroundCalibration trong main.py), phải khớp đúng bộ K này.
static const perception::CameraCalibration kCameraCalibration{
    cv::Size(640, 480),
    // Calib ống kính (calib/calib_camera.py) 08/11/2026 trực tiếp ở 640x480 (45 ảnh bàn cờ, RMS 0.41 px)
    cv::Matx33d(512.492640, 0.0, 287.186733,    // fx, 0, cx
                0.0, 687.516488, 284.552142,    // 0, fy, cy
                0.0, 0.0, 1.0),
    cv::Vec<double, 5>(-0.35453198, 0.18076870, -0.00151746, 0.00140839, -0.06370173), // k1, k2, p1, p2, k3
};

// Client nhận UDP: mỗi luồng một cổng đích riêng. Xe không bind cổng nguồn (hệ điều hành tự cấp).
// kUdpClientIp và kTcpServerHost là mặc định, tham số dòng lệnh 1 và 2 ghi đè.
// Cổng phải khớp LANE_PORT / ROAD_PORT trong main.py.
static const std::string       kUdpClientIp   = "0.0.0.0";
static constexpr std::uint16_t kUdpLanePort   = 7000;   // ảnh camera bám làn đã nắn méo (main.py dò làn)
static constexpr std::uint16_t kUdpClientPort = 7001;   // ảnh camera nhìn đường
static constexpr std::uint16_t kUdpTextPort   = 9000;   // text telemetry (@speed, @ref, @steer, @imu)

// Server TCP gửi "@vcb:...", "@state:...", "@mode:...", "@distance:..." (IPv4 dạng số)
static const std::string       kTcpServerHost = "192.168.1.10";
static constexpr std::uint16_t kTcpServerPort = 6000;

// Cổng serial của Nucleo
static const hardware::serialhandler::SerialParams kSerialParams{
    "/dev/ttyACM0",   // device
    115200,           // baudrate
};

// Đo fps ghi vào một FrameBuffer theo seq, cho dòng [Status]
class FpsMeter {
public:
    explicit FpsMeter(std::shared_ptr<const hardware::camera::FrameBuffer> buffer)
        : buffer_(std::move(buffer)), lastSeq_(buffer_->latest().info.seq), lastTime_(Clock::now()) {}

    // fps trung bình từ lần gọi trước (hoặc lúc tạo) tới giờ
    double update() {
        const std::uint64_t seq  = buffer_->latest().info.seq;
        const auto          now  = Clock::now();
        const double        secs = std::chrono::duration<double>(now - lastTime_).count();
        const double        fps  = secs > 0.0 ? static_cast<double>(seq - lastSeq_) / secs : 0.0;
        lastSeq_  = seq;
        lastTime_ = now;
        return fps;
    }

private:
    using Clock = std::chrono::steady_clock;

    std::shared_ptr<const hardware::camera::FrameBuffer> buffer_;
    std::uint64_t                                        lastSeq_;
    Clock::time_point                                    lastTime_;
};

// Cửa sổ hiển thị, kéo to nhỏ được
static constexpr const char* kCameraWindow     = "Camera";
static constexpr const char* kLaneCameraWindow = "Lane Camera (undistorted)";
static const cv::Size        kWindowSize{960, 540};

static void open_window(const char* name) {
    cv::namedWindow(name, cv::WINDOW_NORMAL);
    cv::resizeWindow(name, kWindowSize.width, kWindowSize.height);
}

// Đọc phím từ stdin không chặn, không cần Enter, không echo. Ctrl+C vẫn dừng. Khôi phục cấu hình khi hủy.
class TerminalKeys {
public:
    TerminalKeys() {
        if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &saved_) != 0) return;
        termios raw = saved_;
        raw.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);
        raw.c_cc[VMIN]  = 0;   // read() trả về ngay, 0 byte nếu chưa có phím
        raw.c_cc[VTIME] = 0;
        active_ = tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0;
    }
    ~TerminalKeys() {
        if (active_) tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
    }
    TerminalKeys(const TerminalKeys&)            = delete;
    TerminalKeys& operator=(const TerminalKeys&) = delete;

    // Một phím mỗi lần gọi, -1 nếu chưa có
    int read_key() {
        if (!active_) return -1;
        unsigned char c = 0;
        if (::read(STDIN_FILENO, &c, 1) != 1) return -1;
        if (c != 27) return c;
        unsigned char seq[2] = {0, 0};
        // Không có byte tiếp theo: phím ESC thật
        if (::read(STDIN_FILENO, &seq[0], 1) != 1) return 27;
        if (seq[0] != '[' && seq[0] != 'O') return 27;
        if (::read(STDIN_FILENO, &seq[1], 1) != 1) return 27;
        return -1;   // phím đặc biệt (mũi tên...): bỏ qua
    }

private:
    termios saved_{};
    bool    active_ = false;
};

// Đổi mã cv::waitKeyEx sang mã dùng chung
static int window_key(int raw) {
    if (raw == -1) return -1;
    return raw & 0xFF;
}

// Phím '1'..'4' -> state
static constexpr std::array<statemachine::SystemMode, 4> kKeyStates{
    statemachine::SystemMode::Default,
    statemachine::SystemMode::Auto,
    statemachine::SystemMode::Manual,
    statemachine::SystemMode::Stop,
};

int main(int argc, char** argv) {
    // Đăng ký bắt tín hiệu Ctrl+C (SIGINT) và SIGTERM để tắt hệ thống an toàn
    std::signal(SIGINT, shutdown_signal_handler);
    std::signal(SIGTERM, shutdown_signal_handler);

    const std::string udpClientIp   = (argc >= 2) ? argv[1] : kUdpClientIp;
    const std::string tcpServerHost = (argc >= 3) ? argv[2] : kTcpServerHost;
    const hardware::serialhandler::SerialParams serialParams = kSerialParams;

    // Hủy theo thứ tự ngược khai báo: gateway sống lâu nhất vì subscriber unsubscribe khi bị hủy
    utils::QueuesList queues = utils::create_default_queues_list();

    gateway::ThreadGateway gateway(queues, std::cout, false);
    gateway.start();

    statemachine::StateMachine sm(queues, std::cout);

    // Một thread đọc cả hai camera: camera nhìn đường + camera bám làn (nắn méo)
    hardware::camera::ThreadCameraHandler camera(queues, kRoadCameraSource, kLaneCameraSource, kCameraCalibration,
                                                 cv::CAP_GSTREAMER, std::cerr, true);
    camera.start();

    // Network: UDP gửi hai luồng ảnh + telemetry, TCP nhận @vcb, @state/@mode, @distance (cần sm, nên sm khai báo trước)
    network::ThreadNetworkHandler net(queues, sm, camera.frame_buffer(), camera.lane_frame_buffer(),
                                      tcpServerHost, kTcpServerPort, std::cerr, false);
    if (!net.add_udp_client(udpClientIp, kUdpClientPort, kUdpLanePort, kUdpTextPort)) {
        std::cerr << "UDP client khong hop le: " << udpClientIp << ':' << kUdpClientPort
                  << " (cam bam lan " << kUdpLanePort << ", text " << kUdpTextPort << ")\n";
        return 1;
    }
    net.start();

    // Serial tới Nucleo. Khai báo sau net để hủy trước; destructor gửi lệnh dừng rồi đóng cổng.
    hardware::serialhandler::ThreadSerialHandler serial(queues, serialParams, std::cerr, false);
    serial.start();

    // Control: chỉ subscribe StateChange, phần điều khiển code sau. Khai báo sau serial để hủy trước serial.
    control::ThreadControlHandler control(queues, std::cerr, true);
    control.start();

    // Nhận thông báo CameraFrame / LaneCameraFrame, ảnh lấy từ FrameBuffer tương ứng
    MessageHandlerSubscriber frameSub    (queues, MessageName::CameraFrame,     "lastonly", true, "CameraViewer");
    MessageHandlerSubscriber laneFrameSub(queues, MessageName::LaneCameraFrame, "lastonly", true, "CameraViewer");
    const auto frames     = camera.frame_buffer();        // ảnh camera nhìn đường (cửa sổ Camera)
    const auto laneFrames = camera.lane_frame_buffer();   // ảnh camera bám làn đã nắn méo
    const auto rawFrames  = camera.raw_buffer();          // ảnh thô camera bám làn cho phím s

    // Chờ gateway xử lý xong các subscribe rồi mới gửi state khởi đầu
    std::this_thread::sleep_for(100ms);
    sm.initialize_starting_mode();

    std::cout << "Serial " << serialParams.device << " @" << serialParams.baudrate << ": "
              << (serial.is_connected() ? "da ket noi" : "chua ket noi (se tu thu lai)") << '\n';

    // Chờ hai camera mở (tối đa 30s). Thiếu camera đường thì thoát, thiếu camera bám làn thì chỉ cảnh báo.
    std::cout << "Dang cho camera mo..." << std::flush;
    for (int i = 0; i < 300 && !(camera.is_opened() && camera.is_lane_camera_opened())
                    && !g_shutdownRequested.load(); ++i) {
        std::this_thread::sleep_for(100ms);
        if (i % 10 == 0) std::cout << "." << std::flush;
    }
    std::cout << '\n';
    if (!camera.is_opened()) {
        if (g_shutdownRequested.load()) {
            std::cout << "Nhan tin hieu dung trong luc cho camera.\n";
        } else {
            std::cerr << "Khong mo duoc camera nhin duong (qua 30s timeout), thoat.\n";
        }
        return 1;
    }
    if (!camera.is_lane_camera_opened()) {
        std::cerr << "Chua mo duoc camera bam lan (se tu thu lai), PC chua nhan duoc anh de do lan.\n";
    }

    std::cout << "Dang xem camera. Nhan q hoac ESC de thoat, s de luu anh (test.jpg: anh tho cam bam lan, road.jpg: "
                 "cam nhin duong).\n"
              << "Phim 1..4 doi state: 1 DEFAULT, 2 AUTO, 3 MANUAL, 4 STOP.\n"
              << "UDP stream -> " << udpClientIp << ':' << kUdpLanePort << " (cam bam lan da nan meo), "
              << udpClientIp << ':' << kUdpClientPort << " (cam nhin duong) va " << udpClientIp << ':'
              << kUdpTextPort << " (text)\n"
              << "TCP client: " << tcpServerHost << ':' << kTcpServerPort
              << " (@vcb:speed;steer;dir, @state/@mode:<TEN STATE>, @distance:<m tu camera, am = khong co vat can>)\n";

    // Chỉ mở cửa sổ khi có màn hình
    const char* display     = std::getenv("DISPLAY");
    const bool  showWindows = display != nullptr && *display != '\0';
    if (showWindows) {
        open_window(kCameraWindow);
        open_window(kLaneCameraWindow);
    } else {
        std::cout << "Khong co man hinh (DISPLAY rong): khong mo cua so camera, dieu khien bang phim go trong "
                     "terminal nay.\n";
    }
    TerminalKeys terminal;
    hardware::camera::Frame current;   // frame cam nhìn đường đang hiển thị

    using Clock = std::chrono::steady_clock;
    auto nextStatus = Clock::now() + utils::logcfg::kMainStatusPeriod;
    FpsMeter cameraFps(frames);
    FpsMeter laneCameraFps(rawFrames);

    // imshow/waitKey phải chạy trên thread chính
    while (!g_shutdownRequested.load()) {
        if (const auto msg = frameSub.receive()) {
            if (std::any_cast<FrameInfo>(&*msg)) {
                current = frames->latest();   // có thể mới hơn seq trong thông báo (lastonly)
                if (showWindows && current.image) cv::imshow(kCameraWindow, *current.image);
            }
        }

        if (const auto msg = laneFrameSub.receive()) {
            const auto laneFrame = laneFrames->latest();
            if (showWindows && std::any_cast<FrameInfo>(&*msg) && laneFrame.image) {
                cv::imshow(kLaneCameraWindow, *laneFrame.image);
            }
        }

        // Dòng trạng thái tổng thể
        if (Clock::now() >= nextStatus) {
            nextStatus = Clock::now() + utils::logcfg::kMainStatusPeriod;
            const double camFps  = cameraFps.update();
            const double laneFps = laneCameraFps.update();
            std::cout << "[Status] state=" << statemachine::mode_to_string(sm.get_mode())
                      << " | serial=" << (serial.is_connected() ? "connected" : "lost")
                      << " | road_cam="
                      << (camera.is_opened() ? "opened " + utils::logcfg::format_fps(camFps) : "lost")
                      << " | lane_cam="
                      << (camera.is_lane_camera_opened() ? "opened " + utils::logcfg::format_fps(laneFps) : "lost")
                      << " | udp=" << (net.is_udp_enabled() ? "on" : "off")
                                   << (net.is_udp_open() ? "/open" : "/closed")
                      << " | tcp=" << (net.is_tcp_enabled() ? "on" : "off")
                                   << (net.is_tcp_connected() ? "/connected" : "/disconnected")
                      << " | control=" << (control.is_enabled() ? "on" : "off")
                      << '\n';
        }

        // Phím từ terminal trước, rồi từ cửa sổ; waitKeyEx cũng là khoảng nghỉ giữa các vòng
        int key = terminal.read_key();
        if (showWindows) {
            const int windowKey = window_key(cv::waitKeyEx(5));
            if (key == -1) key = windowKey;
        } else {
            std::this_thread::sleep_for(5ms);
        }
        if (key == 27 || key == 'q') {
            std::cout << "[Main] Chuyen ve STOP va tat he thong...\n";
            sm.request_mode(statemachine::SystemMode::Stop);
            std::this_thread::sleep_for(100ms);
            break;
        }
        if (key == 's') {
            // test.jpg: ảnh THÔ để calib (pick_marks.py tự nắn méo bằng bộ số trong main.cpp; lưu ảnh đã nắn méo thì
            // bị nắn méo hai lần)
            const auto rawFrame = rawFrames->latest();
            if (rawFrame.image) {
                cv::imwrite("test.jpg", *rawFrame.image);
                std::cout << "Da luu test.jpg (anh tho cam bam lan, seq=" << rawFrame.info.seq << ")\n";
            } else {
                std::cout << "Cam bam lan chua co anh, khong luu test.jpg\n";
            }
            if (current.image) {
                cv::imwrite("road.jpg", *current.image);
                std::cout << "Da luu road.jpg (cam nhin duong, seq=" << current.info.seq << ")\n";
            }
        }
        if (key >= '1' && key < '1' + static_cast<int>(kKeyStates.size())) {
            const statemachine::SystemMode target = kKeyStates[static_cast<std::size_t>(key - '1')];
            const bool changed = sm.request_mode(target);
            std::cout << "[Main] Request state " << statemachine::mode_to_string(target)
                      << (changed ? "" : " (da o state nay)") << '\n';
        }
    }

    // Nếu thoát do Ctrl+C mà chưa qua STOP, đưa về STOP
    if (sm.get_mode() != statemachine::SystemMode::Stop) {
        sm.request_mode(statemachine::SystemMode::Stop);
        std::this_thread::sleep_for(100ms);
    }

    if (showWindows) cv::destroyAllWindows();
    std::cout << "Dang tat he thong...\n";
    return 0;
}
