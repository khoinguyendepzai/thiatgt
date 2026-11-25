#include <any>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <termios.h>
#include <unistd.h>

#include <control/threadControlHandler.hpp>
#include <gateway/threadGateWay.hpp>
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

// Chương trình xe: 2 camera (cam bám làn nắn méo + cam người lái) -> UDP về PC (main.py dò làn), TCP, serial, state
// machine. Phím (terminal hoặc cửa sổ): q / ESC thoát (về STOP), s lưu test.jpg (ảnh thô cam bám làn) + road.jpg,
// 1..4 đổi state (DEFAULT, AUTO, MANUAL, STOP).
// Chạy: ./braincar [<ip nhận UDP | any> [<ip server TCP>]]

// Calib ống kính cam bám làn ở 640x480 (homography nằm ở PC, GroundCalibration trong main.py)
static const perception::CameraCalibration kCameraCalibration{
    cv::Size(640, 480),
    cv::Matx33d(512.492640, 0.0, 287.186733,    // fx, 0, cx
                0.0, 687.516488, 284.552142,    // 0, fy, cy
                0.0, 0.0, 1.0),
    cv::Vec<double, 5>(-0.35453198, 0.18076870, -0.00151746, 0.00140839, -0.06370173),   // k1, k2, p1, p2, k3
};

// Cổng UDP phải khớp LANE_PORT / ROAD_PORT trong main.py
static const std::string       kUdpClientIp   = "0.0.0.0";   // 0.0.0.0 / any = broadcast
static constexpr std::uint16_t kUdpLanePort   = 7000;        // cam bám làn đã nắn méo
static constexpr std::uint16_t kUdpClientPort = 7001;        // cam người lái
static constexpr std::uint16_t kUdpTextPort   = 9000;        // telemetry text
static const std::string       kTcpServerHost = "0.0.0.0";
static constexpr std::uint16_t kTcpServerPort = 6000;
static const hardware::serialhandler::SerialParams kSerialParams{"/dev/ttyACM0", 115200};

static constexpr const char* kCameraWindow     = "Camera";
static constexpr const char* kLaneCameraWindow = "Lane Camera (undistorted)";

static std::atomic<bool> g_shutdownRequested{false};
static void shutdown_signal_handler(int) { g_shutdownRequested.store(true); }

// Đo fps ghi vào một FrameBuffer theo seq, cho dòng [Status] (giống Brain / BrainUp)
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

// Ảnh hiện trong cửa sổ khi camera chưa có frame
static cv::Mat no_signal_image(const std::string& camera) {
    cv::Mat image(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::putText(image, "NO SIGNAL", {20, 60}, cv::FONT_HERSHEY_SIMPLEX, 1.2, {0, 0, 255}, 2);
    cv::putText(image, camera, {20, 120}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {255, 255, 255}, 2);
    return image;
}

// Đọc phím từ terminal không chặn, không cần Enter. Khôi phục cấu hình terminal khi hủy.
class TerminalKeys {
public:
    TerminalKeys() {
        if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &saved_) != 0) return;
        termios raw = saved_;
        raw.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);
        raw.c_cc[VMIN]  = 0;
        raw.c_cc[VTIME] = 0;
        active_ = tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0;
    }
    ~TerminalKeys() { if (active_) tcsetattr(STDIN_FILENO, TCSANOW, &saved_); }
    TerminalKeys(const TerminalKeys&)            = delete;
    TerminalKeys& operator=(const TerminalKeys&) = delete;

    // Một phím mỗi lần gọi, -1 nếu chưa có. Phím mũi tên (ESC [ x) bỏ qua, ESC đứng một mình = 27.
    int read_key() {
        unsigned char c = 0;
        if (!active_ || ::read(STDIN_FILENO, &c, 1) != 1) return -1;
        if (c != 27) return c;
        unsigned char seq[2];
        if (::read(STDIN_FILENO, &seq[0], 1) != 1 || (seq[0] != '[' && seq[0] != 'O')) return 27;
        if (::read(STDIN_FILENO, &seq[1], 1) != 1) return 27;
        return -1;
    }

private:
    termios saved_{};
    bool    active_ = false;
};

static constexpr std::array<statemachine::SystemMode, 4> kKeyStates{
    statemachine::SystemMode::Default, statemachine::SystemMode::Auto,
    statemachine::SystemMode::Manual,  statemachine::SystemMode::Stop,
};

int main(int argc, char** argv) {
    std::signal(SIGINT, shutdown_signal_handler);
    std::signal(SIGTERM, shutdown_signal_handler);

    // Tắt log INFO của libcamera và WARN của OpenCV (camera thiếu thì thử mở lại mỗi giây, log rất dày).
    // Đặt trước mọi lệnh OpenCV/camera: hai thư viện đọc biến môi trường ở lần dùng đầu tiên.
    setenv("LIBCAMERA_LOG_LEVELS", "*:ERROR", 0);
    setenv("OPENCV_LOG_LEVEL", "ERROR", 0);

    const std::string udpClientIp   = (argc >= 2) ? argv[1] : kUdpClientIp;
    const std::string tcpServerHost = (argc >= 3) ? argv[2] : kTcpServerHost;

    // Pipeline Jetson, chép nguyên từ README của BrainUp (mục 3.4). Xin sensor mode 1920x1080 của IMX219 (vùng cắt 16:9
    // giữa cảm biến), nvvidconv xoay 180° (cam lắp ngược, flip-method=2) và thu nhỏ bằng phần cứng, videoconvert
    // BGRx -> BGR cho OpenCV.
    //   - Cam bám làn: sensor-id=0, kéo giãn cả khung 16:9 về 640x480 = đúng hình học lúc calib kCameraCalibration
    //     (nên fx ≠ fy). Đổi bất cứ gì ở pipeline này thì phải calib lại.
    //   - Cam người lái: sensor-id=1, 960x540 (giữ 16:9).
    // Hai cửa sổ bị tráo ảnh thì đổi sensor-id=0 và sensor-id=1 cho nhau.
    const std::string laneSource =
        "nvarguscamerasrc sensor-id=0 ! "
        "video/x-raw(memory:NVMM), width=1920, height=1080, format=NV12, framerate=30/1 ! "
        "nvvidconv flip-method=2 ! "
        "video/x-raw, width=640, height=480, format=BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false";
    const std::string roadSource =
        "nvarguscamerasrc sensor-id=1 ! "
        "video/x-raw(memory:NVMM), width=1920, height=1080, format=NV12, framerate=30/1 ! "
        "nvvidconv flip-method=2 ! "
        "video/x-raw, width=960, height=540, format=BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false";

    // Hủy theo thứ tự ngược khai báo: gateway sống lâu nhất vì subscriber unsubscribe khi bị hủy
    utils::QueuesList queues = utils::create_default_queues_list();
    gateway::ThreadGateway gateway(queues, std::cout, false);
    gateway.start();

    statemachine::StateMachine sm(queues, std::cout);

    hardware::camera::ThreadCameraHandler camera(queues, roadSource, laneSource, kCameraCalibration);
    camera.start();

    // Tham số cuối = debugging: true thì in dữ liệu TCP nhận được ([TcpClient][debug] RX raw + #vcb đã tách) và log
    // gửi UDP. Đổi true/false ở đây rồi build lại.
    network::ThreadNetworkHandler net(queues, sm, camera.frame_buffer(), camera.lane_frame_buffer(),
                                      tcpServerHost, kTcpServerPort, std::cerr, true);
    if (!net.add_udp_client(udpClientIp, kUdpClientPort, kUdpLanePort, kUdpTextPort)) {
        std::cerr << "UDP client khong hop le: " << udpClientIp << '\n';
        return 1;
    }
    net.start();

    // Serial khai báo sau net để hủy trước: destructor gửi lệnh dừng rồi đóng cổng
    // Tham số cuối = debugging: true thì in lệnh ThreadWrite ghi xuống Nucleo ([ThreadWrite][debug]) và tin ThreadRead
    // nhận từ Nucleo. Đổi true/false ở đây rồi build lại.
    hardware::serialhandler::ThreadSerialHandler serial(queues, kSerialParams, std::cerr, true);
    serial.start();

    // Control khai báo sau serial để hủy trước serial (phần điều khiển code sau)
    control::ThreadControlHandler control(queues);
    control.start();

    MessageHandlerSubscriber frameSub    (queues, MessageName::CameraFrame,     "lastonly", true, "CameraViewer");
    MessageHandlerSubscriber laneFrameSub(queues, MessageName::LaneCameraFrame, "lastonly", true, "CameraViewer");
    const auto frames     = camera.frame_buffer();
    const auto laneFrames = camera.lane_frame_buffer();
    const auto rawFrames  = camera.raw_buffer();

    // Chờ gateway xử lý xong các subscribe rồi mới gửi state khởi đầu
    std::this_thread::sleep_for(100ms);
    sm.initialize_starting_mode();

    // Luôn mở cửa sổ; chạy qua SSH (DISPLAY rỗng) thì hiện lên màn hình của Pi. Không mở được thì dùng phím terminal.
    const char* display = std::getenv("DISPLAY");
    if (display == nullptr || *display == '\0') setenv("DISPLAY", ":0", 1);
    bool showWindows = true;
    try {
        for (const char* name : {kCameraWindow, kLaneCameraWindow}) {
            cv::namedWindow(name, cv::WINDOW_NORMAL);
            cv::resizeWindow(name, 960, 540);
        }
    } catch (const cv::Exception&) {
        showWindows = false;
        std::cerr << "Khong mo duoc cua so, dieu khien bang phim trong terminal.\n";
    }

    TerminalKeys terminal;
    using Clock = std::chrono::steady_clock;
    auto nextNoSignal = Clock::now();
    auto nextStatus   = Clock::now() + utils::logcfg::kMainStatusPeriod;
    FpsMeter roadCameraFps(frames);
    FpsMeter laneCameraFps(rawFrames);   // ảnh thô: đếm đúng số frame camera đọc được

    // imshow/waitKey phải chạy trên thread chính
    while (!g_shutdownRequested.load()) {
        if (const auto msg = frameSub.receive(); msg && showWindows && std::any_cast<FrameInfo>(&*msg)) {
            if (const auto f = frames->latest(); f.image) cv::imshow(kCameraWindow, *f.image);
        }
        if (const auto msg = laneFrameSub.receive(); msg && showWindows && std::any_cast<FrameInfo>(&*msg)) {
            if (const auto f = laneFrames->latest(); f.image) cv::imshow(kLaneCameraWindow, *f.image);
        }

        // Camera chưa có ảnh: hiện NO SIGNAL thay vì cửa sổ đen trống (mỗi giây một lần)
        if (showWindows && Clock::now() >= nextNoSignal) {
            nextNoSignal = Clock::now() + 1s;
            if (!frames->latest().image) {
                cv::imshow(kCameraWindow, no_signal_image("Cam nguoi lai (sensor 1)"));
            }
            if (!laneFrames->latest().image) {
                cv::imshow(kLaneCameraWindow, no_signal_image("Cam bam lan (sensor 0)"));
            }
        }

        // Dòng trạng thái tổng thể (giống Brain / BrainUp, bỏ detect/lane/lc vì xe không còn dò làn, bám làn)
        if (Clock::now() >= nextStatus) {
            nextStatus = Clock::now() + utils::logcfg::kMainStatusPeriod;
            const double roadFps = roadCameraFps.update();
            const double laneFps = laneCameraFps.update();
            std::cout << "[Status] state=" << statemachine::mode_to_string(sm.get_mode())
                      << " | serial=" << (serial.is_connected() ? "connected" : "lost")
                      << " | road_cam="
                      << (camera.is_opened() ? "opened " + utils::logcfg::format_fps(roadFps) : "lost")
                      << " | lane_cam="
                      << (camera.is_lane_camera_opened() ? "opened " + utils::logcfg::format_fps(laneFps) : "lost")
                      << " | udp=" << (net.is_udp_enabled() ? "on" : "off")
                                   << (net.is_udp_open() ? "/open" : "/closed")
                      << " | tcp=" << (net.is_tcp_enabled() ? "on" : "off")
                                   << (net.is_tcp_connected() ? "/connected" : "/disconnected")
                      << '\n';
        }

        // Phím từ terminal trước, rồi từ cửa sổ; waitKey cũng là khoảng nghỉ giữa các vòng
        int key = terminal.read_key();
        if (showWindows) {
            const int windowKey = cv::waitKey(5);
            if (key == -1 && windowKey != -1) key = windowKey & 0xFF;
        } else {
            std::this_thread::sleep_for(5ms);
        }

        if (key == 27 || key == 'q') break;
        if (key == 's') {
            // test.jpg là ảnh THÔ để calib (pick_marks.py tự nắn méo)
            if (const auto f = rawFrames->latest(); f.image) cv::imwrite("test.jpg", *f.image);
            if (const auto f = frames->latest(); f.image) cv::imwrite("road.jpg", *f.image);
            std::cout << "Da luu test.jpg, road.jpg\n";
        }
        if (key >= '1' && key < '1' + static_cast<int>(kKeyStates.size())) {
            sm.request_mode(kKeyStates[static_cast<std::size_t>(key - '1')]);
        }
    }

    // Thoát (q, ESC, Ctrl+C): về STOP trước khi tắt
    sm.request_mode(statemachine::SystemMode::Stop);
    std::this_thread::sleep_for(100ms);
    if (showWindows) cv::destroyAllWindows();
    return 0;
}
