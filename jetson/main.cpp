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
#include <string>
#include <thread>
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
#include <utils/queues.hpp>

using namespace std::chrono_literals;
using utils::messages::FrameInfo;
using utils::messages::MessageHandlerSubscriber;
using utils::messages::MessageName;

// Chương trình xe: 2 camera (cam bám làn nắn méo + cam người lái) -> UDP về PC (main.py dò làn), TCP, serial, state
// machine. Phím (terminal hoặc cửa sổ): q / ESC thoát (về STOP), s lưu test.jpg (ảnh thô cam bám làn) + road.jpg,
// 1..4 đổi state (DEFAULT, AUTO, MANUAL, STOP).
// Chạy: ./braincar [<ip nhận UDP | any> [<ip server TCP>]]

// Sensor như "nvarguscamerasrc sensor-id=N" trên Jetson: sensor N = camera thứ N của "rpicam-hello --list-cameras".
// Hai cửa sổ bị tráo ảnh thì đổi hai số này cho nhau.
static constexpr int kLaneSensorId = 0;   // cam bám làn
static constexpr int kRoadSensorId = 1;   // cam người lái

// Pipeline chung cho hai camera (sau dòng chọn sensor). Phải giữ đúng hình học lúc calib kCameraCalibration: xin
// 960x540 để libcamera chọn sensor mode 1920x1080, rồi kéo giãn về 640x480 (vì vậy fx ≠ fy).
static const std::string kCameraPipeline =
    "video/x-raw, width=960, height=540, format=NV12, framerate=30/1 ! "
    "videoscale add-borders=false ! "
    "video/x-raw, width=640, height=480 ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! "
    "appsink drop=true max-buffers=1 sync=false";

// Calib ống kính cam bám làn ở 640x480 (homography nằm ở PC, GroundCalibration trong main.py)
static const perception::CameraCalibration kCameraCalibration{
    cv::Size(640, 480),
    cv::Matx33d(512.492640, 0.0, 287.186733,
                0.0, 687.516488, 284.552142,
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

// camera-name theo thứ tự của "rpicam-hello --list-cameras": dòng "N : imx219 [...] (<tên>)". Rỗng nếu không chạy được.
static std::vector<std::string> list_libcamera_cameras() {
    std::vector<std::string> names;
    FILE* pipe = popen("rpicam-hello --list-cameras 2>&1", "r");
    if (pipe == nullptr) return names;
    char buffer[512];
    while (std::fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        const std::string line(buffer);
        const auto first = line.find_first_not_of(" \t");
        const auto colon = line.find(" : ");
        const auto open  = line.rfind('(');
        const auto close = line.rfind(')');
        if (first == std::string::npos || !std::isdigit(static_cast<unsigned char>(line[first]))
            || colon == std::string::npos || open == std::string::npos || close == std::string::npos || close < open
            || line.substr(first, colon - first) != std::to_string(names.size())) {
            continue;   // không phải dòng camera (dòng mode, log...)
        }
        names.push_back(line.substr(open + 1, close - open - 1));
    }
    pclose(pipe);
    return names;
}

// Pipeline của sensor N. Không có sensor này thì báo lỗi; ThreadCameraHandler sẽ thử mở lại, hệ thống vẫn chạy.
static std::string camera_source(int sensorId, const std::vector<std::string>& names) {
    const auto id = static_cast<std::size_t>(sensorId);
    if (id < names.size()) return "libcamerasrc camera-name=\"" + names[id] + "\" ! " + kCameraPipeline;
    if (names.empty() && sensorId == 0) return "libcamerasrc ! " + kCameraPipeline;   // camera đầu tiên
    std::cerr << "[Main] ERROR: Khong thay sensor " << sensorId << " (rpicam-hello thay " << names.size()
              << " camera). Kiem tra cap va dtoverlay trong /boot/firmware/config.txt\n";
    return "libcamerasrc camera-name=\"sensor-" + std::to_string(sensorId) + "-not-found\" ! " + kCameraPipeline;
}

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

    const std::vector<std::string> cameraNames = list_libcamera_cameras();
    const std::string laneSource = camera_source(kLaneSensorId, cameraNames);
    const std::string roadSource = camera_source(kRoadSensorId, cameraNames);

    // Hủy theo thứ tự ngược khai báo: gateway sống lâu nhất vì subscriber unsubscribe khi bị hủy
    utils::QueuesList queues = utils::create_default_queues_list();
    gateway::ThreadGateway gateway(queues, std::cout, false);
    gateway.start();

    statemachine::StateMachine sm(queues, std::cout);

    hardware::camera::ThreadCameraHandler camera(queues, roadSource, laneSource, kCameraCalibration);
    camera.start();

    network::ThreadNetworkHandler net(queues, sm, camera.frame_buffer(), camera.lane_frame_buffer(),
                                      tcpServerHost, kTcpServerPort);
    if (!net.add_udp_client(udpClientIp, kUdpClientPort, kUdpLanePort, kUdpTextPort)) {
        std::cerr << "UDP client khong hop le: " << udpClientIp << '\n';
        return 1;
    }
    net.start();

    // Serial khai báo sau net để hủy trước: destructor gửi lệnh dừng rồi đóng cổng
    hardware::serialhandler::ThreadSerialHandler serial(queues, kSerialParams);
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
                cv::imshow(kCameraWindow, no_signal_image("Cam nguoi lai (sensor " + std::to_string(kRoadSensorId) + ")"));
            }
            if (!laneFrames->latest().image) {
                cv::imshow(kLaneCameraWindow, no_signal_image("Cam bam lan (sensor " + std::to_string(kLaneSensorId) + ")"));
            }
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
