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

// Chương trình xe (Jetson): 2 camera (cam bám làn nắn méo + cam nhìn đường) -> UDP về PC (main_master.py), TCP nhận
// lệnh #vcb, serial xuống NUCLEO, state machine. Phím (terminal hoặc cửa sổ): q / ESC thoát (về STOP), s lưu test.jpg
// (ảnh thô cam bám làn) + road.jpg, 1..4 đổi state (DEFAULT, AUTO, MANUAL, STOP).
// Chạy: ./braincar [<ip PC nhận UDP | any> [<ip PC chạy server TCP>]]   (mặc định cả hai là kPcIp)

// sensor-id của nvarguscamerasrc: 0 = cổng CAM0, 1 = cổng CAM1 (Jetson Nano B01 / Xavier NX có 2 cổng; Nano A02 chỉ
// có 1 cổng). Hai cửa sổ bị tráo ảnh thì đổi hai số này cho nhau. Đặt -1 để TẮT camera đó (board chỉ có 1 cổng,
// chạy thử 1 cam): không thử mở, không làm chậm camera còn lại.
// Dây trên xe: cam bám làn cắm CAM1, cam nhìn đường cắm CAM0. Đổi ở đây thì đổi cả LANE_SENSOR_ID trong
// calib/calib_camera.py.
static constexpr int kLaneSensorId = 1;   // cam bám làn
static constexpr int kRoadSensorId = 0;   // cam nhìn đường (YOLO)

// flip-method của nvvidconv: 0 = giữ nguyên, 2 = xoay 180° (cam lắp ngược). Áp cho CẢ HAI camera. Phải giống
// FLIP_METHOD trong calib/calib_camera.py: K và hệ số méo đo ở hướng nào thì chỉ đúng ở hướng đó (xoay 180° làm đổi
// cx, cy, p1, p2).
// Cam lắp ngược trên xe nên phải là 2. kCameraCalibration và homography trên PC đều đo ở hướng này: bản Pi dùng
// libcamera, libcamera tự xoay 180° theo "rotation" của device tree; nvarguscamerasrc thì không tự xoay, để 0 là ảnh
// lộn ngược.
static constexpr int kFlipMethod = 2;

// Cỡ ảnh mỗi camera. Cam bám làn 640x480: đúng cỡ calib kCameraCalibration (ảnh 16:9 kéo giãn về 4:3, nên fx ≠ fy).
// Cam nhìn đường 640x360 (giữ tỉ lệ 16:9): main_master.py nhận đúng cỡ này (ROAD_FRAME_SIZE); ảnh bị kéo giãn thì
// chiều cao khung bao sai và khoảng cách D = f·H/h của YOLO sai theo.
static constexpr int kLaneWidth = 640, kLaneHeight = 480;
static constexpr int kRoadWidth = 640, kRoadHeight = 360;

// Calib ống kính cam bám làn ở 640x480 (homography nằm ở PC, GroundCalibration trong main.py)
static const perception::CameraCalibration kCameraCalibration{
    cv::Size(640, 480),
    cv::Matx33d(512.492640, 0.0, 287.186733,    // fx, 0, cx
                0.0, 687.516488, 284.552142,    // 0, fy, cy
                0.0, 0.0, 1.0),
    cv::Vec<double, 5>(-0.35453198, 0.18076870, -0.00151746, 0.00140839, -0.06370173),   // k1, k2, p1, p2, k3
};

// IP của PC chạy main_master.py (xem bằng ipconfig trên PC). Gửi thẳng tới PC, KHÔNG broadcast: trên Wi-Fi gói broadcast
// đi ở tốc độ cơ bản rất thấp, ảnh 2 camera cắt thành nhiều gói sẽ rớt gói liên tục và PC không ghép đủ ảnh nào.
// TCP client không được để 0.0.0.0: Linux hiểu 0.0.0.0 là chính máy này, xe sẽ tự nối vào chính nó.
static const std::string kPcIp = "192.168.1.109";

// Cổng phải khớp --lane-port / --road-port / --tcp-port của main_master.py
static const std::string       kUdpClientIp   = kPcIp;       // "any" / "0.0.0.0" = broadcast (không nên dùng)
static constexpr std::uint16_t kUdpLanePort   = 7000;        // cam bám làn đã nắn méo
static constexpr std::uint16_t kUdpClientPort = 7001;        // cam nhìn đường
static constexpr std::uint16_t kUdpTextPort   = 9000;        // telemetry text (main_master không nghe cổng này, vô hại)
static const std::string       kTcpServerHost = kPcIp;
static constexpr std::uint16_t kTcpServerPort = 6000;
static const hardware::serialhandler::SerialParams kSerialParams{"/dev/ttyACM0", 115200};

static constexpr const char* kCameraWindow     = "Camera";
static constexpr const char* kLaneCameraWindow = "Lane Camera (undistorted)";

static std::atomic<bool> g_shutdownRequested{false};
static void shutdown_signal_handler(int) { g_shutdownRequested.store(true); }

// Pipeline GStreamer của một camera CSI trên Jetson. Xin sensor mode 1920x1080 (16:9, vùng cắt giữa cảm biến giống mode
// 1920x1080 lúc calib), nvvidconv xoay + thu nhỏ bằng phần cứng, videoconvert BGRx -> BGR cho OpenCV.
// sensorId < 0: chuỗi rỗng = camera bị tắt (ThreadCameraHandler không thử mở).
static std::string camera_source(int sensorId, int width, int height) {
    if (sensorId < 0) return "";
    return "nvarguscamerasrc sensor-id=" + std::to_string(sensorId) + " ! "
           "video/x-raw(memory:NVMM), width=1920, height=1080, format=NV12, framerate=30/1 ! "
           "nvvidconv flip-method=" + std::to_string(kFlipMethod) + " ! "
           "video/x-raw, width=" + std::to_string(width) + ", height=" + std::to_string(height) + ", format=BGRx ! "
           "videoconvert ! video/x-raw, format=BGR ! "
           "appsink drop=true max-buffers=1 sync=false";
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

    // Tắt WARN của OpenCV (camera thiếu thì thử mở lại, log rất dày). Đặt trước mọi lệnh OpenCV: đọc ở lần dùng đầu.
    setenv("OPENCV_LOG_LEVEL", "ERROR", 0);

    const std::string udpClientIp   = (argc >= 2) ? argv[1] : kUdpClientIp;
    const std::string tcpServerHost = (argc >= 3) ? argv[2] : kTcpServerHost;
    std::cout << "[Main] Anh UDP -> " << udpClientIp << " (cong " << kUdpLanePort << ", " << kUdpClientPort
              << "), TCP server " << tcpServerHost << ':' << kTcpServerPort << '\n';

    const std::string laneSource = camera_source(kLaneSensorId, kLaneWidth, kLaneHeight);
    const std::string roadSource = camera_source(kRoadSensorId, kRoadWidth, kRoadHeight);

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

    // Chỉ mở cửa sổ khi có DISPLAY (chạy từ terminal trên màn hình Jetson, hoặc ssh -X / DISPLAY=:0 ./braincar).
    // Không tự đặt DISPLAY=:0 như bản Pi: OpenCV của JetPack (GTK) gọi gtk_init(), không mở được màn hình là THOÁT luôn
    // chương trình, không ném exception để bắt. Không có màn hình thì điều khiển bằng phím trong terminal.
    const char* display     = std::getenv("DISPLAY");
    bool        showWindows = display != nullptr && *display != '\0';
    if (!showWindows) {
        std::cout << "[Main] Khong co DISPLAY: chay khong cua so, bam phim trong terminal (1..4 doi state, q thoat).\n";
    }
    if (showWindows) try {
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
                cv::imshow(kCameraWindow, no_signal_image("Cam nhin duong (sensor " + std::to_string(kRoadSensorId) + ")"));
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
