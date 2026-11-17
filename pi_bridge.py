#!/usr/bin/env python3
"""
pi_bridge.py - chạy trên Raspberry Pi 5: nối PC (main_master.py) với Nucleo (firmware của anh An) và gửi 2 camera.

    PC main_master.py --TCP 6000, "@vcb:speed;steer;dir;;\\r\\n" 50 lần/s--> Pi 5 --USB (ST-LINK)--> Nucleo
         "#speed:<m/s>;;\\r\\n"  "#steer:<độ, dương = phải>;;\\r\\n"  "#dir:<1/0/-1>;;\\r\\n"
    Camera bird's-eye (giữ làn, chuyển làn) --UDP 7000, mỗi gói một ảnh JPEG 640x480--> PC
    Camera nhìn đường (YOLO, khoảng cách)    --UDP 7001, mỗi gói một ảnh JPEG 640x360--> PC
    Nucleo --"@hall:<m/s>;...;;"  "@imu:roll;pitch;yaw;ax;ay;az;;"--> Pi 5 (in trạng thái mỗi giây)

Giao thức serial lấy từ phần mềm BrainUp của anh An (nhánh DucAn), firmware không được chia sẻ:
  - ThreadTcpClient tách "@vcb:speed;steer;dir" thành 3 giá trị Speed, Steer, Directory, rồi ThreadWrite ghi xuống
    Nucleo mỗi giá trị một dòng "#action:giá_trị;;\\r\\n" (MessageConverter). pi_bridge làm đúng như vậy.
  - Speed: m/s. Steer: độ, "#steer dương = phải", tối đa ±24 (ThreadLaneKeeping::kMaxSteerDeg). Directory: 1 tới,
    -1 lùi, 0 đứng. Lệnh dừng (ThreadWrite::send_stop): speed 0, steer 0, dir 0.
  - Nucleo gửi lên "@hall:speed;..." (CurrentSpeed) và "@imu:roll;pitch;yaw;accelx;accely;accelz" (ThreadRead).
  - Tên lệnh "speed", "steer", "dir" suy ra từ send_speed/send_steer/send_dir của ThreadWrite: bảng commands() trong
    messageConverter.cpp không có ở đây. Sai tên thì sửa CMD_SPEED / CMD_STEER / CMD_DIR bên dưới.

IP, cổng, cổng serial, tên lệnh và pipeline camera là HẰNG SỐ ngay bên dưới (như main.cpp của BrainUp).
Tham số dòng lệnh chỉ để ghi đè tạm khi thử.

Cài trên Pi (Raspberry Pi OS Bookworm) - DÙNG OpenCV CỦA APT (có GStreamer), KHÔNG dùng opencv-python của pip:
    sudo apt install python3-opencv python3-serial gstreamer1.0-libcamera gstreamer1.0-tools \\
                     gstreamer1.0-plugins-base gstreamer1.0-plugins-good
Xem tên 2 camera:  rpicam-hello --list-cameras   (chép đường dẫn trong ngoặc vào LANE_/ROAD_CAMERA_NAME)
Thử camera:        python3 pi_bridge.py --camera-test lane      (hoặc road)
Chạy:              python3 pi_bridge.py

Chuỗi in ra console viết không dấu (quy ước của dự án).
"""

import argparse
import glob
import math
import socket
import threading
import time

# ======================================= CẤU HÌNH (sửa ở đây) =======================================
PC_IP = "192.168.1.122"        # IP máy PC chạy main_master.py (trên PC gõ ipconfig để xem; đổi Wi-Fi có thể đổi IP)
PC_TCP_PORT = 6000             # main_master: TCP server, xe kết nối vào để nhận lệnh @vcb

# Nucleo cắm vào Pi bằng cáp mini-USB của ST-LINK -> /dev/ttyACM0 (kSerialParams của BrainUp). Lidar là /dev/ttyUSB0.
NUCLEO_SERIAL = "/dev/ttyACM0"
NUCLEO_BAUD = 115200
CMD_SPEED = "speed"            # "#speed:<m/s>;;"
CMD_STEER = "steer"            # "#steer:<độ>;;", dương = phải
CMD_DIR = "dir"                # "#dir:<1/0/-1>;;"
MAX_SPEED_MPS = 1.5            # kẹp tốc độ gửi xuống Nucleo
MAX_STEER_DEG = 24.0           # kẹp góc lái (ThreadLaneKeeping::kMaxSteerDeg)

# Hai camera CSI của Pi 5. Tên lấy từ "rpicam-hello --list-cameras" (đường dẫn trong ngoặc); để "" thì libcamerasrc
# lấy camera đầu tiên. Giống BrainUp: camera bird's-eye ở sensor-id 0, camera nhìn đường ở sensor-id 1.
LANE_CAMERA_NAME = "/base/axi/pcie@1000120000/rp1/i2c@88000/imx219@10"   # cổng CAM/DISP 0 (kiểm tra lại trên Pi)
ROAD_CAMERA_NAME = "/base/axi/pcie@120000/rp1/i2c@80000/imx219@10"   # cổng CAM/DISP 1 (kiểm tra lại trên Pi)


def camera_pipeline(camera_name, width, height):
    """Như kLaneCameraSource / kRoadCameraSource trên Jetson: chụp 1920x1080 30 fps, thu nhỏ (kéo dãn, không viền
    đen, như nvvidconv), xoay 180 độ (flip-method=2), đổi sang BGR cho OpenCV. Jetson: nvarguscamerasrc / nvvidconv;
    Pi 5: libcamerasrc / videoscale / videoflip."""
    name = f' camera-name="{camera_name}"' if camera_name else ""
    return (f"libcamerasrc{name} ! "
            "video/x-raw, width=1920, height=1080, framerate=30/1 ! "
            "videoscale add-borders=false ! "
            f"video/x-raw, width={width}, height={height} ! "
            "videoflip method=rotate-180 ! "
            "videoconvert ! "
            "video/x-raw, format=BGR ! "
            "appsink drop=true max-buffers=1 sync=false")


# Camera bird's-eye: giữ làn, LDWS, chuyển làn (main_master: LaneVision + planner). 640x480 = cỡ GroundCalibration.
LANE_CAMERA_SOURCE = camera_pipeline(LANE_CAMERA_NAME, 640, 480)
LANE_FRAME_SIZE = (640, 480)
LANE_VIDEO_PORT = 7000         # main_master --lane-port
LANE_SOURCE_PORT = 9000        # cổng nguồn UDP trên Pi (kUdpPort)

# Camera nhìn đường: YOLO đo khoảng cách (main_master: ObstacleDetector, đưa ảnh về 1280x720 16:9). 640x360 giữ đúng
# tỉ lệ 16:9 và vừa một gói UDP; YOLO (imgsz 640) cũng thu ảnh về khổ 640 nên không mất chi tiết.
ROAD_CAMERA_SOURCE = camera_pipeline(ROAD_CAMERA_NAME, 640, 360)
ROAD_FRAME_SIZE = (640, 360)
ROAD_VIDEO_PORT = 7001         # main_master --road-port
ROAD_SOURCE_PORT = 9001

JPEG_QUALITY = 85              # kJpegQuality của BrainUp
MAX_UDP_PAYLOAD = 65507        # một gói UDP/IPv4


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def parse_vcb(line):
    """'@vcb:0.813;5.000;1;;' -> (speed, steer, dir) hoặc None (sai định dạng / không phải @vcb)."""
    line = line.strip()
    if not line.startswith("@vcb:"):
        return None
    fields = line[5:].split(";")
    if len(fields) < 3:
        return None
    try:
        speed, steer, direction = float(fields[0]), float(fields[1]), int(float(fields[2]))
    except ValueError:
        return None
    if not (math.isfinite(speed) and math.isfinite(steer)):
        return None
    return max(0.0, speed), steer, direction


def encode_commands(speed, steer_deg, direction):
    """Ba dòng lệnh cho Nucleo, cùng thứ tự với @vcb (speed, steer, dir), như ThreadWrite của BrainUp."""
    speed = clamp(speed, 0.0, MAX_SPEED_MPS)
    steer = clamp(steer_deg, -MAX_STEER_DEG, MAX_STEER_DEG)
    d = 1 if direction > 0 else (-1 if direction < 0 else 0)
    return (f"#{CMD_SPEED}:{speed:.3f};;\r\n"
            f"#{CMD_STEER}:{steer:.3f};;\r\n"
            f"#{CMD_DIR}:{d};;\r\n").encode("ascii")


def parse_nucleo_line(line):
    """'@hall:0.31;;' -> ('hall', ['0.31']). Dòng không phải '@action:...' -> None."""
    line = line.strip()
    if not line.startswith("@") or ":" not in line:
        return None
    action, values = line[1:].split(":", 1)
    return action, [v for v in values.split(";") if v != ""]


# ========================================= Nucleo (USB serial) =========================================
class NucleoLink:
    """Ghi lệnh xuống Nucleo, đọc @hall / @imu. Rút cáp / Nucleo khởi động lại thì tự mở lại cổng mỗi 1 s.
    port: /dev/ttyACM0, /dev/serial/by-id/..., COM5 (Windows) hoặc URL pyserial (socket://host:port để thử)."""

    REOPEN_S = 1.0

    def __init__(self, port, baud=NUCLEO_BAUD):
        self.port, self.baud = port, baud
        self.ser = None
        self.lock = threading.Lock()
        self.sent = 0                  # số lần gửi bộ 3 lệnh
        self.errors = 0
        self.speed = None              # m/s từ @hall
        self.speed_time = 0.0
        self.imu = None                # [roll, pitch, yaw, ax, ay, az] từ @imu
        self.other = {}                # action khác Nucleo gửi lên (để in)
        self._next_open = 0.0
        self._warned = False
        self._running = True
        self._reader = threading.Thread(target=self._read_loop, name="nucleo-read", daemon=True)
        self._reader.start()

    def _open(self):
        import serial  # sudo apt install python3-serial
        if "://" in self.port:
            return serial.serial_for_url(self.port, baudrate=self.baud, timeout=0.05, write_timeout=0.05)
        ser = serial.Serial()
        ser.port, ser.baudrate, ser.timeout, ser.write_timeout = self.port, self.baud, 0.05, 0.05
        ser.dtr = False   # ST-LINK không reset STM32 theo DTR/RTS; tắt cho chắc
        ser.rts = False
        ser.open()
        return ser

    def maintain(self):
        """Mở cổng nếu đang đóng (tối đa mỗi REOPEN_S). Gọi định kỳ từ vòng chính."""
        if self.ser is not None or time.monotonic() < self._next_open:
            return
        with self.lock:
            if self.ser is not None:
                return
            self._next_open = time.monotonic() + self.REOPEN_S
            try:
                self.ser = self._open()
                self._warned = False
                print(f"[NUCLEO] Da mo {self.port} @{self.baud}", flush=True)
            except Exception as e:
                if not self._warned:
                    self._warned = True
                    ports = sorted(glob.glob("/dev/serial/by-id/*")) + sorted(glob.glob("/dev/ttyACM*")) + \
                        sorted(glob.glob("/dev/ttyUSB*"))
                    print(f"[NUCLEO] Chua mo duoc {self.port}: {e}. Cong dang co: {ports or 'khong co'}. "
                          "Se thu lai moi 1 s.", flush=True)

    def _close_locked(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            print("[NUCLEO] Mat ket noi serial, se mo lai", flush=True)

    def send(self, speed, steer_deg, direction):
        """Ghi 3 dòng #speed, #steer, #dir trong một lần ghi (không bị luồng khác chen vào giữa)."""
        data = encode_commands(speed, steer_deg, direction)
        with self.lock:
            if self.ser is None:
                return False
            try:
                self.ser.write(data)
            except Exception:
                self.errors += 1
                self._close_locked()
                return False
            self.sent += 1
            return True

    def send_stop(self):
        """Như ThreadWrite::send_stop của BrainUp: speed 0, steer 0, dir 0."""
        return self.send(0.0, 0.0, 0)

    def _read_loop(self):
        pending = b""
        while self._running:
            ser = self.ser
            if ser is None:
                time.sleep(0.05)
                pending = b""
                continue
            try:
                chunk = ser.read(ser.in_waiting or 1)
            except Exception:
                with self.lock:
                    if self.ser is ser:
                        self._close_locked()
                continue
            if not chunk:
                continue
            pending += chunk
            if b"\n" not in pending:
                if len(pending) > 4096:   # rác không có xuống dòng
                    pending = b""
                continue
            *lines, pending = pending.split(b"\n")
            for raw in lines:
                msg = parse_nucleo_line(raw.decode("ascii", errors="ignore"))
                if msg is None:
                    continue
                action, values = msg
                try:
                    if action == "hall" and values:
                        self.speed, self.speed_time = float(values[0]), time.monotonic()
                    elif action == "imu" and len(values) >= 6:
                        self.imu = [float(v) for v in values[:6]]
                    else:
                        self.other[action] = ";".join(values)
                except ValueError:
                    pass

    def close(self):
        self._running = False
        with self.lock:
            if self.ser is not None:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None


# ============================================= PC (TCP) =============================================
class PcLink:
    """TCP client tới main_master.py (TCP server). Mỗi lần có lệnh @vcb mới gọi on_command((speed, steer, dir))."""

    SILENT_RECONNECT_S = 2.0

    def __init__(self, host, port, on_command):
        self.host, self.port, self.on_command = host, port, on_command
        self.connected = False
        self._running = True
        self._thread = threading.Thread(target=self._run, name="pc-tcp", daemon=True)
        self._thread.start()

    @staticmethod
    def _keepalive(sock):
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        for name, value in (("TCP_KEEPIDLE", 1), ("TCP_KEEPINTVL", 1), ("TCP_KEEPCNT", 3)):
            if hasattr(socket, name):   # Linux
                sock.setsockopt(socket.IPPROTO_TCP, getattr(socket, name), value)

    def _run(self):
        warned = False
        while self._running:
            try:
                sock = socket.create_connection((self.host, self.port), timeout=1.0)
            except OSError as e:
                if not warned:
                    print(f"[PC] Chua ket noi duoc {self.host}:{self.port} ({e}). main_master.py da chay chua? "
                          "Dung IP PC chua? Tuong lua Windows cho Python chua? Se thu lai moi 1 s.", flush=True)
                    warned = True
                time.sleep(1.0)
                continue
            warned = False
            self._keepalive(sock)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(0.2)
            self.connected = True
            print(f"[PC] Da ket noi {self.host}:{self.port}", flush=True)
            buf = b""
            last_data = time.monotonic()
            try:
                while self._running:
                    try:
                        data = sock.recv(4096)
                    except socket.timeout:
                        if time.monotonic() - last_data > self.SILENT_RECONNECT_S:
                            print("[PC] PC im lang qua lau, noi lai", flush=True)
                            break
                        continue
                    if not data:
                        break   # PC đóng kết nối
                    last_data = time.monotonic()
                    buf += data
                    if b"\n" not in buf:
                        if len(buf) > 65536:
                            buf = b""
                        continue
                    *lines, buf = buf.split(b"\n")
                    newest = None
                    for raw in lines:
                        cmd = parse_vcb(raw.decode("ascii", errors="ignore"))
                        if cmd is not None:
                            newest = cmd
                    if newest is not None:
                        self.on_command(newest)
            except OSError:
                pass
            finally:
                sock.close()
                self.connected = False
                print("[PC] Mat ket noi PC, dang noi lai...", flush=True)
            time.sleep(0.2)

    def stop(self):
        self._running = False


# ============================================ CAMERA ============================================
def opencv_has_gstreamer(cv2):
    """OpenCV có biên dịch kèm GStreamer không (bản apt python3-opencv: có; bản pip opencv-python: KHÔNG)."""
    for line in cv2.getBuildInformation().splitlines():
        if line.strip().startswith("GStreamer:"):
            return "YES" in line
    return False


def open_camera(pipeline):
    """Mở camera bằng đúng pipeline GStreamer cho trước. Trả về cv2.VideoCapture đã mở, hoặc nêu RuntimeError rõ lý do."""
    import cv2
    if not opencv_has_gstreamer(cv2):
        raise RuntimeError(
            f"OpenCV {cv2.__version__} ({cv2.__file__}) KHONG co GStreamer. Dung ban apt: "
            "'pip uninstall opencv-python opencv-contrib-python' (neu da cai bang pip) roi "
            "'sudo apt install python3-opencv'. Khong chay trong venv khong co --system-site-packages.")
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        test = pipeline.split("! appsink")[0] + "! fakesink"
        raise RuntimeError(
            "GStreamer khong mo duoc pipeline. Kiem tra lan luot tren Pi: (1) rpicam-hello --list-cameras phai thay "
            "imx219 va dung ten camera trong LANE_/ROAD_CAMERA_NAME; (2) tat chuong trinh khac dang dung camera; "
            f"(3) chay thu pipeline: gst-launch-1.0 -v {test.replace(', ', ',')}")
    return cap


class CameraStreamer(threading.Thread):
    """Đọc một camera bằng pipeline GStreamer, nén JPEG, gửi MỖI ẢNH MỘT GÓI UDP tới PC (như cam_udp_sender.cpp):
    socket không chặn, SO_SNDBUF 1 MB, bind cổng nguồn riêng, buffer đầy thì bỏ ảnh."""

    def __init__(self, name, pipeline, frame_size, pc_ip, pc_port, source_port, quality=JPEG_QUALITY):
        super().__init__(name=f"camera-{name}", daemon=True)
        self.cam_name = name
        self.pipeline, self.frame_size = pipeline, frame_size
        self.addr, self.source_port, self.quality = (pc_ip, pc_port), source_port, quality
        self.sent = 0
        self.dropped = 0
        self.last_size = 0
        self.error = ""
        self._running = True

    def _log(self, text):
        print(f"[CAM {self.cam_name}] {text}", flush=True)

    def run(self):
        import cv2
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)
        try:
            sock.bind(("0.0.0.0", self.source_port))
        except OSError as e:
            self.error = f"khong bind duoc cong UDP {self.source_port} ({e}): pi_bridge khac dang chay?"
            self._log(self.error)
            return
        sock.setblocking(False)
        while self._running:
            try:
                cap = open_camera(self.pipeline)
            except RuntimeError as e:
                if self.error != str(e):
                    self.error = str(e)
                    self._log(f"{e} Thu lai moi 2 s.")
                time.sleep(2.0)
                continue
            self.error = ""
            self._log(f"Da mo camera -> {self.frame_size[0]}x{self.frame_size[1]} JPEG q{self.quality}, gui tu cong "
                      f"{self.source_port} toi {self.addr[0]}:{self.addr[1]}")
            fails = 0
            try:
                while self._running:
                    ok, frame = cap.read()
                    if not ok or frame is None:
                        fails += 1
                        if fails > 30:
                            raise RuntimeError("camera ngung tra anh")
                        time.sleep(0.01)
                        continue
                    fails = 0
                    if (frame.shape[1], frame.shape[0]) != self.frame_size:   # pipeline đã đúng cỡ; phòng hờ
                        frame = cv2.resize(frame, self.frame_size, interpolation=cv2.INTER_AREA)
                    self._send(sock, frame, cv2)
            except Exception as e:
                self.error = str(e)
                self._log(f"{e}, mo lai camera")
            finally:
                cap.release()
        sock.close()

    def _send(self, sock, frame, cv2):
        q = self.quality
        while True:
            ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, q])
            if ok and len(jpg) <= MAX_UDP_PAYLOAD:
                break
            q -= 10   # ảnh quá lớn cho một gói UDP: hạ chất lượng riêng ảnh này
            if q < 30:
                self.dropped += 1
                return
        try:
            sock.sendto(jpg.tobytes(), self.addr)
            self.sent += 1
            self.last_size = len(jpg)
        except (BlockingIOError, OSError):
            self.dropped += 1   # buffer gửi đầy / mạng lỗi: bỏ ảnh, UDP không đảm bảo

    def stop(self):
        self._running = False


def camera_test(name, pipeline, frames=60):
    """Mở một camera, đọc 'frames' ảnh, in fps và kích thước, lưu ảnh cuối ra camera_test_<name>.jpg."""
    import cv2
    print(f"[CAM TEST {name}] OpenCV {cv2.__version__}, GStreamer: {'CO' if opencv_has_gstreamer(cv2) else 'KHONG'}")
    print(f"[CAM TEST {name}] Pipeline: {pipeline}")
    try:
        cap = open_camera(pipeline)
    except RuntimeError as e:
        print(f"[CAM TEST {name}] LOI: {e}")
        return 1
    t0, n, frame = time.monotonic(), 0, None
    while n < frames and time.monotonic() - t0 < 10.0:
        ok, f = cap.read()
        if ok and f is not None:
            frame, n = f, n + 1
    cap.release()
    if frame is None:
        print(f"[CAM TEST {name}] LOI: mo duoc pipeline nhung khong doc duoc anh nao")
        return 1
    dt = time.monotonic() - t0
    out = f"camera_test_{name}.jpg"
    cv2.imwrite(out, frame)
    print(f"[CAM TEST {name}] OK: {n} anh trong {dt:.1f} s ({n / dt:.1f} anh/s), co anh {frame.shape[1]}x"
          f"{frame.shape[0]}, da luu {out} (kiem tra dung camera, dung chieu, khong vien den)")
    return 0


# ============================================ CẦU NỐI ============================================
class Bridge:
    CMD_TIMEOUT_S = 0.3       # lệnh PC cũ hơn mức này -> gửi lệnh dừng
    STOP_PERIOD_S = 0.05      # gửi lệnh dừng 20 lần/s
    STATUS_PERIOD_S = 1.0

    def __init__(self, nucleo):
        self.nucleo = nucleo
        self.lock = threading.Lock()
        self.last_cmd = None
        self.last_cmd_time = -1e9
        self.cmd_count = 0

    def on_pc_command(self, cmd):
        speed, steer, direction = cmd
        with self.lock:   # cùng khoá với watchdog: không chen lệnh dừng vào giữa lúc lệnh vừa tới
            self.last_cmd, self.last_cmd_time = cmd, time.monotonic()
            self.cmd_count += 1
            self.nucleo.send(speed, steer, direction)

    def watchdog(self, now):
        with self.lock:
            if now - self.last_cmd_time > self.CMD_TIMEOUT_S:
                self.nucleo.send_stop()
                return True
        return False


def main():
    ap = argparse.ArgumentParser(description="Cau noi PC (main_master) <-> Nucleo (firmware anh An) + gui 2 camera, "
                                             "chay tren Raspberry Pi 5. Mac dinh lay tu hang so dau file.")
    ap.add_argument("--pc", default=PC_IP, help=f"IP may PC chay main_master.py (mac dinh {PC_IP})")
    ap.add_argument("--serial", default=NUCLEO_SERIAL, help=f"cong USB cua Nucleo (mac dinh {NUCLEO_SERIAL})")
    ap.add_argument("--lane-camera", default=LANE_CAMERA_SOURCE,
                    help="pipeline camera bird's-eye (mac dinh LANE_CAMERA_SOURCE), 'none' de tat")
    ap.add_argument("--road-camera", default=ROAD_CAMERA_SOURCE,
                    help="pipeline camera nhin duong (mac dinh ROAD_CAMERA_SOURCE), 'none' de tat")
    ap.add_argument("--camera-test", choices=["lane", "road"], help="chi thu mot camera: doc 60 anh, luu anh, thoat")
    args = ap.parse_args()

    if args.camera_test:
        raise SystemExit(camera_test(args.camera_test, args.lane_camera if args.camera_test == "lane"
                                     else args.road_camera))

    nucleo = NucleoLink(args.serial, NUCLEO_BAUD)
    bridge = Bridge(nucleo)
    nucleo.maintain()
    pc = PcLink(args.pc, PC_TCP_PORT, bridge.on_pc_command)
    cams = []
    for name, pipeline, size, port, src in (("lane", args.lane_camera, LANE_FRAME_SIZE, LANE_VIDEO_PORT, LANE_SOURCE_PORT),
                                            ("road", args.road_camera, ROAD_FRAME_SIZE, ROAD_VIDEO_PORT, ROAD_SOURCE_PORT)):
        if pipeline.strip().lower() != "none":
            cam = CameraStreamer(name, pipeline, size, args.pc, port, src)
            cam.start()
            cams.append(cam)

    print(f"[PI] Cau noi chay: PC {args.pc} (TCP {PC_TCP_PORT}; anh lane -> UDP {LANE_VIDEO_PORT}, road -> UDP "
          f"{ROAD_VIDEO_PORT}) <-> Nucleo {args.serial}. Ctrl+C de dung.", flush=True)
    last_stop = last_status = time.monotonic()
    last_counts = [bridge.cmd_count, nucleo.sent] + [c.sent for c in cams]
    try:
        while True:
            now = time.monotonic()
            nucleo.maintain()
            if now - last_stop >= Bridge.STOP_PERIOD_S:
                last_stop = now
                bridge.watchdog(now)
            if now - last_status >= Bridge.STATUS_PERIOD_S:
                dt = now - last_status
                last_status = now
                counts = [bridge.cmd_count, nucleo.sent] + [c.sent for c in cams]
                rates = [(a - b) / dt for a, b in zip(counts, last_counts)]
                last_counts = counts
                print_status(pc, nucleo, bridge, cams, rates, now)
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n[PI] Dung: gui lenh dung cho Nucleo", flush=True)
    finally:
        pc.stop()
        for cam in cams:
            cam.stop()
        for _ in range(5):
            nucleo.send_stop()
            time.sleep(0.02)
        nucleo.close()


def print_status(pc, nucleo, bridge, cams, rates, now):
    cmd_rate, sent_rate, cam_rates = rates[0], rates[1], rates[2:]
    if bridge.last_cmd is not None and now - bridge.last_cmd_time <= Bridge.CMD_TIMEOUT_S:
        s, st, d = bridge.last_cmd
        pc_txt = f"lenh {cmd_rate:4.0f}/s, {s:.2f} m/s, lai {st:+.1f} do, dir {d:+d}"
    else:
        pc_txt = "KHONG CO LENH -> gui DUNG"
    pc_txt = ("PC OK " if pc.connected else "PC MAT ") + pc_txt
    if nucleo.ser is None:
        nuc_txt = "NUCLEO CHUA MO CONG"
    else:
        nuc_txt = f"Nucleo gui {sent_rate:.0f} bo lenh/s"
        if nucleo.speed is not None and now - nucleo.speed_time <= 0.5:
            nuc_txt += f" | do {nucleo.speed:+.2f} m/s"
        else:
            nuc_txt += " | CHUA NHAN @hall"
        if nucleo.imu is not None:
            nuc_txt += f" | yaw {nucleo.imu[2]:.1f}"
    cam_txt = "".join(f" | cam {c.cam_name} " + (f"{r:.0f} anh/s, {c.last_size // 1024} KB" if not c.error else "LOI")
                      for c, r in zip(cams, cam_rates))
    print(f"[PI] {pc_txt} | {nuc_txt}{cam_txt}", flush=True)


if __name__ == "__main__":
    main()
