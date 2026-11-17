#!/usr/bin/env python3
"""
pi_bridge.py - chạy trên Raspberry Pi 5: nối PC (main_master.py) với STM32 NUCLEO-F401RE (stm32_vehicle) và gửi camera.

    PC main_master.py --TCP 6000, "@vcb:speed;steer;dir;;\\r\\n" 50 lần/s--> Pi 5 --USB (ST-LINK), gói 10 byte--> STM32
    Camera CSI --GStreamer LANE_CAMERA_SOURCE--> Pi 5 --UDP 7000, mỗi gói MỘT ảnh JPEG 640x480--> PC (main_master)
    STM32 --"@hall:..." 20 lần/s--> Pi 5 (in trạng thái mỗi giây)

IP, cổng, cổng serial và pipeline camera khai báo bằng HẰNG SỐ ngay bên dưới, giống kLaneCameraSource / kUdpClientIp /
kUdpClientPort / kUdpPort của BrainUp (cam_udp_sender.cpp). Muốn đổi thì sửa hằng số. Tham số dòng lệnh chỉ để ghi
đè tạm khi thử.

- Lệnh @vcb chuyển xuống STM32 NGAY khi nhận (không chờ chu kỳ); nhiều lệnh dồn trong một lần đọc thì chỉ gửi lệnh
  MỚI NHẤT (giống wheel_pedal_bridge.py).
- Quá CMD_TIMEOUT_S không có lệnh mới (PC tắt, Wi-Fi rớt): Pi tự gửi gói DỪNG 20 lần/s. Pi treo / rút cáp: STM32 tự
  dừng sau 250 ms (LINK_TIMEOUT_MS trong stm32_vehicle/Core/Inc/config.h).
- PC im lặng quá 2 s (kết nối chết sau khi rớt Wi-Fi): đóng và nối lại.

Cài trên Pi (Raspberry Pi OS Bookworm) - DÙNG OpenCV CỦA APT (có GStreamer), KHÔNG dùng opencv-python của pip:
    sudo apt install python3-opencv python3-serial gstreamer1.0-libcamera gstreamer1.0-tools \\
                     gstreamer1.0-plugins-base gstreamer1.0-plugins-good
Thử riêng camera (chụp 60 ảnh, đo fps, lưu camera_test.jpg rồi thoát):
    python3 pi_bridge.py --camera-test
Chạy:
    python3 pi_bridge.py

Chuỗi in ra console viết không dấu (quy ước của dự án).
"""

import argparse
import glob
import math
import socket
import struct
import threading
import time

# ======================================= CẤU HÌNH (sửa ở đây) =======================================
PC_IP = "192.168.1.122"        # IP máy PC chạy main_master.py (trên PC gõ ipconfig để xem; đổi Wi-Fi có thể đổi IP)
PC_TCP_PORT = 6000             # main_master: TCP server, xe kết nối vào để nhận lệnh @vcb
PC_VIDEO_PORT = 7000           # main_master: cổng UDP nhận ảnh (kUdpClientPort)
VIDEO_SOURCE_PORT = 9000       # cổng nguồn UDP trên Pi (kUdpPort). Hai bản pi_bridge cùng chạy sẽ báo "dang dung"

# NUCLEO cắm vào Pi bằng cáp mini-USB của ST-LINK -> /dev/ttyACM0. Lidar RPLIDAR là /dev/ttyUSB0 nên không lẫn.
# Chắc chắn hơn: ls /dev/serial/by-id/ rồi dán đường dẫn usb-STMicroelectronics_STM32_STLink_...-if02 vào đây.
STM32_SERIAL = "/dev/ttyACM0"
STM32_BAUD = 115200            # = LINK_BAUD trong stm32_vehicle/Core/Inc/config.h
MAX_SPEED_MPS = 1.5            # kẹp tốc độ gửi xuống STM32

# Camera bird's-eye: IMX219-160 cắm cổng CSI của Pi 5. Giữ cách làm của kLaneCameraSource trên Jetson:
#   Jetson:  nvarguscamerasrc 1920x1080 30fps ! nvvidconv flip-method=2 -> 640x480 BGRx ! videoconvert ! BGR ! appsink
#   Pi 5:    libcamerasrc     1920x1080 30fps ! videoscale -> 640x480 (kéo dãn như nvvidconv) ! videoflip xoay 180
#            ! videoconvert ! BGR ! appsink
# Ảnh gửi đi là ảnh THÔ chưa khử méo, PC tự khử méo. Camera/pipeline khác Jetson nên PHẢI calib lại trên Pi.
# Có 2 camera: thêm camera-name=<tên> sau libcamerasrc (tên lấy từ: rpicam-hello --list-cameras).
LANE_CAMERA_SOURCE = (
    "libcamerasrc ! "
    "video/x-raw, width=1920, height=1080, framerate=30/1 ! "
    "videoscale add-borders=false ! "
    "video/x-raw, width=640, height=480 ! "
    "videoflip method=rotate-180 ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! "
    "appsink drop=true max-buffers=1 sync=false"
)
FRAME_SIZE = (640, 480)        # ảnh gửi PC (GroundCalibration.width/height của main_master)
JPEG_QUALITY = 85              # kJpegQuality của BrainUp
MAX_UDP_PAYLOAD = 65507        # một gói UDP/IPv4

# ======================= GÓI LỆNH Pi -> STM32 (khớp stm32_vehicle/Core/Inc/protocol.h) =======================
HEADER = b"\xAA\x55"
FLAG_ENABLE = 0x01
FLAG_RAW = 0x02
_BODY = struct.Struct("<2shHbBB")   # header, góc lái x100, tốc độ x1000, chiều, cờ, số thứ tự  (9 byte) + XOR


def xor_checksum(data):
    c = 0
    for b in data:
        c ^= b
    return c


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def encode_command(steer_deg, speed, direction, enable, seq, raw=False):
    """Gói 10 byte: góc lái (độ, + = phải), tốc độ (m/s >= 0; raw: u 0..1), chiều (1/-1/0), cờ, số thứ tự."""
    steer = int(round(clamp(steer_deg, -90.0, 90.0) * 100))
    spd = int(round(clamp(speed, 0.0, 1.0 if raw else 10.0) * 1000))
    d = 1 if direction > 0 else (-1 if direction < 0 else 0)
    flags = (FLAG_ENABLE if enable else 0) | (FLAG_RAW if raw else 0)
    body = _BODY.pack(HEADER, steer, spd, d, flags, seq & 0xFF)
    return body + bytes([xor_checksum(body)])


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


HALL_STATES = {0: "KHOI DONG", 1: "CHAY", 2: "PI BAO DUNG", 3: "MAT LENH", 4: "LOI ENCODER KHONG QUAY",
               5: "LOI ENCODER NGUOC CHIEU"}


def parse_hall(line):
    """'@hall:v;counts;act;u;signal;state;ok;bad;lost;;' của STM32 -> dict, hoặc None."""
    line = line.strip()
    if not line.startswith("@hall:"):
        return None
    f = line[6:].split(";")
    if len(f) < 9:
        return None
    try:
        return {"v": float(f[0]), "counts": int(f[1]), "action": f[2], "u": float(f[3]), "signal": float(f[4]),
                "state": int(f[5]), "ok": int(f[6]), "bad": int(f[7]), "lost": int(f[8])}
    except ValueError:
        return None


# ========================================= STM32 (USB serial) =========================================
class McuLink:
    """Gửi gói lệnh xuống STM32, đọc dòng @hall / @info. Rút cáp / STM32 khởi động lại thì tự mở lại cổng mỗi 1 s.
    port: /dev/ttyACM0, /dev/serial/by-id/..., COM5 (Windows) hoặc URL pyserial (socket://host:port để thử)."""

    REOPEN_S = 1.0

    def __init__(self, port, baud=STM32_BAUD):
        self.port, self.baud = port, baud
        self.ser = None
        self.lock = threading.Lock()
        self.seq = 0
        self.sent = 0
        self.errors = 0
        self.telemetry = None
        self.telemetry_time = 0.0
        self.info = ""
        self._next_open = 0.0
        self._warned = False
        self._running = True
        self._reader = threading.Thread(target=self._read_loop, name="stm32-read", daemon=True)
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
                print(f"[STM32] Da mo {self.port} @{self.baud}", flush=True)
            except Exception as e:
                if not self._warned:
                    self._warned = True
                    ports = sorted(glob.glob("/dev/serial/by-id/*")) + sorted(glob.glob("/dev/ttyACM*")) + \
                        sorted(glob.glob("/dev/ttyUSB*"))
                    print(f"[STM32] Chua mo duoc {self.port}: {e}. Cong dang co: {ports or 'khong co'}. "
                          "Se thu lai moi 1 s.", flush=True)

    def _close_locked(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            print("[STM32] Mat ket noi serial, se mo lai", flush=True)

    def send_command(self, steer_deg, speed, direction, enable, raw=False):
        with self.lock:
            if self.ser is None:
                return False
            pkt = encode_command(steer_deg, speed, direction, enable, self.seq, raw)
            try:
                self.ser.write(pkt)
            except Exception:
                self.errors += 1
                self._close_locked()
                return False
            self.seq = (self.seq + 1) & 0xFF
            self.sent += 1
            return True

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
                text = raw.decode("ascii", errors="ignore")
                t = parse_hall(text)
                if t is not None:
                    self.telemetry, self.telemetry_time = t, time.monotonic()
                elif text.startswith("@info:"):
                    self.info = text.strip()
                    print(f"[STM32] {self.info}", flush=True)

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
            "imx219; (2) tat chuong trinh khac dang dung camera; (3) chay thu pipeline: "
            f"gst-launch-1.0 -v {test.replace(', ', ',')}")
    return cap


class CameraStreamer(threading.Thread):
    """Đọc camera bằng LANE_CAMERA_SOURCE, nén JPEG, gửi MỖI ẢNH MỘT GÓI UDP tới PC (như cam_udp_sender.cpp):
    socket không chặn, SO_SNDBUF 1 MB, bind cổng nguồn VIDEO_SOURCE_PORT, buffer đầy thì bỏ ảnh."""

    def __init__(self, pipeline, pc_ip, pc_port, source_port, quality):
        super().__init__(name="camera", daemon=True)
        self.pipeline, self.addr, self.source_port, self.quality = pipeline, (pc_ip, pc_port), source_port, quality
        self.sent = 0
        self.dropped = 0
        self.last_size = 0
        self.error = ""
        self._running = True

    def run(self):
        import cv2
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)
        try:
            sock.bind(("0.0.0.0", self.source_port))
        except OSError as e:
            self.error = f"khong bind duoc cong UDP {self.source_port} ({e}): pi_bridge khac dang chay?"
            print(f"[CAM] {self.error}", flush=True)
            return
        sock.setblocking(False)
        while self._running:
            try:
                cap = open_camera(self.pipeline)
            except RuntimeError as e:
                if self.error != str(e):
                    self.error = str(e)
                    print(f"[CAM] {e} Thu lai moi 2 s.", flush=True)
                time.sleep(2.0)
                continue
            self.error = ""
            print(f"[CAM] Da mo camera -> {FRAME_SIZE[0]}x{FRAME_SIZE[1]} JPEG q{self.quality}, gui tu cong "
                  f"{self.source_port} toi {self.addr[0]}:{self.addr[1]}", flush=True)
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
                    if (frame.shape[1], frame.shape[0]) != FRAME_SIZE:   # pipeline đã đưa về 640x480; phòng hờ
                        frame = cv2.resize(frame, FRAME_SIZE, interpolation=cv2.INTER_AREA)
                    self._send(sock, frame, cv2)
            except Exception as e:
                self.error = str(e)
                print(f"[CAM] {e}, mo lai camera", flush=True)
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


def camera_test(pipeline, frames=60):
    """Mở camera, đọc 'frames' ảnh, in fps và kích thước, lưu ảnh cuối ra camera_test.jpg."""
    import cv2
    print(f"[CAM TEST] OpenCV {cv2.__version__}, GStreamer: {'CO' if opencv_has_gstreamer(cv2) else 'KHONG'}")
    print(f"[CAM TEST] Pipeline: {pipeline}")
    try:
        cap = open_camera(pipeline)
    except RuntimeError as e:
        print(f"[CAM TEST] LOI: {e}")
        return 1
    t0, n, frame = time.monotonic(), 0, None
    while n < frames and time.monotonic() - t0 < 10.0:
        ok, f = cap.read()
        if ok and f is not None:
            frame, n = f, n + 1
    cap.release()
    if frame is None:
        print("[CAM TEST] LOI: mo duoc pipeline nhung khong doc duoc anh nao")
        return 1
    dt = time.monotonic() - t0
    cv2.imwrite("camera_test.jpg", frame)
    print(f"[CAM TEST] OK: {n} anh trong {dt:.1f} s ({n / dt:.1f} anh/s), co anh {frame.shape[1]}x{frame.shape[0]}, "
          "da luu camera_test.jpg (kiem tra anh dung chieu, khong bi vien den)")
    return 0


# ============================================ CẦU NỐI ============================================
class Bridge:
    CMD_TIMEOUT_S = 0.3       # lệnh PC cũ hơn mức này -> gửi gói dừng
    STOP_PERIOD_S = 0.05      # gửi gói dừng 20 lần/s (STM32 thấy liên kết còn sống, đứng yên)
    STATUS_PERIOD_S = 1.0

    def __init__(self, mcu, max_speed):
        self.mcu = mcu
        self.max_speed = max_speed
        self.lock = threading.Lock()
        self.last_cmd = None
        self.last_cmd_time = -1e9
        self.cmd_count = 0
        self.last_steer = 0.0

    def on_pc_command(self, cmd):
        speed, steer, direction = cmd
        speed = min(speed, self.max_speed)
        with self.lock:   # cùng khoá với watchdog: không chen gói dừng vào giữa lúc lệnh vừa tới
            self.last_cmd, self.last_cmd_time = (speed, steer, direction), time.monotonic()
            self.cmd_count += 1
            self.last_steer = steer
            self.mcu.send_command(steer, speed, direction, enable=True)

    def watchdog(self, now):
        with self.lock:
            if now - self.last_cmd_time > self.CMD_TIMEOUT_S:
                self.mcu.send_command(self.last_steer, 0.0, 0, enable=False)
                return True
        return False


def main():
    ap = argparse.ArgumentParser(description="Cau noi PC (main_master) <-> STM32 + gui camera, chay tren Raspberry Pi 5. "
                                             "Mac dinh lay tu hang so dau file.")
    ap.add_argument("--pc", default=PC_IP, help=f"IP may PC chay main_master.py (mac dinh {PC_IP})")
    ap.add_argument("--serial", default=STM32_SERIAL, help=f"cong USB cua NUCLEO (mac dinh {STM32_SERIAL})")
    ap.add_argument("--camera", default=LANE_CAMERA_SOURCE,
                    help="pipeline GStreamer cua camera (mac dinh LANE_CAMERA_SOURCE), hoac 'none' de tat camera")
    ap.add_argument("--camera-test", action="store_true", help="chi thu camera: doc 60 anh, luu camera_test.jpg, thoat")
    args = ap.parse_args()

    if args.camera_test:
        raise SystemExit(camera_test(args.camera))

    mcu = McuLink(args.serial, STM32_BAUD)
    bridge = Bridge(mcu, MAX_SPEED_MPS)
    mcu.maintain()
    pc = PcLink(args.pc, PC_TCP_PORT, bridge.on_pc_command)
    cam = None
    if args.camera.strip().lower() != "none":
        cam = CameraStreamer(args.camera, args.pc, PC_VIDEO_PORT, VIDEO_SOURCE_PORT, JPEG_QUALITY)
        cam.start()

    print(f"[PI] Cau noi chay: PC {args.pc} (TCP {PC_TCP_PORT}, video UDP {PC_VIDEO_PORT}) <-> STM32 {args.serial}. "
          "Ctrl+C de dung.", flush=True)
    last_stop = last_status = time.monotonic()
    last_counts = (0, mcu.sent, cam.sent if cam else 0)
    try:
        while True:
            now = time.monotonic()
            mcu.maintain()
            if now - last_stop >= Bridge.STOP_PERIOD_S:
                last_stop = now
                bridge.watchdog(now)
            if now - last_status >= Bridge.STATUS_PERIOD_S:
                dt = now - last_status
                last_status = now
                counts = (bridge.cmd_count, mcu.sent, cam.sent if cam else 0)
                rates = [(a - b) / dt for a, b in zip(counts, last_counts)]
                last_counts = counts
                print_status(pc, mcu, bridge, cam, rates, now)
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n[PI] Dung: gui lenh dung cho STM32", flush=True)
    finally:
        pc.stop()
        if cam:
            cam.stop()
        for _ in range(5):
            mcu.send_command(bridge.last_steer, 0.0, 0, enable=False)
            time.sleep(0.02)
        mcu.close()


def print_status(pc, mcu, bridge, cam, rates, now):
    cmd_rate, sent_rate, cam_rate = rates
    if bridge.last_cmd is not None and now - bridge.last_cmd_time <= Bridge.CMD_TIMEOUT_S:
        s, st, d = bridge.last_cmd
        pc_txt = f"lenh {cmd_rate:4.0f}/s, {s:.2f} m/s, lai {st:+.1f} do, dir {d:+d}"
    else:
        pc_txt = "KHONG CO LENH -> gui DUNG"
    pc_txt = ("PC OK " if pc.connected else "PC MAT ") + pc_txt
    t = mcu.telemetry
    if mcu.ser is None:
        mcu_txt = "STM32 CHUA MO CONG"
    elif t is None or now - mcu.telemetry_time > 0.5:
        mcu_txt = f"STM32 gui {sent_rate:.0f} goi/s, CHUA NHAN @hall (da nap stm32_vehicle chua?)"
    else:
        mcu_txt = (f"STM32 gui {sent_rate:.0f} goi/s | do {t['v']:+.2f} m/s, xung {t['counts']}, {t['action']} "
                   f"u {t['u']:.2f} ({t['signal']:.0f}), {HALL_STATES.get(t['state'], t['state'])}, "
                   f"goi ok/loi/mat {t['ok']}/{t['bad']}/{t['lost']}")
    cam_txt = "" if cam is None else (f" | cam {cam_rate:.0f} anh/s, {cam.last_size // 1024} KB" if not cam.error
                                      else " | cam LOI (xem dong [CAM])")
    print(f"[PI] {pc_txt} | {mcu_txt}{cam_txt}", flush=True)


if __name__ == "__main__":
    main()
