#!/usr/bin/env python3
"""
pi_bridge.py - chạy trên Raspberry Pi 5 (thay Jetson): nối PC (main_master.py) với ESP32 (esp32_vehicle).

    PC main_master.py --TCP 6000, "@vcb:speed;steer;dir;;\\r\\n" 50 lần/s--> Pi 5 --USB, gói 10 byte--> ESP32
    Camera bird's-eye --UDP 7000, mỗi gói một ảnh JPEG 640x480 (như cam_udp_sender.cpp)--> PC
    ESP32 --"@hall:..." 20 lần/s--> Pi 5 (in trạng thái)

- Lệnh @vcb chuyển xuống ESP32 NGAY khi nhận (không chờ chu kỳ); nhiều lệnh dồn trong một lần đọc thì chỉ gửi lệnh
  MỚI NHẤT (giống wheel_pedal_bridge.py).
- Quá CMD_TIMEOUT_S không có lệnh mới (PC tắt, Wi-Fi rớt): Pi tự gửi gói DỪNG 20 lần/s. Pi treo / rút cáp: ESP32 tự
  dừng sau 250 ms (LINK_TIMEOUT_MS trong esp32_vehicle/config.h).
- PC im lặng quá PC_SILENT_RECONNECT_S (kết nối chết sau khi rớt Wi-Fi): đóng và nối lại. main_master luôn dùng kết
  nối mới nhất nên nối lại không cần chờ.

Cài trên Pi (Raspberry Pi OS Bookworm):
    sudo apt install python3-opencv python3-serial python3-picamera2
    sudo nmcli connection modify "<ten wifi>" 802-11-wireless.powersave 2   # tắt tiết kiệm điện Wi-Fi (gây trễ 100-300 ms)
Chạy:
    python3 pi_bridge.py --pc 192.168.1.100 --serial /dev/serial/by-id/<ten ESP32>
Thử không có camera / không có ESP32 (ví dụ trên PC):
    python3 pi_bridge.py --pc 127.0.0.1 --camera anh.jpg --serial socket://127.0.0.1:9999

Chuỗi in ra console viết không dấu (quy ước của dự án).
"""

import argparse
import glob
import math
import os
import socket
import struct
import threading
import time

# ======================= GÓI LỆNH Pi -> ESP32 (khớp esp32_vehicle/protocol.h) =======================
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
    """'@hall:v;counts;act;u;signal;state;ok;bad;lost;;' của ESP32 -> dict, hoặc None."""
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


# ========================================= ESP32 (USB serial) =========================================
class Esp32Link:
    """Gửi gói lệnh xuống ESP32, đọc dòng @hall. Rút cáp / ESP32 khởi động lại thì tự mở lại cổng mỗi 1 s.
    port: /dev/ttyUSB0, /dev/serial/by-id/..., COM5 (Windows) hoặc URL pyserial (socket://host:port để thử)."""

    REOPEN_S = 1.0

    def __init__(self, port, baud=115200):
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
        self._reader = threading.Thread(target=self._read_loop, name="esp32-read", daemon=True)
        self._reader.start()

    def _open(self):
        import serial  # sudo apt install python3-serial
        if "://" in self.port:
            ser = serial.serial_for_url(self.port, baudrate=self.baud, timeout=0.05, write_timeout=0.05)
        else:
            ser = serial.Serial()
            ser.port, ser.baudrate, ser.timeout, ser.write_timeout = self.port, self.baud, 0.05, 0.05
            # DTR/RTS tắt trước khi mở: mạch tự nạp của ESP32 DevKit không bị kéo reset / vào chế độ nạp
            ser.dtr = False
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
                print(f"[ESP32] Da mo {self.port} @{self.baud}", flush=True)
            except Exception as e:
                if not self._warned:
                    self._warned = True
                    ports = sorted(glob.glob("/dev/serial/by-id/*")) + sorted(glob.glob("/dev/ttyUSB*")) + \
                        sorted(glob.glob("/dev/ttyACM*"))
                    print(f"[ESP32] Chua mo duoc {self.port}: {e}. Cong dang co: {ports or 'khong co'}. "
                          "Se thu lai moi 1 s.", flush=True)

    def _close_locked(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            print("[ESP32] Mat ket noi serial, se mo lai", flush=True)

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
                if len(pending) > 4096:   # rác không có xuống dòng (log khởi động của ESP32)
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
                    print(f"[ESP32] {self.info}", flush=True)

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
                          "Se thu lai moi 1 s.", flush=True)
                    warned = True
                time.sleep(1.0)
                continue
            warned = False
            self._keepalive(sock)
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
class CameraStreamer(threading.Thread):
    """Đọc camera, đưa về 640x480, nén JPEG, gửi MỖI ẢNH MỘT GÓI UDP tới PC (định dạng main_master.FrameReceiver).

    source:
      picam             camera CSI của Pi 5 qua Picamera2 (mặc định)
      gst:<pipeline>    OpenCV + GStreamer, ví dụ gst:libcamerasrc ! video/x-raw,width=1280,height=720 ! videoconvert ! appsink
      0, 1, ...         camera USB (V4L2)
      <file>            ảnh hoặc video để thử (lặp lại mãi, không xoay)
    Ảnh camera được chụp 16:9 rồi KÉO DÃN về 640x480 như pipeline Jetson cũ (nvvidconv 1920x1080 -> 640x480) và xoay
    180 độ (flip-method=2), để gần với bộ calib cũ nhất. Camera/Pi khác Jetson nên vẫn PHẢI calib lại.
    """

    OUT_SIZE = (640, 480)
    MAX_UDP = 65507

    def __init__(self, source, host, port, quality=80, fps=30.0, rotate180=True, capture_size=(1280, 720),
                 sensor_size=(1920, 1080), camera_index=0):
        super().__init__(name="camera", daemon=True)
        self.source, self.addr, self.quality, self.fps = source, (host, port), quality, fps
        self.rotate180, self.capture_size, self.sensor_size, self.camera_index = \
            rotate180, capture_size, sensor_size, camera_index
        self.sent = 0
        self.dropped = 0
        self.last_size = 0
        self.error = ""
        self._running = True

    # ---------- nguồn ảnh: trả về (hàm đọc -> ảnh BGR hoặc None, hàm đóng, có cần xoay bằng OpenCV) ----------
    def _open_picam(self):
        from picamera2 import Picamera2  # sudo apt install python3-picamera2
        from libcamera import Transform
        cam = Picamera2(self.camera_index)
        kw = dict(main={"size": self.capture_size, "format": "RGB888"},   # RGB888 của Picamera2 = thứ tự BGR
                  transform=Transform(hflip=1, vflip=1) if self.rotate180 else Transform(),
                  controls={"FrameRate": self.fps}, buffer_count=3)
        try:
            cfg = cam.create_video_configuration(sensor={"output_size": self.sensor_size}, **kw)
        except Exception:   # Picamera2 cũ không có tham số sensor
            cfg = cam.create_video_configuration(raw={"size": self.sensor_size}, **kw)
        cam.configure(cfg)
        cam.start()
        return (lambda: cam.capture_array("main")), (lambda: (cam.stop(), cam.close())), False

    def _open_cv(self):
        import cv2
        src = self.source
        if src.startswith("gst:"):
            cap = cv2.VideoCapture(src[4:], cv2.CAP_GSTREAMER)
        elif src.isdigit():
            cap = cv2.VideoCapture(int(src))
        elif os.path.splitext(src)[1].lower() in (".jpg", ".jpeg", ".png", ".bmp"):
            img = cv2.imread(src)
            if img is None:
                raise RuntimeError(f"khong doc duoc anh {src}")
            return (lambda: img), (lambda: None), False
        else:   # video: phát lặp lại
            cap = cv2.VideoCapture(src)
            if not cap.isOpened():
                raise RuntimeError(f"khong mo duoc {src}")

            def read_loop():
                ok, frame = cap.read()
                if not ok:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ok, frame = cap.read()
                return frame if ok else None
            return read_loop, cap.release, False
        if not cap.isOpened():
            raise RuntimeError(f"khong mo duoc camera {src}")
        return (lambda: (lambda r: r[1] if r[0] else None)(cap.read())), cap.release, self.rotate180

    def run(self):
        import cv2
        is_file = self.source not in ("picam",) and not self.source.startswith("gst:") and not self.source.isdigit()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)
        sock.setblocking(False)
        period = 1.0 / self.fps
        while self._running:
            try:
                read, close, rotate = self._open_picam() if self.source == "picam" else self._open_cv()
                self.error = ""
                print(f"[CAM] Da mo camera '{self.source}', gui {self.OUT_SIZE[0]}x{self.OUT_SIZE[1]} JPEG "
                      f"q{self.quality} toi {self.addr[0]}:{self.addr[1]}", flush=True)
            except Exception as e:
                if self.error != str(e):
                    self.error = str(e)
                    print(f"[CAM] Loi mo camera '{self.source}': {e}. Thu lai moi 2 s.", flush=True)
                time.sleep(2.0)
                continue
            next_t = time.monotonic()
            fails = 0
            try:
                while self._running:
                    frame = read()
                    if frame is None:
                        fails += 1
                        if fails > 30:
                            raise RuntimeError("mat anh camera")
                        time.sleep(0.01)
                        continue
                    fails = 0
                    if frame.ndim == 3 and frame.shape[2] == 4:
                        frame = frame[:, :, :3]
                    if rotate:
                        frame = cv2.rotate(frame, cv2.ROTATE_180)
                    if (frame.shape[1], frame.shape[0]) != self.OUT_SIZE:
                        frame = cv2.resize(frame, self.OUT_SIZE, interpolation=cv2.INTER_AREA)
                    self._send(sock, frame, cv2)
                    if is_file:   # ảnh/video thử: giữ đúng fps
                        next_t += period
                        time.sleep(max(0.0, next_t - time.monotonic()))
            except Exception as e:
                print(f"[CAM] {e}, mo lai camera", flush=True)
            finally:
                try:
                    close()
                except Exception:
                    pass
        sock.close()

    def _send(self, sock, frame, cv2):
        q = self.quality
        while True:
            ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, q])
            if ok and len(jpg) <= self.MAX_UDP:
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


# ============================================ CẦU NỐI ============================================
class Bridge:
    CMD_TIMEOUT_S = 0.3       # lệnh PC cũ hơn mức này -> gửi gói dừng
    STOP_PERIOD_S = 0.05      # gửi gói dừng 20 lần/s (ESP32 thấy liên kết còn sống, đứng yên)
    STATUS_PERIOD_S = 1.0

    def __init__(self, esp, max_speed):
        self.esp = esp
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
            self.esp.send_command(steer, speed, direction, enable=True)

    def watchdog(self, now):
        with self.lock:
            if now - self.last_cmd_time > self.CMD_TIMEOUT_S:
                self.esp.send_command(self.last_steer, 0.0, 0, enable=False)
                return True
        return False


def main():
    ap = argparse.ArgumentParser(description="Cau noi PC (main_master) <-> ESP32 + gui camera, chay tren Raspberry Pi 5")
    ap.add_argument("--pc", default="192.168.1.100", help="IP may PC chay main_master.py")
    ap.add_argument("--tcp-port", type=int, default=6000, help="cong TCP lenh @vcb cua main_master")
    ap.add_argument("--video-port", type=int, default=7000, help="cong UDP nhan anh cua main_master")
    ap.add_argument("--serial", default="/dev/ttyUSB0",
                    help="cong USB cua ESP32 (nen dung /dev/serial/by-id/... vi lidar cung la ttyUSB)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--camera", default="picam", help="picam | gst:<pipeline> | 0 (USB) | file anh/video | none")
    ap.add_argument("--camera-index", type=int, default=0, help="so thu tu camera CSI cho picam (0 hoac 1)")
    ap.add_argument("--jpeg-quality", type=int, default=80)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--no-rotate", action="store_true", help="khong xoay anh 180 do")
    ap.add_argument("--max-speed", type=float, default=1.5, help="kep toc do gui xuong ESP32 (m/s)")
    args = ap.parse_args()

    esp = Esp32Link(args.serial, args.baud)
    bridge = Bridge(esp, args.max_speed)
    esp.maintain()
    pc = PcLink(args.pc, args.tcp_port, bridge.on_pc_command)
    cam = None
    if args.camera != "none":
        cam = CameraStreamer(args.camera, args.pc, args.video_port, args.jpeg_quality, args.fps,
                             rotate180=not args.no_rotate, camera_index=args.camera_index)
        cam.start()

    print(f"[PI] Cau noi chay: PC {args.pc} (TCP {args.tcp_port}, video UDP {args.video_port}) <-> ESP32 "
          f"{args.serial}. Ctrl+C de dung.", flush=True)
    last_stop = last_status = time.monotonic()
    last_counts = (0, esp.sent, cam.sent if cam else 0)
    try:
        while True:
            now = time.monotonic()
            esp.maintain()
            if now - last_stop >= Bridge.STOP_PERIOD_S:
                last_stop = now
                bridge.watchdog(now)
            if now - last_status >= Bridge.STATUS_PERIOD_S:
                dt = now - last_status
                last_status = now
                counts = (bridge.cmd_count, esp.sent, cam.sent if cam else 0)
                rates = [(a - b) / dt for a, b in zip(counts, last_counts)]
                last_counts = counts
                print_status(pc, esp, bridge, cam, rates, now)
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n[PI] Dung: gui lenh dung cho ESP32", flush=True)
    finally:
        pc.stop()
        if cam:
            cam.stop()
        for _ in range(5):
            esp.send_command(bridge.last_steer, 0.0, 0, enable=False)
            time.sleep(0.02)
        esp.close()


def print_status(pc, esp, bridge, cam, rates, now):
    cmd_rate, sent_rate, cam_rate = rates
    if bridge.last_cmd is not None and now - bridge.last_cmd_time <= Bridge.CMD_TIMEOUT_S:
        s, st, d = bridge.last_cmd
        pc_txt = f"lenh {cmd_rate:4.0f}/s, {s:.2f} m/s, lai {st:+.1f} do, dir {d:+d}"
    else:
        pc_txt = "KHONG CO LENH -> gui DUNG"
    pc_txt = ("PC OK " if pc.connected else "PC MAT ") + pc_txt
    t = esp.telemetry
    if esp.ser is None:
        esp_txt = "ESP32 CHUA MO CONG"
    elif t is None or now - esp.telemetry_time > 0.5:
        esp_txt = f"ESP32 gui {sent_rate:.0f} goi/s, CHUA NHAN @hall"
    else:
        esp_txt = (f"ESP32 gui {sent_rate:.0f} goi/s | do {t['v']:+.2f} m/s, xung {t['counts']}, {t['action']} "
                   f"u {t['u']:.2f} ({t['signal']:.0f}), {HALL_STATES.get(t['state'], t['state'])}, "
                   f"goi ok/loi/mat {t['ok']}/{t['bad']}/{t['lost']}")
    cam_txt = "" if cam is None else (f" | cam {cam_rate:.0f} anh/s, {cam.last_size // 1024} KB" if not cam.error
                                      else f" | cam LOI: {cam.error}")
    print(f"[PI] {pc_txt} | {esp_txt}{cam_txt}", flush=True)


if __name__ == "__main__":
    main()
