"""
He thong tong hop: tinh van toc + goc lai tu ban dap / vo lang (theo docs/RawVehicleController.cpp)
va gioi han toc do toi da khi camera phat hien vat can, sau do gui lenh sang Jetson Nano.

Quy uoc bien khoang cach (distance_xe_may, distance_nguoi_di_bo, distance_o_to):
    -1  : khong co vat the tren duong
    -2  : mat camera
    >=0 : khoang cach toi vat the (m)

Du lieu ban dap / vo lang doc tu Arduino (docs/ban_dap_PID.ino) qua Serial 115200,
moi dong co dang:  "encoder|ga|lui|phanh"   vd: "-42|63.5|0.0|0.0"

Chay thu mo phong:   python main.py --sim
Chay that:           python main.py --serial-port COM3 --jetson-ip 192.168.1.50 --jetson-port 5005
                     (Linux: --serial-port /dev/ttyUSB0 hoac /dev/ttyACM0)
"""

import argparse
import json
import math
import socket
import threading
import time
from dataclasses import dataclass

# ============================================================================
# 1. HANG SO (lay tu docs/LogicTypes.hpp)
# ============================================================================
STEER_CENTER = 0.0
STEER_MIN = -20.0
STEER_MAX = 20.0

# Encoder vo lang (khop voi encoderMin/encoderMax trong docs/ban_dap_PID.ino)
ENCODER_MIN = -360
ENCODER_MAX = 360

VEHICLE_MASS_KG = 4.0
MAX_DRIVE_FORCE_N = 4.0
MAX_BRAKE_FORCE_N = 3.0
ROLLING_RESISTANCE_COEFF = 0.02
AIR_DRAG_COEFF = 0.8
GRAVITY = 9.81

MAX_FORWARD_SPEED = 1.3   # m/s
MAX_REVERSE_SPEED = 0.8   # m/s
SPEED_DEADZONE = 0.15

DT_SEC = 0.02             # chu ky dieu khien 50 Hz (giong dtSec_ trong C++)

# ============================================================================
# 2. THAM SO GIOI HAN THEO VAT CAN
# ============================================================================
NO_OBJECT = -1
CAMERA_LOST = -2

# (khoang cach bat dau giam toc, khoang cach dung lai) don vi m
LIMIT_PROFILE = {
    "nguoi_di_bo": (4.0, 0.5),
    "xe_may": (3.0, 0.2),
    "o_to": (3.0, 0.2),
}

# Toc do tang lai toc do toi da khi vat the roi khung hinh (m/s moi giay).
# 0.2 -> mat ~6.5 s de tu 0 len lai 1.3 m/s.
SPEED_LIMIT_RECOVER_RATE = 0.2


def clampf(x, lo, hi):
    return max(lo, min(hi, x))


def apply_pedal_interlock(ga, lui, phanh):
    """
    Khong bao gio de chan ga va chan phanh cung duong: phanh uu tien.
    Neu phanh > 0 thi ga = 0 (va ga lui = 0, vi ban dap lui cung la chan ga khi chay lui).
    """
    if phanh > 0.0:
        ga = 0.0
        lui = 0.0
    return ga, lui, phanh


# ============================================================================
# 3. CAU TRUC DU LIEU (Input / Output giong LogicTypes.hpp)
# ============================================================================
@dataclass
class Input:
    encoder: int = 0           # vo lang, -360..360
    ga: float = 0.0            # ban dap ga 0..100 (%)
    lui: float = 0.0           # ban dap lui 0..100 (%)
    phanh: float = 0.0         # ban dap phanh 0..100 (%)
    driveDir: int = 0          # 1 tien, -1 lui, 0 N
    distance_xe_may: float = NO_OBJECT
    distance_nguoi_di_bo: float = NO_OBJECT
    distance_o_to: float = NO_OBJECT
    reset: int = 0


@dataclass
class Output:
    steerCmd: float = STEER_CENTER   # do
    speedCmd: float = 0.0            # m/s
    driveDirCmd: int = 0
    speedLimit: float = MAX_FORWARD_SPEED  # toc do toi da dang ap dung (de debug)
    limitReason: str = "none"


# ============================================================================
# 4. BO GIOI HAN TOC DO THEO VAT CAN
# ============================================================================
class ObstacleSpeedLimiter:
    """
    Tinh toc do toi da cho phep dua tren khoang cach toi vat can.

    Bien dang toc do theo kieu phanh deu (v = sqrt(2*a*(d - d_stop))):
      - d >= d_start          -> v_max = MAX_FORWARD_SPEED
      - d_stop < d < d_start  -> giam dan, cang gan cang cham
      - d <= d_stop           -> v_max = 0 (xe dung)
    Gia toc a duoc chon sao cho tai d_start dung bang MAX_FORWARD_SPEED:
      nguoi di bo: a = 1.3^2 / (2*3.5) ~ 0.24 m/s^2
      xe may/o to: a = 1.3^2 / (2*2.8) ~ 0.30 m/s^2
    Ca hai deu nho hon kha nang phanh cua xe (3 N / 4 kg = 0.75 m/s^2) nen xe theo kip.

    Khi het vat can (-1) hoac mat camera (-2): gioi han tang tu tu ve lai MAX_FORWARD_SPEED.
    Khi gioi han moi thap hon gioi han hien tai: ap dung ngay (an toan uu tien).
    """

    def __init__(self):
        self.current_limit = MAX_FORWARD_SPEED

    def reset(self):
        self.current_limit = MAX_FORWARD_SPEED

    @staticmethod
    def _limit_for(distance, d_start, d_stop):
        if distance < 0:            # -1 hoac -2: khong gioi han
            return MAX_FORWARD_SPEED
        if distance <= d_stop:
            return 0.0
        if distance >= d_start:
            return MAX_FORWARD_SPEED
        ratio = (distance - d_stop) / (d_start - d_stop)
        return MAX_FORWARD_SPEED * math.sqrt(ratio)

    def target_limit(self, inp: Input):
        """Tra ve (gioi han muc tieu, ly do) - lay gia tri nho nhat trong cac vat the."""
        candidates = [
            (self._limit_for(inp.distance_nguoi_di_bo, *LIMIT_PROFILE["nguoi_di_bo"]), "nguoi_di_bo"),
            (self._limit_for(inp.distance_xe_may, *LIMIT_PROFILE["xe_may"]), "xe_may"),
            (self._limit_for(inp.distance_o_to, *LIMIT_PROFILE["o_to"]), "o_to"),
        ]
        limit, reason = min(candidates, key=lambda c: c[0])
        if limit >= MAX_FORWARD_SPEED:
            all_d = (inp.distance_nguoi_di_bo, inp.distance_xe_may, inp.distance_o_to)
            reason = "camera_lost" if all(d == CAMERA_LOST for d in all_d) else "none"
        return limit, reason

    def update(self, inp: Input, dt=DT_SEC):
        target, reason = self.target_limit(inp)
        if target < self.current_limit:
            self.current_limit = target                       # giam ngay
        else:
            self.current_limit = min(target, self.current_limit + SPEED_LIMIT_RECOVER_RATE * dt)
            if self.current_limit < MAX_FORWARD_SPEED and reason in ("none", "camera_lost"):
                reason = "recovering"
        return self.current_limit, reason


# ============================================================================
# 5. BO DIEU KHIEN XE (port tu docs/RawVehicleController.cpp + gioi han vat can)
# ============================================================================
class RawVehicleController:
    def __init__(self):
        self.speed = 0.0
        self.dt = DT_SEC
        self.limiter = ObstacleSpeedLimiter()

    def reset(self):
        self.speed = 0.0
        self.limiter.reset()

    def update(self, inp: Input) -> Output:
        if inp.reset:
            self.reset()
        out = Output()
        self._update_steering(inp, out)
        self._update_vehicle_dynamics(inp, out)
        return out

    # Vo lang: encoder -360..360 -> goc lai -20..20 do (tuyen tinh, 0 -> 0 do)
    def _update_steering(self, inp: Input, out: Output):
        encoder = max(ENCODER_MIN, min(ENCODER_MAX, int(inp.encoder)))
        out.steerCmd = STEER_MIN + (encoder - ENCODER_MIN) * (STEER_MAX - STEER_MIN) / (ENCODER_MAX - ENCODER_MIN)

    def _update_vehicle_dynamics(self, inp: Input, out: Output):
        ga = clampf(max(0.0, inp.ga), 0.0, 100.0)
        lui = clampf(max(0.0, inp.lui), 0.0, 100.0)
        phanh = clampf(max(0.0, inp.phanh), 0.0, 100.0)

        if ga < 5.0:
            ga = 0.0
        if lui < 5.0:
            lui = 0.0
        if phanh < 5.0:
            phanh = 0.0

        ga, lui, phanh = apply_pedal_interlock(ga, lui, phanh)

        # Gioi han toc do theo vat can (camera dat phia truoc -> chi ap dung khi chay tien)
        speed_limit, reason = self.limiter.update(inp, self.dt)

        if inp.driveDir == 1:
            throttle = ga / 100.0
            max_speed = min(MAX_FORWARD_SPEED, speed_limit)
        elif inp.driveDir == -1:
            throttle = lui / 100.0
            max_speed = MAX_REVERSE_SPEED
            reason = "reverse_no_limit"
        else:
            throttle = 0.0
            max_speed = self.speed

        drive_force = MAX_DRIVE_FORCE_N * throttle
        brake_force = MAX_BRAKE_FORCE_N * (phanh / 100.0)

        # Phanh tu dong: neu xe dang nhanh hon gioi han -> bo ga, them luc phanh
        if inp.driveDir == 1 and self.speed > max_speed:
            drive_force = 0.0
            brake_force = max(brake_force, MAX_BRAKE_FORCE_N)

        rolling_force = 0.0
        if self.speed > SPEED_DEADZONE:
            rolling_force = ROLLING_RESISTANCE_COEFF * VEHICLE_MASS_KG * GRAVITY

        air_drag_force = AIR_DRAG_COEFF * self.speed * self.speed

        total_force = drive_force - brake_force - rolling_force - air_drag_force
        acceleration = total_force / VEHICLE_MASS_KG

        self.speed += acceleration * self.dt
        self.speed = clampf(self.speed, 0.0, max_speed)   # dam bao cung khong vuot gioi han

        out.speedCmd = self.speed
        out.driveDirCmd = inp.driveDir
        out.speedLimit = max_speed if inp.driveDir == 1 else MAX_REVERSE_SPEED
        out.limitReason = reason


# ============================================================================
# 6. GUI LENH SANG JETSON NANO (UDP + JSON)
# ============================================================================
class JetsonSender:
    """Gui goi JSON: {"steer": do, "speed": m/s, "dir": -1/0/1} qua UDP."""

    def __init__(self, ip, port):
        self.addr = (ip, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, out: Output):
        packet = {
            "steer": round(out.steerCmd, 3),
            "speed": round(out.speedCmd, 3),
            "dir": out.driveDirCmd,
        }
        self.sock.sendto(json.dumps(packet).encode("utf-8"), self.addr)

    def close(self):
        self.sock.close()


# ============================================================================
# 7. DOC DU LIEU DAU VAO
# ============================================================================
SERIAL_BAUD = 115200
SERIAL_TIMEOUT_SEC = 0.3   # qua thoi gian nay khong co du lieu -> coi nhu mat ket noi


class PedalSerialReader:
    """
    Doc Serial tu Arduino (docs/ban_dap_PID.ino) trong 1 thread rieng, luon giu goi moi nhat.
    Arduino da lam san: chuan hoa ban dap ve 0..100 %, deadzone 20 %, khong cho ga va lui cung luc,
    gioi han encoder trong [-360, 360].
    """

    def __init__(self, port, baud=SERIAL_BAUD):
        import serial  # pip install pyserial

        self.ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2.0)                 # Arduino tu reset khi mo cong Serial -> doi khoi dong
        self.ser.reset_input_buffer()

        self._lock = threading.Lock()
        self._latest = None             # (encoder, ga, lui, phanh)
        self._last_time = 0.0
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    @staticmethod
    def parse_line(line):
        """'count|ga|lui|phanh' -> (int, float, float, float) hoac None neu dong loi."""
        parts = line.strip().split("|")
        if len(parts) != 4:
            return None
        try:
            return int(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])
        except ValueError:
            return None

    def _loop(self):
        while self._running:
            try:
                raw = self.ser.readline()
            except Exception:
                time.sleep(0.05)
                continue
            if not raw:
                continue
            data = self.parse_line(raw.decode("ascii", errors="ignore"))
            if data is None:
                continue
            with self._lock:
                self._latest = data
                self._last_time = time.monotonic()

    def read(self):
        """Tra ve (encoder, ga, lui, phanh, connected)."""
        with self._lock:
            data = self._latest
            age = time.monotonic() - self._last_time
        if data is None or age > SERIAL_TIMEOUT_SEC:
            # Mat ket noi ban dap -> nha ga, phanh het (an toan)
            enc = data[0] if data else 0
            return enc, 0.0, 0.0, 100.0, False
        return (*data, True)

    def close(self):
        self._running = False
        self._thread.join(timeout=0.5)
        self.ser.close()


class DriveDirSelector:
    """
    Arduino khong gui cong tac so, nen suy ra huong chay tu ban dap:
      - dap ga  -> muon chay tien (1)
      - dap lui -> muon chay lui (-1)
      - khong dap gi -> giu huong cu (xe tu troi cham dan do ma sat)
    Chi cho doi chieu khi xe gan nhu dung (speed <= SPEED_DEADZONE). Neu dang chay tien ma dap lui,
    xe giu huong tien voi ga = 0 cho den khi cham lai, sau do moi chuyen sang lui.
    """

    def __init__(self):
        self.dir = 0

    def update(self, ga, lui, current_speed):
        if ga > 0.0:
            requested = 1
        elif lui > 0.0:
            requested = -1
        else:
            requested = self.dir

        changed = False
        if requested != self.dir and (self.dir == 0 or current_speed <= SPEED_DEADZONE):
            self.dir = requested
            changed = True
        return self.dir, changed


def read_camera_distances():
    """
    TODO: noi voi module detect de lay 3 khoang cach (m), -1 = khong co vat, -2 = mat camera.
    Tra ve (distance_xe_may, distance_nguoi_di_bo, distance_o_to).
    Hien tai tra ve -2 (mat camera) -> nguoi lai lai binh thuong.
    """
    return CAMERA_LOST, CAMERA_LOST, CAMERA_LOST


def read_driver_and_camera(pedals: PedalSerialReader, dir_sel: DriveDirSelector,
                           controller: "RawVehicleController") -> Input:
    encoder, ga, lui, phanh, _connected = pedals.read()
    ga, lui, phanh = apply_pedal_interlock(ga, lui, phanh)   # truoc khi suy ra huong chay
    drive_dir, changed = dir_sel.update(ga, lui, controller.speed)
    if changed:
        controller.speed = 0.0   # doi chieu chi xay ra khi xe gan dung -> bat dau lai tu 0

    d_xe_may, d_nguoi, d_o_to = read_camera_distances()
    return Input(
        encoder=encoder,
        ga=ga,
        lui=lui,
        phanh=phanh,
        driveDir=drive_dir,
        distance_xe_may=d_xe_may,
        distance_nguoi_di_bo=d_nguoi,
        distance_o_to=d_o_to,
    )


def simulated_input(t: float) -> Input:
    """Kich ban mo phong: dap ga 100%, xe tien toi nguoi di bo, nguoi di bo roi khung hinh."""
    inp = Input(encoder=30, ga=100.0, driveDir=1)
    if 3.0 <= t < 12.0:
        # Nguoi di bo xuat hien o 5 m, khoang cach giam theo quang duong xe di
        inp.distance_nguoi_di_bo = max(0.0, 5.0 - simulated_input.travelled)
    elif 12.0 <= t < 14.0:
        inp.distance_nguoi_di_bo = CAMERA_LOST
    return inp


simulated_input.travelled = 0.0


# ============================================================================
# 8. VONG LAP CHINH
# ============================================================================
def run(args):
    controller = RawVehicleController()
    sender = JetsonSender(args.jetson_ip, args.jetson_port) if not args.no_send else None
    pedals = None
    dir_sel = DriveDirSelector()
    if not args.sim:
        pedals = PedalSerialReader(args.serial_port)
    print("Chay he thong tong hop")

    t = 0.0
    last_print = -1.0
    try:
        while True:
            loop_start = time.perf_counter()

            if args.sim:
                inp = simulated_input(t)
            else:
                inp = read_driver_and_camera(pedals, dir_sel, controller)

            out = controller.update(inp)

            if args.sim and 3.0 <= t < 12.0:
                simulated_input.travelled += out.speedCmd * DT_SEC

            if sender:
                sender.send(out)

            if t - last_print >= 0.5:
                print(f"t={t:5.2f}s  enc={inp.encoder:4d}  ga={inp.ga:5.1f}  lui={inp.lui:5.1f}  "
                      f"phanh={inp.phanh:5.1f}  dir={inp.driveDir:2d} | "
                      f"steer={out.steerCmd:6.2f} deg  speed={out.speedCmd:4.2f} m/s  "
                      f"limit={out.speedLimit:4.2f}  ped={inp.distance_nguoi_di_bo:5.2f}  "
                      f"reason={out.limitReason}")
                last_print = t

            t += DT_SEC
            if args.sim:
                if t >= args.sim_time:
                    break
            else:
                elapsed = time.perf_counter() - loop_start
                time.sleep(max(0.0, DT_SEC - elapsed))
    except KeyboardInterrupt:
        pass
    finally:
        if sender:
            sender.close()
        if pedals:
            pedals.close()


def parse_args():
    p = argparse.ArgumentParser(description="Dieu khien xe + gioi han toc do theo vat can")
    p.add_argument("--serial-port", default="COM3",
                   help="cong Serial cua Arduino ban dap (Windows: COM3, Linux: /dev/ttyUSB0)")
    p.add_argument("--jetson-ip", default="192.168.1.50")
    p.add_argument("--jetson-port", type=int, default=5005)
    p.add_argument("--no-send", action="store_true", help="khong gui UDP")
    p.add_argument("--sim", action="store_true", help="chay kich ban mo phong")
    p.add_argument("--sim-time", type=float, default=22.0)
    return p.parse_args()


if __name__ == "__main__":
    run(parse_args())
