"""
MAIN MASTER - HUST Robotics 2026
Tích hợp 4 luồng xử lý: YOLO, LDWS, Lane Change Planner, Vehicle Dynamics.
Bao gồm TCP Server giao tiếp chuẩn C++ (gửi lệnh #vcb, xem CMD_PREFIX) xuống xe (Jetson).

Xe gửi HAI luồng ảnh JPEG, mỗi luồng một cổng UDP:
  - Camera bird's-eye, cổng 7000 (--lane-port), 640x480: LDWS (giữ làn) + planner (chuyển làn).
  - Camera nhìn đường, cổng 7001 (--road-port), 640x360: YOLO đo khoảng cách vật cản.
Nhận được cả hai kiểu xe (FrameReceiver tự nhận ra):
  - Jetson (braincar, ThreadUdpServer): mỗi ảnh cắt thành nhiều gói 1300 B, đầu mỗi gói 8 byte >IHH
    (frameId, chỉ số gói, tổng số gói). Ảnh cam bám làn ĐÃ khử méo trên Jetson -> mặc định không khử méo lại.
  - Pi 5 (pi5/pi_bridge.py): mỗi gói một ảnh JPEG, ảnh cam bám làn CHƯA khử méo -> chạy với --lane-raw.

Ba vòng lặp chạy song song (cùng ý tưởng với wheel_pedal_bridge.py: lệnh lái không bao giờ phải chờ việc nặng):
  - Vòng điều khiển 50 Hz (thread riêng): bàn đạp / vô lăng MỚI NHẤT -> RawVehicleController -> gửi #vcb qua TCP.
    Không chờ ảnh: YOLO / LDWS chậm hay mất camera thì lệnh lái vẫn đi đều 50 lần mỗi giây.
  - Vòng YOLO (thread riêng): ảnh camera nhìn đường MỚI NHẤT -> khoảng cách vật cản.
  - Vòng làn (thread chính, vì cv2.imshow phải ở thread chính): ảnh bird's-eye MỚI NHẤT -> LDWS, planner (dùng khoảng
    cách mới nhất của vòng YOLO) -> hướng chuyển làn; hiển thị cả hai camera.
    Kết quả mỗi camera cũ hơn VISION_TIMEOUT_S thì coi như mất camera đó (-2 cho khoảng cách, quy ước của main_4.py).

Chạy:  python main_master.py
"""

import argparse
import os
import socket
import struct
import threading
import time
import traceback
from typing import NamedTuple, Optional

import cv2
import numpy as np

# --- 1. IMPORT MODULE BÁM LÀN (LDWS) ---
from lane_vision import LaneVision
from lane_warning import LaneWarning
from main_2 import GroundCalibration, IsoLdwCriteria, TrackAndVehicleParams

# --- 2. IMPORT MODULE CHUYỂN LÀN (PLANNER) ---
from lane_change_planner import LaneChangePlanner, PlannerConfig

# --- 3. IMPORT MODULE NHẬN DIỆN VẬT CẢN (YOLO) ---
from object_detector import ObstacleDetector

# --- 4. IMPORT MODULE ĐIỀU KHIỂN (Lấy từ main_4.py, bỏ JetsonSender cũ) ---
from main_4 import (
    CAMERA_LOST,
    DT_SEC,
    DriveDirSelector,
    Input,
    Output,
    PedalSerialReader,
    RawVehicleController,
    apply_pedal_interlock,
)

LANE_FRAME_SIZE = (640, 480)   # camera bird's-eye: cỡ của GroundCalibration
ROAD_FRAME_SIZE = (640, 360)   # camera nhìn đường: 16:9, ObstacleDetector tự đưa về 1280x720
VISION_TIMEOUT_S = 0.5         # kết quả ảnh cũ hơn mức này -> coi như mất camera đó
NO_OBSTACLE_M = 2.5            # khoảng cách đưa planner khi không thấy vật cản (như bản cũ)
MAX_CONTROL_DT_S = 0.1         # chặn bước tích phân nếu thread điều khiển bị trễ bất thường
STATUS_PERIOD_S = 0.5          # in trạng thái tối đa 2 lần/giây (print liên tục làm chậm, xem wheel_pedal_bridge.py)
NO_PEDALS = (0, 0.0, 0.0, 100.0, False)   # không mở được Serial: như PedalSerialReader khi mất kết nối (phanh 100 %)
# Ký tự đầu lệnh TCP gửi xe: "#vcb:...;;". braincar trên Jetson nhận cả '#' lẫn '@' (bản threadTcpClient.cpp đã sửa);
# bản braincar cũ và pi_bridge (Pi 5) chỉ nhận '@' -> đổi lại "@" nếu chạy với chúng.
CMD_PREFIX = "#"


# --- 5. MODULE TRUYỀN THÔNG ---
class JetsonSender:
    """TCP Server chờ máy tính trên xe (Jetson / Pi 5) kết nối để nhận lệnh điều khiển.

    - accept() không chặn: mỗi chu kỳ chỉ kiểm tra có kết nối mới không, không làm trễ vòng 50 Hz.
    - Luôn dùng kết nối MỚI NHẤT: xe khởi động lại / rớt Wi-Fi rồi nối lại thì thay kết nối cũ ngay, không phải chờ
      kết nối cũ báo lỗi (có thể mất hàng chục giây).
    - sendall có timeout: xe ngừng đọc (buffer TCP đầy) thì sau SEND_TIMEOUT_S bỏ kết nối, không treo vòng điều khiển.
    """

    SEND_TIMEOUT_S = 0.1

    def __init__(self, port=6000):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        if os.name != "nt":
            # Linux: bind lại cổng được ngay sau khi tắt. Windows: SO_REUSEADDR cho HAI chương trình cùng bind một cổng
            # (xe có thể nối nhầm vào bản đang chạy cũ) nên không bật
            self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(("0.0.0.0", port))
        self.server.listen(1)
        self.server.setblocking(False)
        self.client = None
        self.addr = None

    @property
    def connected(self):
        return self.client is not None

    def _accept_new(self):
        try:
            client, addr = self.server.accept()
        except (BlockingIOError, InterruptedError):
            return  # chưa có ai kết nối
        except OSError:
            return
        client.settimeout(self.SEND_TIMEOUT_S)
        client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        if self.client:
            print(f"[TCP] Ket noi moi tu {addr} thay ket noi cu {self.addr}")
            self.client.close()
        else:
            print(f"[TCP] Xe da ket noi tu {addr}")
        self.client, self.addr = client, addr

    def send(self, out):
        # 1. Nhận kết nối mới (nếu có), không chờ
        self._accept_new()
        if not self.client:
            return

        # 2. Đóng gói lệnh theo chuẩn C++ (braincar ThreadTcpClient): #vcb:<speed>;<steer>;<dir>;;\r\n
        msg = f"{CMD_PREFIX}vcb:{out.speedCmd:.3f};{out.steerCmd:.3f};{out.driveDirCmd};;\r\n"

        # 3. Gửi lệnh đi
        try:
            self.client.sendall(msg.encode("ascii"))
        except OSError:  # gồm cả socket.timeout
            print("[TCP] Mat ket noi toi xe. Dang cho ket noi lai...")
            self.client.close()
            self.client = None

    def send_stop(self):
        """Lệnh dừng (tốc độ 0, lái thẳng, số N), cùng định dạng #vcb, gửi khi tắt chương trình."""
        self.send(Output())

    def close(self):
        if self.client:
            self.client.close()
        self.server.close()


class FrameAssembler:
    """Ghép các gói UDP của MỘT luồng ảnh thành JPEG (cùng giao thức FrameAssembler trong main.py của nhóm Jetson).

    Gói Jetson: [frameId u32][chỉ số gói u16][tổng số gói u16][payload], big-endian. Ảnh thiếu gói thì bỏ (quá STALE_S
    hoặc đã có ảnh mới hơn ghép xong). frameId lùi mạnh nghĩa là braincar vừa chạy lại -> đếm lại từ đầu.
    Gói bắt đầu bằng FF D8 (đầu JPEG) là một ảnh trọn trong một gói (pi_bridge.py): trả luôn. Gói Jetson chỉ bắt đầu
    bằng FF D8 khi frameId >= 0xFFD80000 (hơn 4 tỉ ảnh), nên không nhầm được.
    """

    HEADER = struct.Struct(">IHH")
    JPEG_SOI = b"\xff\xd8"
    STALE_S = 1.0
    RESTART_GAP = 1000

    def __init__(self):
        self.partial = {}  # frameId -> [tổng số gói, {chỉ số: payload}, thời điểm gói đầu]
        self.last_done = 0
        self.incomplete = 0  # số ảnh bị bỏ vì thiếu gói (rớt gói Wi-Fi)

    def push(self, packet, now):
        """Thêm một gói. Trả về bytes JPEG khi có một ảnh mới hoàn chỉnh, ngược lại None."""
        if packet[:2] == self.JPEG_SOI:
            return packet
        if len(packet) <= self.HEADER.size:
            return None
        fid, idx, total = self.HEADER.unpack_from(packet)
        if total == 0 or idx >= total:
            return None
        if fid <= self.last_done:
            if self.last_done - fid < self.RESTART_GAP:
                return None  # gói của ảnh cũ hơn ảnh đã ghép xong
            self.last_done, self.partial = 0, {}

        entry = self.partial.get(fid)
        if entry is None or entry[0] != total:
            entry = self.partial[fid] = [total, {}, now]
        entry[1][idx] = packet[self.HEADER.size:]
        if len(entry[1]) < total:
            if len(self.partial) > 8:
                self._drop_stale(now)
            return None

        jpeg = b"".join(entry[1][i] for i in range(total))
        self.incomplete += sum(1 for k in self.partial if k < fid)
        self.last_done = fid
        self.partial = {k: v for k, v in self.partial.items() if k > fid}
        return jpeg

    def _drop_stale(self, now):
        old = len(self.partial)
        self.partial = {k: v for k, v in self.partial.items() if now - v[2] < self.STALE_S}
        self.incomplete += old - len(self.partial)


class FrameReceiver:
    """Nhận ảnh JPEG qua UDP (Jetson: nhiều gói/ảnh; pi_bridge: một gói/ảnh), luôn trả ảnh MỚI NHẤT. Mỗi camera một
    receiver.

    Camera gửi 30 ảnh/s, vòng ảnh (YOLO trên CPU) chỉ xử lý được ~5-12 ảnh/s nên các gói dồn lại trong buffer socket.
    Đọc lần lượt từng ảnh thì ảnh đang xử lý cũ dần tới vài giây. Ở đây mỗi lần đọc hết các gói đang chờ, ghép ảnh và
    chỉ giữ ảnh hoàn chỉnh mới nhất (giống wheel_pedal_bridge.py chỉ giữ dòng Serial cuối).
    """

    WAIT_S = 0.1

    def __init__(self, port, frame_size, name=""):
        self.frame_size = frame_size
        self.name = name or f"UDP {port}"
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        self.sock.bind(("0.0.0.0", port))
        self.assembler = FrameAssembler()
        self.skipped = 0  # tổng số ảnh hoàn chỉnh nhưng đã cũ, bị bỏ qua
        self._warned_size = False

    def _recv(self, timeout):
        """Một gói, hoặc None nếu không có gói trong timeout (timeout <= 0: không chờ)."""
        while True:
            try:
                if timeout <= 0:
                    self.sock.setblocking(False)
                else:
                    self.sock.settimeout(timeout)
                packet, _ = self.sock.recvfrom(65535)
                return packet
            except (socket.timeout, BlockingIOError, InterruptedError):
                return None
            except ConnectionResetError:
                continue  # Windows báo ICMP port unreachable qua recv: bỏ qua, đọc gói kế

    def latest(self):
        """(ảnh BGR cỡ frame_size, thời điểm nhận) hoặc (None, None) nếu không ghép được ảnh nào trong WAIT_S."""
        deadline = time.perf_counter() + self.WAIT_S
        newest = None
        while True:
            # Chưa có ảnh: chờ gói tới hạn chót. Đã có ảnh: chỉ vét nốt các gói đang chờ (không chờ thêm).
            timeout = deadline - time.perf_counter() if newest is None else 0.0
            packet = self._recv(timeout)
            if packet is None:
                break
            jpeg = self.assembler.push(packet, time.perf_counter())
            if jpeg is not None:
                if newest is not None:
                    self.skipped += 1
                newest = jpeg
        if newest is None:
            return None, None
        t_recv = time.perf_counter()

        frame = cv2.imdecode(np.frombuffer(newest, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return None, None
        size = (frame.shape[1], frame.shape[0])
        if size != self.frame_size:
            if not self._warned_size:
                self._warned_size = True
                print(f"[WARN] {self.name}: xe gui anh {size[0]}x{size[1]}, chuong trinh can "
                      f"{self.frame_size[0]}x{self.frame_size[1]} -> keo gian. Sai ti le thi khoang cach YOLO / "
                      f"hinh hoc lan bi sai: sua kich thuoc anh tren xe (kLaneWidth / kRoadWidth trong main.cpp).")
            frame = cv2.resize(frame, self.frame_size)
        return frame, t_recv

    def close(self):
        self.sock.close()


class RoadResult(NamedTuple):
    """Kết quả mới nhất của vòng YOLO (camera nhìn đường). Thay cả object một lần nên không cần khoá."""

    t: float  # thời điểm nhận ảnh (time.perf_counter)
    distances: dict  # {"xe_may": m, "nguoi_di_bo": m, "o_to": m}, -1 = không có vật
    proc_ms: float
    count: int  # số ảnh đã xử lý (để tính fps)


class LaneResult(NamedTuple):
    """Kết quả mới nhất của vòng làn (camera bird's-eye)."""

    t: float
    plan_dir: Optional[str]  # "KEEP" / "LEFT" / "RIGHT", None nếu planner không chọn quỹ đạo nào
    plan_status: str
    lor: float
    proc_ms: float
    count: int


# --- 6. HỆ THỐNG ĐIỀU KHIỂN TRUNG TÂM ---
class AutonomousMasterController:
    def __init__(self, args, pedals=None):
        self.args = args

        self.calib = GroundCalibration()
        self.params = TrackAndVehicleParams()
        self.iso = IsoLdwCriteria()

        self.vision = LaneVision(self.calib, self.params)
        self.warning = LaneWarning(self.params, self.iso)
        # Jetson (braincar) gửi ảnh cam bám làn ĐÃ khử méo (cùng bộ K với GroundCalibration); khử méo thêm lần nữa làm
        # cong vạch và sai homography. Chỉ pi_bridge.py gửi ảnh thô -> --lane-raw.
        self.lane_raw = bool(getattr(args, "lane_raw", False))

        m_per_px_x = self.vision.BEV_WIDTH / self.calib.width
        m_per_px_y = self.vision.BEV_DEPTH / self.calib.height
        self.planner_cfg = PlannerConfig(
            speed_mps=args.target_speed,
            meters_per_pixel_x=m_per_px_x,
            meters_per_pixel_y=m_per_px_y,
            ego_x_px=self.calib.width / 2.0,
            ego_y_px=float(self.calib.height - 1),
            lane_width_m=self.params.lane_width,
            vehicle_width_m=self.params.vehicle_width,
            vehicle_length_m=self.params.rear_axle_to_front,
            trigger_distance_m=1.1,
            safe_margin_m=0.10,
        )
        self.planner = LaneChangePlanner(self.planner_cfg)
        self.detector = ObstacleDetector()

        self.vehicle_ctrl = RawVehicleController()
        self.dir_selector = DriveDirSelector()
        self.pedals = pedals

        # TCP Server chờ xe kết nối (mặc định cổng 6000)
        self.sender = None if args.no_send else JetsonSender(port=getattr(args, "tcp_port", 6000))

        # Trạng thái dùng chung giữa các vòng lặp (mỗi biến một vòng ghi, thay cả object nên không cần khoá)
        self.road_result = None   # RoadResult mới nhất: vòng YOLO ghi
        self.lane_result = None   # LaneResult mới nhất: vòng làn ghi
        self.yolo_view = None     # ảnh YOLO đã vẽ hộp, vòng làn (thread chính) hiển thị
        self._road_frames = 0
        self._lane_frames = 0
        self.stop_event = threading.Event()
        self._control_thread = None
        self._road_thread = None
        self._t_start = time.perf_counter()
        self._last_status = None  # (thời điểm, số ảnh làn, số ảnh YOLO, số dòng Serial) lần in trước

    # ----------------------------- VÒNG YOLO (camera nhìn đường, thread riêng) -----------------------------
    def road_step(self, raw_frame: np.ndarray, t_now: float, draw: bool = True):
        """YOLO + khoảng cách trên ảnh camera nhìn đường (giữ nguyên thuật toán), công bố cho vòng làn và vòng điều
        khiển. Trả về ảnh đã vẽ hộp (draw=True) hoặc None."""
        t0 = time.perf_counter()
        distances, yolo_frame = self.detector.process_frame(raw_frame, t_now)
        self._road_frames += 1
        self.road_result = RoadResult(t=t_now, distances=distances, proc_ms=(time.perf_counter() - t0) * 1000.0,
                                      count=self._road_frames)
        return self.detector.draw_bboxes(yolo_frame) if draw else None

    def _road_loop(self, receiver: "FrameReceiver", draw: bool):
        try:
            while not self.stop_event.is_set():
                frame, t_recv = receiver.latest()
                if frame is not None:
                    view = self.road_step(frame, t_recv, draw)
                    if draw:
                        self.yolo_view = view
        except Exception:
            traceback.print_exc()
            print("[YOLO] Vong YOLO bi loi -> dung chuong trinh")
            self.stop_event.set()

    # ----------------------------- VÒNG LÀN (camera bird's-eye, thread chính) -----------------------------
    def front_obstacle(self, now: float):
        """(khoảng cách vật cản gần nhất phía trước, tuổi số đo) cho planner, từ kết quả YOLO còn mới."""
        road = self.fresh_road(now)
        if road is None:
            return NO_OBSTACLE_M, 0.0
        valid = [d for d in road.distances.values() if d >= 0]
        return (min(valid) if valid else NO_OBSTACLE_M), max(0.0, now - road.t)

    def lane_step(self, raw_frame: np.ndarray, t_now: float, draw: bool = True):
        """LDWS -> planner trên ảnh bird's-eye (giữ nguyên thuật toán), khoảng cách vật cản lấy từ vòng YOLO.
        Trả về (warn_res, plan_res, 3 ảnh hiển thị hoặc None khi draw=False)."""
        t0 = time.perf_counter()
        if self.lane_raw:
            geom, images = self.vision.process_raw_frame(raw_frame, t_now)
        else:
            geom, images = self.vision.process_undistorted(raw_frame, t_now)
        warn_res = self.warning.update(geom)

        front_dist, dist_age = self.front_obstacle(t_now)
        plan_res = self.planner.propose(
            bev=images.bev,
            front_distance_m=front_dist,
            distance_age_s=dist_age,
        )

        self._lane_frames += 1
        self.lane_result = LaneResult(
            t=t_now,
            plan_dir=plan_res.selected.direction if plan_res.selected else None,
            plan_status=plan_res.status,
            lor=warn_res.lor,
            proc_ms=(time.perf_counter() - t0) * 1000.0,
            count=self._lane_frames,
        )

        if not draw:
            return warn_res, plan_res, None
        vis_cam, vis_bev_ldws = self.vision.draw(images, geom, warn_res.level,
                                                 self.warning.hud_rows(warn_res, geom.valid))
        return warn_res, plan_res, (vis_cam, vis_bev_ldws, plan_res.annotated_bev)

    # ----------------------------- VÒNG ĐIỀU KHIỂN (thread riêng, 50 Hz) -----------------------------
    def fresh_road(self, now: float):
        """Kết quả YOLO còn mới, hoặc None nếu chưa có / cũ hơn VISION_TIMEOUT_S (mất camera nhìn đường)."""
        res = self.road_result
        return None if res is None or now - res.t > VISION_TIMEOUT_S else res

    def fresh_lane(self, now: float):
        """Kết quả làn còn mới, hoặc None (mất camera bird's-eye)."""
        res = self.lane_result
        return None if res is None or now - res.t > VISION_TIMEOUT_S else res

    def control_step(self, pedal_data: tuple, road: Optional[RoadResult], lane: Optional[LaneResult]):
        """Phần điều khiển của process_step cũ (giữ nguyên logic). road / lane: kết quả còn mới của từng camera,
        None = mất camera đó."""
        enc, ga, lui, phanh, connected = pedal_data
        ga, lui, phanh = apply_pedal_interlock(ga, lui, phanh)
        drive_dir, changed = self.dir_selector.update(ga, lui, self.vehicle_ctrl.speed)
        if changed:
            # Như read_driver_and_camera() của main_4.py: chỉ đổi chiều khi xe gần dừng -> bắt đầu lại từ 0
            self.vehicle_ctrl.speed = 0.0

        if road is None:
            # Chưa có ảnh hoặc ảnh quá cũ: quy ước "mất camera" của main_4.py (giới hạn tốc độ hồi dần về tối đa)
            distances = {"xe_may": CAMERA_LOST, "nguoi_di_bo": CAMERA_LOST, "o_to": CAMERA_LOST}
        else:
            distances = road.distances
        plan_dir = lane.plan_dir if lane is not None else None

        sys_input = Input(
            encoder=enc, ga=ga, lui=lui, phanh=phanh, driveDir=drive_dir,
            distance_xe_may=distances["xe_may"],
            distance_nguoi_di_bo=distances["nguoi_di_bo"],
            distance_o_to=distances["o_to"],
        )
        sys_output = self.vehicle_ctrl.update(sys_input)

        if plan_dir == "LEFT":
            sys_output.steerCmd = max(-20.0, sys_output.steerCmd - 10.0)
        elif plan_dir == "RIGHT":
            sys_output.steerCmd = min(20.0, sys_output.steerCmd + 10.0)

        return sys_output

    def start(self, road_receiver: Optional["FrameReceiver"] = None, draw: bool = True):
        """Chạy vòng điều khiển 50 Hz và (nếu có road_receiver) vòng YOLO, mỗi vòng một thread."""
        self._control_thread = threading.Thread(target=self._control_loop, name="control-50Hz", daemon=True)
        self._control_thread.start()
        if road_receiver is not None:
            self._road_thread = threading.Thread(target=self._road_loop, args=(road_receiver, draw), name="yolo",
                                                 daemon=True)
            self._road_thread.start()

    def _control_loop(self):
        """Chạy đều mỗi DT_SEC (50 Hz), không phụ thuộc tốc độ xử lý ảnh."""
        last = time.perf_counter() - DT_SEC
        next_tick = time.perf_counter()
        last_print = -STATUS_PERIOD_S
        try:
            while not self.stop_event.is_set():
                now = time.perf_counter()
                # Mô hình động lực học tích phân theo thời gian THẬT giữa hai chu kỳ (bản cũ gọi một lần mỗi ảnh,
                # ~80-250 ms, nhưng vẫn tính như 20 ms -> xe tăng tốc / hồi giới hạn chậm hơn thiết kế vài lần)
                self.vehicle_ctrl.dt = min(max(now - last, 1e-3), MAX_CONTROL_DT_S)
                last = now

                pedal_data = self.pedals.read() if self.pedals else NO_PEDALS
                road, lane = self.fresh_road(now), self.fresh_lane(now)
                out = self.control_step(pedal_data, road, lane)
                if self.sender:
                    self.sender.send(out)

                if now - last_print >= STATUS_PERIOD_S:
                    self._print_status(now, out, pedal_data[4], road, lane)
                    last_print = now

                next_tick += DT_SEC
                delay = next_tick - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_tick = time.perf_counter()  # máy bận, trễ nhịp: bắt nhịp lại, không chạy dồn bù
        except Exception:
            traceback.print_exc()
            print("[CONTROL] Vong dieu khien bi loi -> dung chuong trinh")
            self.stop_event.set()

    def _print_status(self, now, out, pedal_ok, road, lane):
        lane_n, road_n = self._lane_frames, self._road_frames
        packets = self.pedals.packet_count if self.pedals else 0
        if self._last_status is None:
            lane_fps = road_fps = pedal_hz = 0.0
        else:
            t0, l0, r0, p0 = self._last_status
            dt = now - t0
            lane_fps, road_fps, pedal_hz = (lane_n - l0) / dt, (road_n - r0) / dt, (packets - p0) / dt
        self._last_status = (now, lane_n, road_n, packets)

        def cam_text(res, fresh, fps):
            if res is None:
                return "chua co anh"
            txt = f"{fps:4.1f} fps, xu ly {res.proc_ms:.0f} ms"
            return txt + (" (MAT CAMERA)" if fresh is None else "")

        lor = f"{lane.lor:.2f}" if lane else "--"
        plan = (lane.plan_dir or lane.plan_status) if lane else "--"
        car = "da ket noi" if self.sender and self.sender.connected else "chua ket noi"
        print(f"[{now - self._t_start:7.2f}s] speed {out.speedCmd:.2f} m/s | steer {out.steerCmd:+6.1f} | "
              f"dir {out.driveDirCmd:+d} | limit {out.speedLimit:.2f} ({out.limitReason}) | "
              f"ban dap {'OK' if pedal_ok else 'MAT'} {pedal_hz:.0f} Hz | xe {car} | "
              f"cam lan {cam_text(self.lane_result, lane, lane_fps)} | cam YOLO {cam_text(self.road_result, road, road_fps)}"
              f" | LOR {lor} | plan {plan}", flush=True)

    def close(self):
        """Dừng vòng điều khiển và vòng YOLO, gửi lệnh dừng cho xe rồi đóng kết nối."""
        self.stop_event.set()
        if self._control_thread:
            self._control_thread.join(timeout=1.0)
        if self._road_thread:
            self._road_thread.join(timeout=2.0)
        if self.sender:
            self.sender.send_stop()
            self.sender.close()


def main():
    parser = argparse.ArgumentParser(description="He thong Xe Dieu Khien - HUST Robotics 2026")
    parser.add_argument("--serial-port", default="COM7", help="Cong Serial noi Arduino (ban dap, vo lang)")
    parser.add_argument("--lane-port", type=int, default=7000, help="Port UDP anh camera bird's-eye (giu/chuyen lan)")
    parser.add_argument("--road-port", type=int, default=7001, help="Port UDP anh camera nhin duong (YOLO)")
    parser.add_argument("--tcp-port", type=int, default=6000, help="Port TCP cho xe ket noi de nhan lenh #vcb")
    parser.add_argument("--target-speed", type=float, default=0.3, help="Toc do muc tieu m/s (cho planner)")
    parser.add_argument("--lane-raw", action="store_true",
                        help="Anh cam lan CHUA khu meo (xe chay pi5/pi_bridge.py). Mac dinh: Jetson da khu meo")
    parser.add_argument("--no-send", action="store_true", help="Tat gui lenh TCP")
    parser.add_argument("--no-show", action="store_true", help="Tat hien thi do hoa OpenCV")
    args = parser.parse_args()

    try:
        pedals = PedalSerialReader(args.serial_port)
    except ModuleNotFoundError:
        print("[WARN] Chua cai pyserial (pip install pyserial): khong doc duoc ban dap -> phanh 100 %.")
        pedals = None
    except Exception as e:
        print(f"[WARN] Khong mo duoc Serial ban dap {args.serial_port}: {e} -> phanh 100 %.")
        pedals = None

    show = not args.no_show
    master = AutonomousMasterController(args, pedals)
    lane_rx = FrameReceiver(args.lane_port, LANE_FRAME_SIZE, "cam lan")
    road_rx = FrameReceiver(args.road_port, ROAD_FRAME_SIZE, "cam YOLO")
    master.start(road_rx, draw=show)

    print("\n[HUST ROBOTICS] HE THONG MASTER DA SAN SANG!")
    print(f"[INFO] Anh bird's-eye UDP {args.lane_port} ({'THO, tu khu meo' if args.lane_raw else 'da khu meo tren xe'})"
          f" | anh nhin duong (YOLO) UDP {args.road_port} | TCP server cho xe ket noi o cong {args.tcp_port} | "
          f"vong dieu khien {1.0 / DT_SEC:.0f} Hz")
    print("[INFO] Chi chay MOT chuong trinh nhan anh tren PC (main_master.py HOAC main.py cua nhom Jetson): ca hai cung "
          "dung cong 7000/7001.")

    shown_yolo = None
    try:
        while not master.stop_event.is_set():
            frame, t_recv = lane_rx.latest()   # chờ tối đa 0.1 s, nên ảnh YOLO vẫn được vẽ khi mất camera làn
            if frame is not None:
                _, _, views = master.lane_step(frame, t_recv, draw=show)
                if show:
                    f_cam, f_ldws, f_plan = views
                    cv2.imshow("2. LDWS Camera Overlay", f_cam)
                    cv2.imshow("3. BEV Lane Tracking", f_ldws)
                    cv2.imshow("4. BEV Lane Change Planner", f_plan)
            if show:
                yolo = master.yolo_view
                if yolo is not None and yolo is not shown_yolo:
                    cv2.imshow("1. YOLO Detections", yolo)
                    shown_yolo = yolo
                if (cv2.waitKey(1) & 0xFF) in (27, ord("q")):
                    break

    except KeyboardInterrupt:
        print("\n[INFO] Da dung vong lap chinh.")
    finally:
        master.close()
        lane_rx.close()
        road_rx.close()
        if pedals:
            pedals.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
