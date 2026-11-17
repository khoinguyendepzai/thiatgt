"""
MAIN MASTER - HUST Robotics 2026
Tích hợp 4 luồng xử lý: YOLO, LDWS, Lane Change Planner, Vehicle Dynamics.
Bao gồm TCP Server giao tiếp chuẩn C++ (gửi lệnh @vcb) xuống xe (Jetson / Pi 5).

Client jetson/ trên branch Lane-Keeping gửi HAI luồng JPEG chia chunk (>IHH):
  - Camera làn đã khử méo (chưa BEV), cổng 7000 (--lane-port), 640x480: LDWS (giữ làn) + planner (chuyển làn).
  - Camera nhìn đường, cổng 7001 (--road-port), 640x480: YOLO đo khoảng cách vật cản.

Ba vòng lặp chạy song song (cùng ý tưởng với wheel_pedal_bridge.py: lệnh lái không bao giờ phải chờ việc nặng):
  - Vòng điều khiển 50 Hz (thread riêng): bàn đạp / vô lăng MỚI NHẤT -> RawVehicleController -> gửi @vcb qua TCP.
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
import threading
import time
import traceback
from typing import NamedTuple, Optional
from frame_transport import FrameReceiver, ResultHistory

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

LANE_FRAME_SIZE = (640, 480)   # camera làn đã khử méo: cỡ GroundCalibration
ROAD_FRAME_SIZE = (640, 480)   # camera nhìn đường: đúng cỡ gửi từ Jetson; YOLO tự resize
VISION_TIMEOUT_S = 0.5         # kết quả ảnh cũ hơn mức này -> coi như mất camera đó
NO_OBSTACLE_M = 2.5            # khoảng cách đưa planner khi không thấy vật cản (như bản cũ)
MAX_CONTROL_DT_S = 0.1         # chặn bước tích phân nếu thread điều khiển bị trễ bất thường
STATUS_PERIOD_S = 0.5          # in trạng thái tối đa 2 lần/giây (print liên tục làm chậm, xem wheel_pedal_bridge.py)
NO_PEDALS = (0, 0.0, 0.0, 100.0, False)   # không mở được Serial: như PedalSerialReader khi mất kết nối (phanh 100 %)


# --- 5. MODULE TRUYỀN THÔNG ---
class JetsonSender:
    """TCP Server chờ máy tính trên xe (Jetson / Pi 5) kết nối để nhận lệnh điều khiển.

    - accept() không chặn: mỗi chu kỳ chỉ kiểm tra có kết nối mới không, không làm trễ vòng 50 Hz.
    - Luôn dùng kết nối MỚI NHẤT: xe khởi động lại / rớt Wi-Fi rồi nối lại thì thay kết nối cũ ngay, không phải chờ
      kết nối cũ báo lỗi (có thể mất hàng chục giây).
    - sendall có timeout: xe ngừng đọc (buffer TCP đầy) thì sau SEND_TIMEOUT_S bỏ kết nối, không treo vòng điều khiển.
    """

    SEND_TIMEOUT_S = 0.1

    def __init__(self, host="0.0.0.0", port=6000):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        if os.name != "nt":
            # Linux: bind lại cổng được ngay sau khi tắt. Windows: SO_REUSEADDR cho HAI chương trình cùng bind một cổng
            # (xe có thể nối nhầm vào bản đang chạy cũ) nên không bật
            self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((host, port))
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

        # 2. Đóng gói lệnh theo chuẩn C++: @vcb:<speed>;<steer>;<dir>;;\r\n
        msg = f"@vcb:{out.speedCmd:.2f};{out.steerCmd:.2f};{out.driveDirCmd};;\r\n"

        # 3. Gửi lệnh đi
        try:
            self.client.sendall(msg.encode("ascii"))
        except OSError:  # gồm cả socket.timeout
            print("[TCP] Mat ket noi toi xe. Dang cho ket noi lai...")
            self.client.close()
            self.client = None

    def send_stop(self):
        """Lệnh dừng (tốc độ 0, lái thẳng, số N), cùng định dạng @vcb, gửi khi tắt chương trình."""
        self.send(Output())

    def close(self):
        if self.client:
            self.client.close()
        self.server.close()


class RoadResult(NamedTuple):
    """Kết quả mới nhất của vòng YOLO (camera nhìn đường). Thay cả object một lần nên không cần khoá."""

    t: float  # thời điểm nhận ảnh (time.perf_counter)
    distances: dict  # {"xe_may": m, "nguoi_di_bo": m, "o_to": m}, -1 = không có vật
    proc_ms: float
    count: int  # số ảnh đã xử lý (để tính fps)
    frame_id: Optional[int] = None


class LaneResult(NamedTuple):
    """Kết quả mới nhất của vòng làn (camera làn đã khử méo)."""

    t: float
    plan_dir: Optional[str]  # "KEEP" / "LEFT" / "RIGHT", None nếu planner không chọn quỹ đạo nào
    plan_status: str
    lor: float
    proc_ms: float
    count: int
    frame_id: Optional[int] = None
    road_frame_id: Optional[int] = None
    sync_skew_s: Optional[float] = None


# --- 6. HỆ THỐNG ĐIỀU KHIỂN TRUNG TÂM ---
class AutonomousMasterController:
    def __init__(self, args, pedals=None):
        self.args = args

        self.calib = GroundCalibration()
        self.params = TrackAndVehicleParams()
        self.iso = IsoLdwCriteria()

        self.vision = LaneVision(self.calib, self.params)
        self.warning = LaneWarning(self.params, self.iso)

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
        self.sender = None if args.no_send else JetsonSender(
            host=getattr(args, "tcp_host", "0.0.0.0"),
            port=getattr(args, "tcp_port", 6000),
        )

        # Trạng thái dùng chung giữa các vòng lặp (mỗi biến một vòng ghi, thay cả object nên không cần khoá)
        self.road_history = ResultHistory(getattr(args, "max_frame_skew", 0.1), VISION_TIMEOUT_S)
        self.lane_receiver = None
        self.road_receiver = None
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
    def road_step(self, raw_frame: np.ndarray, t_now: float, draw: bool = True, frame_id=None):
        """YOLO + khoảng cách trên ảnh camera nhìn đường (giữ nguyên thuật toán), công bố cho vòng làn và vòng điều
        khiển. Trả về ảnh đã vẽ hộp (draw=True) hoặc None."""
        t0 = time.perf_counter()
        distances, yolo_frame = self.detector.process_frame(raw_frame, t_now)
        self._road_frames += 1
        self.road_result = RoadResult(t=t_now, distances=distances, proc_ms=(time.perf_counter() - t0) * 1000.0,
                                      count=self._road_frames, frame_id=frame_id)
        self.road_history.add(self.road_result)
        return self.detector.draw_bboxes(yolo_frame) if draw else None

    def _road_loop(self, receiver: "FrameReceiver", draw: bool):
        try:
            while not self.stop_event.is_set():
                frame, t_recv = receiver.latest()
                if frame is not None:
                    view = self.road_step(frame, t_recv, draw, frame_id=receiver.last_frame_id)
                    if draw:
                        self.yolo_view = view
        except Exception:
            traceback.print_exc()
            print("[YOLO] Vong YOLO bi loi -> dung chuong trinh")
            self.stop_event.set()

    # ----------------------------- VÒNG LÀN (camera làn đã khử méo, thread chính) -----------------------------
    def front_obstacle(self, now: float):
        """(khoảng cách vật cản gần nhất phía trước, tuổi số đo) cho planner, từ kết quả YOLO còn mới."""
        road = self.road_history.pick(now)
        return self._front_distance(road, now)

    @staticmethod
    def _front_distance(road, lane_t):
        if road is None:
            return None, 0.0
        valid = [d for d in road.distances.values() if np.isfinite(d) and d >= 0]
        return (min(valid) if valid else NO_OBSTACLE_M), lane_t - road.t

    def lane_step(self, raw_frame: np.ndarray, t_now: float, draw: bool = True, frame_id=None):
        """LDWS -> planner trên ảnh bird's-eye (giữ nguyên thuật toán), khoảng cách vật cản lấy từ vòng YOLO.
        Trả về (warn_res, plan_res, 3 ảnh hiển thị hoặc None khi draw=False)."""
        t0 = time.perf_counter()
        geom, images = self.vision.process_undistorted(raw_frame, t_now)
        warn_res = self.warning.update(geom)

        matched_road = self.road_history.pick(t_now)
        front_dist, dist_age = self._front_distance(matched_road, t_now)
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
            frame_id=frame_id,
            road_frame_id=matched_road.frame_id if matched_road else None,
            sync_skew_s=dist_age if matched_road else None,
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
        return None if res is None or not 0 <= now - res.t <= VISION_TIMEOUT_S else res

    def fresh_lane(self, now: float):
        """Kết quả làn còn mới, hoặc None (mất camera làn đã khử méo)."""
        res = self.lane_result
        return None if res is None or not 0 <= now - res.t <= VISION_TIMEOUT_S else res

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
        self.road_receiver = road_receiver
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

        for name, rx in (("lane", self.lane_receiver), ("road", self.road_receiver)):
            if rx is not None:
                st = rx.stats()
                age = st['last_frame_age_s']
                age_text = f"{age:.3f}s" if age is not None else "--"
                print(f"[RX {name}] packets={st['packets']} assembled={st['assembled']} "
                      f"decoded={st['decoded']} delivered={st['delivered']} id={st['last_frame_id']} "
                      f"age={age_text} invalid={st['invalid']} size_errors={st['size_errors']} "
                      f"source={st['last_source']}", flush=True)
        if lane is not None:
            print(f"[SYNC] lane_id={lane.frame_id} road_id={lane.road_frame_id} "
                  f"skew_s={lane.sync_skew_s} status={'PAIRED' if lane.road_frame_id is not None else 'UNPAIRED'}")
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
    parser.add_argument("--lane-port", type=int, default=7000, help="Port UDP anh camera làn đã khử méo (giu/chuyen lan)")
    parser.add_argument("--road-port", type=int, default=7001, help="Port UDP anh camera nhin duong (YOLO)")
    parser.add_argument("--tcp-host", default="0.0.0.0", help="Dia chi IP bind TCP server (mac dinh 0.0.0.0)")
    parser.add_argument("--tcp-port", type=int, default=6000, help="Port TCP cho xe ket noi de nhan lenh @vcb")
    parser.add_argument("--target-speed", type=float, default=0.3, help="Toc do muc tieu m/s (cho planner)")
    parser.add_argument("--no-send", action="store_true", help="Tat gui lenh TCP")
    parser.add_argument("--no-show", action="store_true", help="Tat hien thi do hoa OpenCV")
    parser.add_argument("--max-frame-skew", type=float, default=0.1,
                        help="Max causal lane/road receive-time skew in seconds (0, 0.5]")
    parser.add_argument("--source-ip", help="Only accept UDP frames from this vehicle IPv4 address")
    args = parser.parse_args()
    if not 0 < args.max_frame_skew <= VISION_TIMEOUT_S:
        parser.error("--max-frame-skew must be in (0, 0.5]")
    if args.lane_port == args.road_port:
        parser.error("Camera ports must be different")

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
    lane_rx = FrameReceiver(args.lane_port, LANE_FRAME_SIZE, source_ip=args.source_ip)
    road_rx = FrameReceiver(args.road_port, ROAD_FRAME_SIZE, source_ip=args.source_ip)
    master.lane_receiver = lane_rx
    master.start(road_rx, draw=show)

    print("\n[HUST ROBOTICS] HE THONG MASTER DA SAN SANG!")
    print(f"[INFO] Anh bird's-eye UDP {args.lane_port} | anh nhin duong (YOLO) UDP {args.road_port} | "
          f"TCP server cho xe ket noi o cong {args.tcp_port} | vong dieu khien {1.0 / DT_SEC:.0f} Hz")

    shown_yolo = None
    try:
        while not master.stop_event.is_set():
            frame, t_recv = lane_rx.latest()   # chờ tối đa 0.1 s, nên ảnh YOLO vẫn được vẽ khi mất camera làn
            if frame is not None:
                _, _, views = master.lane_step(frame, t_recv, draw=show, frame_id=lane_rx.last_frame_id)
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
