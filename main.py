"""
MAIN MASTER - HUST Robotics 2026
Tích hợp 4 luồng xử lý: YOLO, LDWS, Lane Change Planner, Vehicle Dynamics.
Bao gồm TCP Server giao tiếp chuẩn C++ (gửi lệnh @vcb) xuống Raspberry Pi 5.
"""

import argparse
import socket
import time
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
    DriveDirSelector,
    Input,
    PedalSerialReader,
    RawVehicleController,
    apply_pedal_interlock,
)

# --- 5. MODULE TRUYỀN THÔNG (TCP Server giao tiếp Pi 5) ---
class JetsonSender:
    """Đóng vai trò TCP Server chờ Pi 5 kết nối để nhận lệnh điều khiển"""
    def __init__(self, port=6000):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(("0.0.0.0", port))
        self.server.listen(1)
        self.server.settimeout(0.02)  # Non-blocking để không làm trễ chu kỳ 50Hz
        self.client = None

    def send(self, out):
        # 1. Chờ Pi 5 kết nối
        if not self.client:
            try:
                self.client, addr = self.server.accept()
                self.client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                print(f"[TCP] Pi 5 đã kết nối thành công từ {addr}")
            except socket.timeout:
                return  # Chưa có Pi 5, bỏ qua để xử lý hình ảnh tiếp

        # 2. Đóng gói lệnh theo chuẩn C++: @vcb:<speed>;<steer>;<dir>;;\r\n
        msg = f"@vcb:{out.speedCmd:.3f};{out.steerCmd:.3f};{out.driveDirCmd};;\r\n"
        
        # 3. Gửi lệnh đi
        try:
            self.client.sendall(msg.encode("ascii"))
        except OSError:
            print("[TCP] Mất kết nối tới Pi 5. Đang chờ kết nối lại...")
            self.client.close()
            self.client = None

    def close(self):
        if self.client:
            self.client.close()
        self.server.close()

# --- 6. HỆ THỐNG ĐIỀU KHIỂN TRUNG TÂM ---
class AutonomousMasterController:
    def __init__(self, args):
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
        
        # Gọi TCP Server ở cổng 6000
        self.sender = JetsonSender(port=6000) if not args.no_send else None

    def process_step(self, raw_frame: np.ndarray, pedal_data: tuple, t_now: float):
        enc, ga, lui, phanh, connected = pedal_data
        ga, lui, phanh = apply_pedal_interlock(ga, lui, phanh)
        drive_dir, _ = self.dir_selector.update(ga, lui, self.vehicle_ctrl.speed)

        distances, yolo_frame = self.detector.process_frame(raw_frame, t_now)
        
        geom, images = self.vision.process_raw_frame(raw_frame, t_now)
        warn_res = self.warning.update(geom)

        valid_dists = [d for d in [distances["nguoi_di_bo"], distances["xe_may"], distances["o_to"]] if d >= 0]
        min_front_dist = min(valid_dists) if valid_dists else 2.5

        plan_res = self.planner.propose(
            bev=images.bev,
            front_distance_m=min_front_dist,
            distance_age_s=0.0,
        )

        sys_input = Input(
            encoder=enc, ga=ga, lui=lui, phanh=phanh, driveDir=drive_dir,
            distance_xe_may=distances["xe_may"],
            distance_nguoi_di_bo=distances["nguoi_di_bo"],
            distance_o_to=distances["o_to"],
        )
        sys_output = self.vehicle_ctrl.update(sys_input)

        if plan_res.selected and plan_res.selected.direction != "KEEP":
            if plan_res.selected.direction == "LEFT":
                sys_output.steerCmd = max(-20.0, sys_output.steerCmd - 10.0)
            elif plan_res.selected.direction == "RIGHT":
                sys_output.steerCmd = min(20.0, sys_output.steerCmd + 10.0)

        if self.sender:
            self.sender.send(sys_output)

        yolo_frame = self.detector.draw_bboxes(yolo_frame)
        vis_cam, vis_bev_ldws = self.vision.draw(images, geom, warn_res.level, self.warning.hud_rows(warn_res, geom.valid))

        return sys_output, warn_res, plan_res, yolo_frame, vis_cam, vis_bev_ldws, plan_res.annotated_bev

def main():
    parser = argparse.ArgumentParser(description="Hệ thống Xe Điều Khiển - HUST Robotics 2026")
    parser.add_argument("--serial-port", default="COM3", help="Cổng Serial nối Arduino (Bàn đạp)")
    parser.add_argument("--video-port", type=int, default=7000, help="Port nhận luồng Video UDP")
    parser.add_argument("--target-speed", type=float, default=0.3, help="Tốc độ mục tiêu m/s")
    parser.add_argument("--no-send", action="store_true", help="Tắt gửi lệnh TCP")
    parser.add_argument("--no-show", action="store_true", help="Tắt hiển thị đồ họa OpenCV")
    args = parser.parse_args()

    master = AutonomousMasterController(args)

    try:
        pedals = PedalSerialReader(args.serial_port)
    except Exception as e:
        print(f"[WARN] Không kết nối được Serial Bàn đạp: {e}. Dùng giá trị mặc định.")
        pedals = None

    video_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    video_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    video_sock.bind(("0.0.0.0", args.video_port))
    video_sock.settimeout(0.5)

    print("\n[HUST ROBOTICS] HỆ THỐNG MASTER ĐÃ SẴN SÀNG!")
    print(f"[INFO] Lắng nghe Video ở UDP {args.video_port} | Mở TCP Server chờ Pi 5 kết nối ở cổng 6000")

    try:
        while True:
            loop_start = time.perf_counter()

            try:
                packet, _ = video_sock.recvfrom(65535)
                raw_frame = cv2.imdecode(np.frombuffer(packet, dtype=np.uint8), cv2.IMREAD_COLOR)
            except socket.timeout:
                continue

            if raw_frame is None:
                continue

            pedal_data = pedals.read() if pedals else (0, 0.0, 0.0, 0.0, False)
            out, warn, plan, f_yolo, f_cam, f_ldws, f_plan = master.process_step(raw_frame, pedal_data, loop_start)

            print(f"[T={loop_start:.2f}s] Speed: {out.speedCmd:.2f}m/s | Steer: {out.steerCmd:.1f}° | "
                  f"LOR: {warn.lor:.2f} | Plan: {plan.selected.direction if plan.selected else 'NONE'} | "
                  f"Limit Reason: {out.limitReason}")

            if not args.no_show:
                cv2.imshow("1. YOLO Detections", f_yolo)
                cv2.imshow("2. LDWS Camera Overlay", f_cam)
                cv2.imshow("3. BEV Lane Tracking", f_ldws)
                cv2.imshow("4. BEV Lane Change Planner", f_plan)
                if (cv2.waitKey(1) & 0xFF) in (27, ord("q")):
                    break

            elapsed = time.perf_counter() - loop_start
            time.sleep(max(0.0, 0.02 - elapsed))

    except KeyboardInterrupt:
        print("\n[INFO] Đã dừng vòng lặp chính.")
    finally:
        video_sock.close()
        if master.sender:
            master.sender.close()
        if pedals:
            pedals.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
