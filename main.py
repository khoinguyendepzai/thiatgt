"""
MAIN FINAL - HUST Robotics 2026
Tích hợp toàn diện 4 luồng xử lý:
1. Nhận diện vật cản & Đo khoảng cách (YOLOv8 + Pinhole Camera).
2. Bám làn & Cảnh báo lệch làn (LDWS).
3. Lập quỹ đạo chuyển làn tránh vật cản (Lane Change Planner).
4. Tính toán Động lực học & Phanh tự động & Giao tiếp Jetson/STM32.
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
from main_4 import CAMERA_LOST, NO_OBJECT

# --- 4. IMPORT MODULE ĐIỀU KHIỂN & GIAO TIẾP ---
from main_4 import (
    DriveDirSelector,
    Input,
    JetsonSender,
    PedalSerialReader,
    RawVehicleController,
    apply_pedal_interlock,
)

class AutonomousMasterController:
    """Bộ não trung tâm điều phối toàn bộ thuật toán tự hành."""

    def __init__(self, args):
        self.args = args

        # 1. Cấu hình hình học sa bàn và xe mô hình
        self.calib = GroundCalibration()
        self.params = TrackAndVehicleParams()
        self.iso = IsoLdwCriteria()

        # 2. Khởi tạo thuật toán Bám làn
        self.vision = LaneVision(self.calib, self.params)
        self.warning = LaneWarning(self.params, self.iso)

        # 3. Khởi tạo thuật toán Chuyển làn (Đồng bộ tỷ lệ m/px từ BEV)
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

        # 4. Khởi tạo thuật toán Nhận diện YOLO
        self.detector = ObstacleDetector()

        # 5. Khởi tạo thuật toán Động lực học & Phanh tự động
        self.vehicle_ctrl = RawVehicleController()
        self.dir_selector = DriveDirSelector()
        self.sender = JetsonSender(args.jetson_ip, args.jetson_port) if not args.no_send else None

    def process_step(self, raw_frame: np.ndarray, pedal_data: tuple, t_now: float):
        """Thực thi một chu kỳ logic (nhận ảnh -> phân tích -> ra lệnh)."""
        
        # BƯỚC 1: Xử lý tín hiệu bàn đạp & Vô lăng
        enc, ga, lui, phanh, connected = pedal_data
        ga, lui, phanh = apply_pedal_interlock(ga, lui, phanh)
        drive_dir, _ = self.dir_selector.update(ga, lui, self.vehicle_ctrl.speed)

        # BƯỚC 2: Nhận diện vật cản & Tính khoảng cách
        distances, yolo_frame = self.detector.process_frame(raw_frame, t_now)
        
        # BƯỚC 3: Xử lý Bám làn (BEV & Polynomial Fit)
        geom, images = self.vision.process_raw_frame(raw_frame, t_now)
        warn_res = self.warning.update(geom)

        # Lọc ra vật cản gần nhất trước đầu xe để đưa vào bộ Planner
        valid_dists = [d for d in [distances["nguoi_di_bo"], distances["xe_may"], distances["o_to"]] if d >= 0]
        min_front_dist = min(valid_dists) if valid_dists else 2.5

        # BƯỚC 4: Lập quỹ đạo tránh vật cản (Frenet)
        plan_res = self.planner.propose(
            bev=images.bev,
            front_distance_m=min_front_dist,
            distance_age_s=0.0,
        )

        # BƯỚC 5: Tổng hợp thông số đưa vào mô hình Động lực học
        sys_input = Input(
            encoder=enc,
            ga=ga,
            lui=lui,
            phanh=phanh,
            driveDir=drive_dir,
            distance_xe_may=distances["xe_may"],
            distance_nguoi_di_bo=distances["nguoi_di_bo"],
            distance_o_to=distances["o_to"],
        )
        
        # Tính toán vận tốc giới hạn và góc lái gốc
        sys_output = self.vehicle_ctrl.update(sys_input)

        # Nếu quỹ đạo an toàn nhất là chuyển làn -> can thiệp bẻ lái tự động
        if plan_res.selected and plan_res.selected.direction != "KEEP":
            if plan_res.selected.direction == "LEFT":
                sys_output.steerCmd = max(-20.0, sys_output.steerCmd - 10.0) # Bẻ trái
            elif plan_res.selected.direction == "RIGHT":
                sys_output.steerCmd = min(20.0, sys_output.steerCmd + 10.0)  # Bẻ phải

        # BƯỚC 6: Gửi lệnh đóng gói (steer, speed, dir) xuống Jetson
        if self.sender:
            self.sender.send(sys_output)

        # BƯỚC 7: Vẽ đồ họa Debug
        yolo_frame = self.detector.draw_bboxes(yolo_frame)
        vis_cam, vis_bev_ldws = self.vision.draw(images, geom, warn_res.level, self.warning.hud_rows(warn_res, geom.valid))

        return sys_output, warn_res, plan_res, yolo_frame, vis_cam, vis_bev_ldws, plan_res.annotated_bev

def main():
    parser = argparse.ArgumentParser(description="Hệ thống Xe Tự Hành - HUST Robotics 2026")
    parser.add_argument("--serial-port", default="COM3", help="Cổng Serial nối Arduino (Bàn đạp)")
    parser.add_argument("--jetson-ip", default="192.168.1.124", help="IP Jetson Nano")
    parser.add_argument("--jetson-port", type=int, default=5005, help="Port nhận điều khiển UDP")
    parser.add_argument("--video-port", type=int, default=7000, help="Port nhận luồng Video UDP")
    parser.add_argument("--target-speed", type=float, default=0.3, help="Tốc độ mục tiêu m/s")
    parser.add_argument("--no-send", action="store_true", help="Tắt gửi lệnh UDP")
    parser.add_argument("--no-show", action="store_true", help="Tắt hiển thị đồ họa OpenCV")
    args = parser.parse_args()

    master = AutonomousMasterController(args)

    # Khởi tạo kết nối Serial đọc PID bàn đạp/vô lăng
    try:
        pedals = PedalSerialReader(args.serial_port)
    except Exception as e:
        print(f"[WARN] Không kết nối được Serial Bàn đạp: {e}. Dùng giá trị mặc định.")
        pedals = None

    # Khởi tạo luồng nhận Video UDP
    video_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    video_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    video_sock.bind(("0.0.0.0", args.video_port))
    video_sock.settimeout(0.5)

    print("\n[HUST ROBOTICS] HỆ THỐNG MASTER ĐÃ SẴN SÀNG!")
    print(f"[INFO] Lắng nghe Video ở cổng UDP {args.video_port} | Gửi điều khiển xuống {args.jetson_ip}:{args.jetson_port}")

    try:
        while True:
            loop_start = time.perf_counter()

            # 1. Nhận khung hình Camera
            try:
                packet, _ = video_sock.recvfrom(65535)
                raw_frame = cv2.imdecode(np.frombuffer(packet, dtype=np.uint8), cv2.IMREAD_COLOR)
            except socket.timeout:
                continue

            if raw_frame is None:
                continue

            # 2. Đọc cảm biến phần cứng
            pedal_data = pedals.read() if pedals else (0, 0.0, 0.0, 0.0, False)

            # 3. Kích hoạt toàn bộ Pipeline
            out, warn, plan, f_yolo, f_cam, f_ldws, f_plan = master.process_step(raw_frame, pedal_data, loop_start)

            # Log hệ thống
            print(f"[T={loop_start:.2f}s] Speed: {out.speedCmd:.2f}m/s | Steer: {out.steerCmd:.1f}° | "
                  f"LOR: {warn.lor:.2f} | Plan: {plan.status} ({plan.selected.direction if plan.selected else 'NONE'}) | "
                  f"Limit Reason: {out.limitReason}")

            # 4. Render đồ họa
            if not args.no_show:
                cv2.imshow("1. YOLO Detections", f_yolo)
                cv2.imshow("2. LDWS Camera Overlay", f_cam)
                cv2.imshow("3. BEV Lane Tracking", f_ldws)
                cv2.imshow("4. BEV Lane Change Planner", f_plan)
                if (cv2.waitKey(1) & 0xFF) in (27, ord("q")):
                    break

            # 5. Đồng bộ chu kỳ 50Hz (20ms) để khớp với hệ thống STM32
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
