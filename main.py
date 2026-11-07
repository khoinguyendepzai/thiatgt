"""LDWS trên PC cho xe mô hình 1:10: nhận ảnh camera từ Jetson, cảnh báo hai mức theo vị trí — LỆCH LÀN (vàng) và CHẠM
VẠCH (đỏ). Không dùng vận tốc hay dự báo.

    main.py          thông số đầu vào (calib, sa bàn và xe, ISO), nhận frame (UDP / video / ảnh), nối hai phần, ghi CSV
    lane_vision.py   phần 1: xử lý ảnh -> thông số hình học của làn (LaneGeometry), vẽ hiển thị
    lane_warning.py  phần 2: LaneGeometry -> LOR -> ngưỡng, debounce, hysteresis -> mức cảnh báo (WarningResult)

Chạy với xe (ảnh JPEG qua UDP 7000):
    python main.py
Chạy lại video hoặc thư mục ảnh đã ghi (thời gian lấy theo fps):
    python main.py --video run.mp4 --csv ketqua.csv
    python main.py --images thu_muc_anh --fps 30 --no-show --csv ketqua.csv
"""

import argparse
import csv
import glob
import os
import socket
import time
from dataclasses import dataclass, field

import cv2
import numpy as np

from lane_vision import LaneVision
from lane_warning import LaneWarning

FRAME_PORT = 7000  # ảnh JPEG 640x480 từ Jetson, mỗi gói UDP một ảnh


# =====================================================================
# 1. THÔNG SỐ ĐẦU VÀO: CALIBRATION, SA BÀN & XE, TIÊU CHÍ ĐÁNH GIÁ
# =====================================================================
@dataclass
class GroundCalibration:
    width: int = 640
    height: int = 480

    # Ma trận Homography: pixel ảnh đã khử méo -> mặt đường (m), gốc tại tâm cầu sau
    # Mảng numpy làm mặc định của dataclass phải qua default_factory (Python >= 3.11 báo lỗi nếu gán thẳng)
    homography: np.ndarray = field(
        default_factory=lambda: np.array(
            [
                [1.73535311e-05, 0.000344733838, 1.09045048],
                [-0.00154327804, 3.85793958e-05, 0.444396494],
                [2.74133999e-05, 0.00411555972, 1.0],
            ],
            dtype=np.float64,
        )
    )

    # Ma trận nội camera (fx, 0, cx; 0, fy, cy; 0, 0, 1) ở cỡ 640x480
    camera_matrix: np.ndarray = field(
        default_factory=lambda: np.array(
            [
                [523.927255, 0.0, 286.149787],
                [0.0, 702.587302, 285.762266],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
    )

    # Hệ số méo ống kính (k1, k2, p1, p2, k3)
    dist_coeffs: np.ndarray = field(
        default_factory=lambda: np.array(
            [-0.35767070, 0.16899265, -0.00088148, 0.00148428, -0.04019552],
            dtype=np.float64,
        )
    )


@dataclass
class TrackAndVehicleParams:
    # Số đo sa bàn từ kTrackParams (main.cpp)
    lane_gap: float = 0.40  # Khoảng trống giữa mép trong 2 vạch (m)
    line_width: float = 0.03  # Bề rộng vạch làn (m)
    dash_length: float = 0.13  # Chiều dài 1 nét đứt (m)
    dash_gap: float = 0.20  # Khe hở giữa 2 nét đứt (m)
    crosswalk_stripe_width: float = 0.05  # Bề rộng sọc đi bộ (m)

    # Kích thước xe từ kLaneParams & kLaneChangeParams (main.cpp)
    wheelbase: float = 0.258  # Chiều dài cơ sở (m)
    vehicle_width: float = 0.21  # Chiều rộng xe (m)
    rear_axle_to_front: float = 0.36  # Cầu sau -> đầu xe (m)

    @property
    def lane_width(self) -> float:
        return self.lane_gap + self.line_width  # 0.43 m


@dataclass
class IsoLdwCriteria:
    """Vùng đặt ngưỡng cảnh báo của ISO 17361, thu nhỏ theo tỉ lệ xe mô hình. Chỉ dùng để đánh giá, không dùng để quyết định.

    ISO 17361 đặt lane boundary tại tâm vạch sơn (mục 3.4) và xét lệch làn tại mép ngoài bánh trước (mục 3.6). Mọi cảnh
    báo phải phát trong vùng giữa vạch cảnh báo sớm nhất (phía trong lane boundary) và vạch cảnh báo muộn nhất (phía
    ngoài); vùng giữa hai vạch sớm nhất là vùng không cảnh báo (mục 3.12, 3.16-3.18). Phạm vi dự án là xe 1:10 trên sa bàn
    có bề rộng làn cho trước, nên các khoảng cách ISO chỉ việc nhân với scale rồi đổi sang LOR bằng chính hình học xe và
    làn trong TrackAndVehicleParams (không cần xe tham chiếu, không có giả thiết đồng dạng).

    Nguồn số liệu (chi tiết trong README và báo cáo PDF):
    - Định nghĩa: ISO 17361:2017 mục 3.4, 3.6, 3.12, 3.16-3.18 (bản xem trước chính thức). Bản 2017 là sửa đổi nhỏ của
      bản 2007 (lời nói đầu của ISO).
    - 0.75 m và 0.3 m: ISO 17361:2007, trích qua báo cáo TRL PPR 374 cho Uỷ ban châu Âu (2008), Bảng 3-1 và mục 3.1.3.2
      (toàn văn ISO phải mua, chưa đối chiếu trực tiếp).
    """

    scale: float = 0.1  # xe mô hình tỉ lệ 1:10
    # Vạch cảnh báo sớm nhất: 0.75 m phía trong lane boundary (ISO cho tốc độ lệch V <= 0.5 m/s; với V lớn hơn vạch này
    # lùi sâu hơn vào trong). Lấy 0.75 m là giá trị chặt nhất, đúng với mọi V nên không phụ thuộc vận tốc
    earliest_line: float = 0.75
    latest_line: float = 0.30  # vạch cảnh báo muộn nhất: 0.3 m phía ngoài lane boundary (xe con)



# =====================================================================
# 2. NỐI PHẦN 1 (XỬ LÝ ẢNH) VỚI PHẦN 2 (CẢNH BÁO)
# =====================================================================
class Ldws:
    """Khởi tạo hai phần từ thông số đầu vào và chạy chúng cho từng frame."""

    def __init__(self, calib=None, params=None, iso=None):
        self.calib = calib if calib is not None else GroundCalibration()
        self.params = params if params is not None else TrackAndVehicleParams()
        self.iso = iso if iso is not None else IsoLdwCriteria()
        self.vision = LaneVision(self.calib, self.params)
        self.warning = LaneWarning(self.params, self.iso)

    def process_raw_frame(self, raw_bgr, t=None):
        """Frame thô -> (dòng log, WarningResult, vis_cam, vis_bev, mask)."""
        geom, images = self.vision.process_raw_frame(raw_bgr, t)
        return self._finish(geom, images)

    def process_undistorted(self, undistorted, t):
        geom, images = self.vision.process_undistorted(undistorted, t)
        return self._finish(geom, images)

    def _finish(self, geom, images):
        t0 = time.perf_counter()
        res = self.warning.update(geom)
        warn_ms = (time.perf_counter() - t0) * 1000.0
        vis_cam, vis_bev = self.vision.draw(images, geom, res.level, self.warning.hud_rows(res, geom.valid))
        return log_row(geom, res, geom.proc_ms + warn_ms), res, vis_cam, vis_bev, images.mask


def log_row(geom, res, proc_ms) -> dict:
    """Một dòng kết quả: thông số hình học của phần 1 và kết quả cảnh báo của phần 2."""
    cm = lambda v: None if v is None else v * 100.0
    return {
        "t": geom.t, "status": res.status, "level": res.level, "direction": res.direction, "reason": res.reason,
        "raw_level": res.raw_level, "lor": res.lor,
        "valid": geom.valid, "n_lines": geom.n_lines, "y_left_cm": cm(geom.y_left), "y_right_cm": cm(geom.y_right),
        "lane_width_cm": geom.lane_width_cm, "offset_cm": geom.offset_cm, "clr_left_cm": geom.clr_left_cm,
        "clr_right_cm": geom.clr_right_cm, "angle_deg": geom.angle_deg, "proc_ms": proc_ms,
    }


# =====================================================================
# 3. NHẬN DỮ LIỆU, GHI KẾT QUẢ, VÒNG LẶP CHÍNH
# =====================================================================
class ResultLog:
    """Ghi kết quả từng frame ra CSV (bỏ qua nếu không có đường dẫn)."""

    FIELDS = ["t", "status", "level", "direction", "reason", "raw_level", "lor", "valid", "n_lines", "y_left_cm",
              "y_right_cm", "lane_width_cm", "offset_cm", "clr_left_cm", "clr_right_cm", "angle_deg", "proc_ms"]

    def __init__(self, path):
        # buffering=1: ghi xuống đĩa từng dòng, tắt ngang chương trình vẫn còn dữ liệu
        self.file = open(path, "w", newline="", encoding="utf-8", buffering=1) if path else None
        self.writer = csv.DictWriter(self.file, self.FIELDS, extrasaction="ignore") if self.file else None
        if self.writer:
            self.writer.writeheader()

    def write(self, row: dict):
        if not self.writer:
            return
        out = {}
        for k in self.FIELDS:
            v = row.get(k)
            out[k] = "" if v is None else (f"{v:.4f}" if isinstance(v, float) else v)
        self.writer.writerow(out)

    def close(self):
        if self.file:
            self.file.close()


def show_frames(vis_cam, vis_bev, mask, wait_ms=1) -> bool:
    """Hiện 3 cửa sổ. Trả về False khi người dùng bấm q/ESC."""
    cv2.imshow("Camera Undistorted + LDWS Overlay", vis_cam)
    cv2.imshow("Bird's-Eye View (BEV)", vis_bev)
    cv2.imshow("Filtered Lane Mask", mask)
    key = cv2.waitKey(wait_ms) & 0xFF
    if key == ord(" "):  # tạm dừng, bấm phím bất kỳ để chạy tiếp
        key = cv2.waitKey(0) & 0xFF
    return key not in (27, ord("q"))


def run_udp_ldws_receiver(args):
    """Nhận frame thô 640x480 (JPEG, UDP 7000) từ Jetson. Chỉ nhận ảnh, không nhận vận tốc."""
    ldws = Ldws()
    log = ResultLog(args.csv)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    sock.bind((args.listen_ip, args.port))
    sock.settimeout(0.5)
    print(f"[PC LDWS] Dang lang nghe anh tho 640x480 tai UDP {args.listen_ip}:{args.port}... "
          "(Ctrl+C hoac q de thoat)", flush=True)

    try:
        while True:
            try:
                packet, _ = sock.recvfrom(65535)
            except socket.timeout:
                if not args.no_show and (cv2.waitKey(1) & 0xFF) in (27, ord("q")):
                    break
                continue
            now = time.perf_counter()
            # Giải mã buffer JPEG nhận qua UDP thành mảng BGR 640x480
            raw_frame = cv2.imdecode(np.frombuffer(packet, dtype=np.uint8), cv2.IMREAD_COLOR)
            if raw_frame is None:
                continue
            if (raw_frame.shape[1], raw_frame.shape[0]) != (640, 480):
                raw_frame = cv2.resize(raw_frame, (640, 480))

            row, _, vis_cam, vis_bev, mask = ldws.process_raw_frame(raw_frame, now)
            log.write(row)
            if not args.no_show and not show_frames(vis_cam, vis_bev, mask):
                break
    finally:
        sock.close()
        log.close()
        cv2.destroyAllWindows()


def iter_replay(args):
    """(t, frame) từ video hoặc thư mục/mẫu glob ảnh. t tính theo fps để debounce và bộ lọc trượt chạy như thật."""
    if args.video:
        cap = cv2.VideoCapture(args.video)
        if not cap.isOpened():
            raise SystemExit(f"Khong mo duoc video {args.video}")
        fps = cap.get(cv2.CAP_PROP_FPS) or args.fps
        i = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            yield i / fps, frame
            i += 1
        cap.release()
        return
    pattern = args.images
    if os.path.isdir(pattern):
        paths = sorted(p for ext in ("jpg", "jpeg", "png", "bmp")
                       for p in glob.glob(os.path.join(pattern, f"*.{ext}")))
    else:
        paths = sorted(glob.glob(pattern))
    if not paths:
        raise SystemExit(f"Khong tim thay anh: {pattern}")
    for i, p in enumerate(paths):
        frame = cv2.imread(p)
        if frame is not None:
            yield i / args.fps, frame


def run_replay(args):
    ldws = Ldws()
    log = ResultLog(args.csv)
    counts, proc = [0, 0, 0], []
    try:
        for t, frame in iter_replay(args):
            if (frame.shape[1], frame.shape[0]) != (640, 480):
                frame = cv2.resize(frame, (640, 480))
            row, res, vis_cam, vis_bev, mask = ldws.process_raw_frame(frame, t)
            log.write(row)
            counts[res.level] += 1
            proc.append(row["proc_ms"])
            if not args.no_show and not show_frames(vis_cam, vis_bev, mask, args.delay):
                break
    finally:
        log.close()
        cv2.destroyAllWindows()
    n = sum(counts)
    if n:
        print(f"[PC LDWS] {n} frame | SAFE {counts[0]} | lech lan {counts[1]} | cham vach {counts[2]} | "
              f"xu ly trung binh {np.mean(proc):.1f} ms, lon nhat {np.max(proc):.1f} ms")


def main():
    parser = argparse.ArgumentParser(description="Canh bao lech lan (LDWS) tren PC cho xe mo hinh")
    src = parser.add_mutually_exclusive_group()
    src.add_argument("--video", help="chay lai tu file video thay vi nhan UDP")
    src.add_argument("--images", help="chay lai tu thu muc anh hoac mau glob (sap xep theo ten)")
    parser.add_argument("--fps", type=float, default=30.0, help="fps cua chuoi anh (hoac video khong ghi fps)")
    parser.add_argument("--delay", type=int, default=1, help="ms cho giua 2 frame khi chay lai co hien thi")
    parser.add_argument("--csv", help="ghi ket qua tung frame ra file CSV")
    parser.add_argument("--no-show", action="store_true", help="khong mo cua so hien thi")
    parser.add_argument("--listen-ip", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=FRAME_PORT, help="cong UDP nhan anh JPEG")
    args = parser.parse_args()

    try:
        if args.video or args.images:
            run_replay(args)
        else:
            run_udp_ldws_receiver(args)
    except KeyboardInterrupt:
        print("[PC LDWS] Dung.")


if __name__ == "__main__":
    main()
