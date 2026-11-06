"""LDWS trên PC cho xe mô hình: nhận ảnh camera và vận tốc dọc từ Jetson, dò làn, bám vạch bằng Kalman, dự báo thời
gian tới lúc cắt vạch (TLC) và cảnh báo lệch làn 2 cấp.

Chạy với xe (ảnh JPEG qua UDP 7000, vận tốc dọc qua UDP 7001):
    python main.py
Chạy lại video hoặc thư mục ảnh đã ghi (thời gian lấy theo fps, vận tốc dọc giả định không đổi):
    python main.py --video run.mp4 --speed 0.5 --csv ketqua.csv
    python main.py --images thu_muc_anh --fps 30 --no-show --csv ketqua.csv
"""

import argparse
import csv
import glob
import os
import socket
import struct
import time
from dataclasses import dataclass, field

import cv2
import numpy as np

FRAME_PORT = 7000  # ảnh JPEG 640x480 từ Jetson, mỗi gói UDP một ảnh
SPEED_PORT = 7001  # vận tốc dọc xe (m/s) từ Jetson, mỗi gói UDP một số


# =====================================================================
# 1. CẤU HÌNH CALIBRATION & SA BÀN TỪ MAIN.CPP & LANEDETECTOR.CPP
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


class LaneLine:
    """Một vạch dò được trên ảnh BEV trong frame hiện tại."""

    def __init__(self):
        self.points = None
        self.ok = False
        self.coeffs = np.zeros(
            3, dtype=np.float64
        )  # x = a*y^2 + b*y + c (trên ảnh BEV)
        self.rows = (0, 0)  # (min_y, max_y)
        self.tracked = False  # True: tìm quanh vạch của frame trước, False: tìm lại từ histogram


class LineTrack:
    """Bộ lọc Kalman bám một vạch qua các frame (Tài liệu 3).

    Trạng thái [y, ẏ]: y là vị trí ngang của vạch ngang đầu xe (m, hệ toạ độ xe, + sang trái), ẏ là tốc độ thay đổi
    của y (m/s). Xe trôi sang trái thì mọi vạch trôi sang phải trong hệ toạ độ xe (ẏ < 0), nên vạch trái đang tiến lại
    gần xe khi ẏ < 0, vạch phải khi ẏ > 0. Mô hình vận tốc không đổi, nhiễu là gia tốc ngang ngẫu nhiên.
    """

    def __init__(self, y, t, coeffs, rows):
        self.x = np.array([y, 0.0], dtype=np.float64)
        self.P = np.diag([0.01**2, 0.10**2])
        self.t = t
        self.last_update = t
        self.rejects = 0  # số frame liên tiếp phép đo trực tiếp bị loại vì lệch dự đoán quá xa
        self.coeffs = np.array(coeffs, dtype=np.float64)  # đa thức BEV gần nhất: vùng tìm vạch cho frame sau
        self.rows = rows
        self.seen = True  # frame này có dò thấy trực tiếp vạch này không

    @property
    def y(self) -> float:
        return float(self.x[0])

    @property
    def vy(self) -> float:
        return float(self.x[1])

    def predict(self, t, accel_std):
        dt = t - self.t
        if dt <= 0:
            return
        F = np.array([[1.0, dt], [0.0, 1.0]])
        Q = accel_std**2 * np.array(
            [[dt**4 / 4.0, dt**3 / 2.0], [dt**3 / 2.0, dt**2]]
        )
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.t = t

    def innovation_std(self, meas_std) -> float:
        return float(np.sqrt(self.P[0, 0] + meas_std**2))

    def update(self, z, meas_std, h):
        """h = (1, 0): đo vị trí y; h = (0, 1): đo tốc độ ẏ."""
        h = np.asarray(h, dtype=np.float64)
        s = h @ self.P @ h + meas_std**2
        k = self.P @ h / s
        self.x = self.x + k * (z - h @ self.x)
        self.P = (np.eye(2) - np.outer(k, h)) @ self.P


# =====================================================================
# 2. BỘ XỬ LÝ DÒ LÀN & CẢNH BÁO LỆCH LÀN TRÊN PC
# =====================================================================
class CalibratedLDWS:
    # Vùng mặt đường Bird's-Eye View (m, hệ tọa độ cầu sau) từ laneDetector.cpp
    BEV_NEAR = 0.45
    BEV_FAR = 1.05
    BEV_HALF_WIDTH = 0.50
    BEV_DEPTH = BEV_FAR - BEV_NEAR  # 0.60 m
    BEV_WIDTH = 2.0 * BEV_HALF_WIDTH  # 1.00 m

    # Ngưỡng lọc ảnh HSV & Top-Hat từ laneDetector.cpp
    WHITE_MIN_V = 150
    WHITE_MAX_S = 70
    MIN_LINE_CONTRAST = 100
    NUM_WINDOWS = 15
    LANE_WIDTH_ALPHA = 0.1

    # Ngưỡng cảnh báo lệch làn. Mọi khoảng hở đo tại hai góc đầu xe (x = rear_axle_to_front, y = ±vehicle_width/2).
    LOR_LEVEL1_TH = 0.45  # Cảnh báo sớm (Nhắc nhở - Vàng)
    LOR_LEVEL2_TH = 0.60  # Cảnh báo khẩn cấp (Lệch làn - Đỏ, chuẩn Tài liệu 2)
    SAFE_ZONE_TH = 0.80  # T_H = 80% nửa độ rộng làn (Tài liệu 2)
    TLC_LEVEL1_S = 1.8  # 1.0 s <= TLC < 1.8 s: cảnh báo sớm (Tài liệu 3)
    TLC_LEVEL2_S = 1.0  # TLC < 1.0 s: cảnh báo khẩn cấp (Tài liệu 3)
    CLEARANCE_WARN_M = 0.025  # Góc đầu xe cách mép vạch dưới 2.5 cm: cảnh báo sớm
    V_LAT_MIN = 0.01  # m/s: trôi ngang chậm hơn mức này coi như nhiễu, không tính TLC
    NARROW_V_MIN = 0.02  # m/s: hai vạch cùng tiến vào giữa nhanh hơn mức này mới coi là làn hẹp lại
    SPEED_MIN_WARN = 0.10  # m/s: xe gần như đứng yên thì không cảnh báo
    DEBOUNCE_TIME_S = 0.20  # Duy trì lỗi > 200ms mới kích hoạt (Tài liệu 3)
    RELEASE_TIME_S = 0.30  # Trở về vùng an toàn > 300ms mới tắt (Tài liệu 3)
    # Hysteresis ngưỡng (Tài liệu 3): đang đếm debounce hoặc đang cảnh báo ở cấp nào thì ngưỡng của cấp đó được nới
    # ra, để nhiễu ước lượng quanh ngưỡng không làm cảnh báo chập chờn hay đặt lại bộ đếm 200 ms
    HYST_TLC = 1.25  # nhân ngưỡng TLC
    HYST_CLEARANCE_M = 0.005  # cộng vào ngưỡng khoảng hở
    HYST_LOR = 0.05  # trừ vào ngưỡng LOR

    # Bám vạch bằng Kalman (Tài liệu 3)
    # m/s²: độ biến thiên của vận tốc ngang. Chọn bằng giả lập (4 chuỗi nhiễu x trôi 2/3/5 cm/s, đứng yên, lái sửa):
    # 0.2 cho thời điểm cảnh báo đều nhất, không báo nhầm, nhả cảnh báo 0.1 s sau khi an toàn
    KF_ACCEL_STD = 0.2
    KF_MEAS_STD = 0.005  # m: sai số đo vị trí vạch dò trực tiếp
    KF_INFER_STD = 0.015  # m: sai số vạch suy từ vạch bên kia + bề rộng làn
    KF_FAR_STD = 0.03  # m: sai số vạch chỉ thấy ở xa, phải ngoại suy dài về đầu xe
    LINE_FIT_SPAN_M = 0.25  # m: khớp thẳng qua đoạn vạch dài chừng này gần xe nhất để ngoại suy về đầu xe
    MAX_EXTRAP_M = 0.45  # m: điểm vạch gần nhất thấy được cách đầu xe xa hơn mức này thì coi là kém tin cậy
    KF_GATE_M = 0.04  # m: loại phép đo lệch dự đoán quá mức này (hoặc 4 sigma)
    KF_MAX_REJECTS = 5  # bị loại liên tiếp chừng này frame thì khởi tạo lại track theo phép đo
    TRACK_TIMEOUT_S = 0.5  # không cập nhật được quá lâu thì bỏ track
    LABEL_HYST_M = 0.02  # vạch phải vượt qua trục giữa xe 2 cm mới đổi nhãn trái <-> phải
    # Đo thêm ẏ = v·(độ dốc vạch) từ vận tốc dọc. Cần góc lệch đúng: CHỈ bật sau khi calib lại homography.
    FUSE_SPEED_HEADING = False
    HEADING_STD = np.tan(np.radians(3.0))

    def __init__(self, calib=None, params=None):
        self.calib = calib if calib is not None else GroundCalibration()
        self.params = params if params is not None else TrackAndVehicleParams()
        self.w = self.calib.width
        self.h = self.calib.height

        self.px_per_m_x = self.w / self.BEV_WIDTH  # 640 px/m
        self.px_per_m_y = self.h / self.BEV_DEPTH  # 800 px/m

        # 1. Tính các ngưỡng hình học từ make_thresholds() trong laneDetector.cpp
        self._make_thresholds()

        # 2. Khởi tạo bản đồ khử méo ống kính (Undistort Map)
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            self.calib.camera_matrix,
            self.calib.dist_coeffs,
            None,
            self.calib.camera_matrix,
            (self.w, self.h),
            cv2.CV_16SC2,
        )

        # 3. Khởi tạo ma trận IPM và ma trận chuyển đổi BEV <-> Mặt đường (m)
        self._init_ipm()

        # 4. Tạo mặt nạ vùng nhìn thấy hợp lệ (visible_ trong laneDetector.cpp)
        white = np.full((self.h, self.w), 255, dtype=np.uint8)
        undist_w = cv2.remap(
            white, self.map1, self.map2, cv2.INTER_NEAREST
        )
        vis_raw = cv2.warpPerspective(
            undist_w, self.ipm_matrix, (self.w, self.h), flags=cv2.INTER_NEAREST
        )
        shrink_k = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (2 * self.px_x(self.th_vis_margin) + 1, 2 * self.px_y(self.th_vis_margin) + 1),
        )
        self.visible_mask = cv2.erode(vis_raw, shrink_k)

        # Biến trạng thái theo dõi làn và cảnh báo
        self.lane_width_m = self.params.lane_width
        self.tracks = {"left": None, "right": None}
        self.warning_level = 0  # 0: An toàn, 1: Cảnh báo sớm, 2: Khẩn cấp
        self.warning_side = None  # "LEFT" / "RIGHT" khi đang cảnh báo
        self.level_since = {1: None, 2: None}  # thời điểm điều kiện cấp 1 / cấp 2 bắt đầu đúng liên tục
        self.safe_start_time = None

    def px_x(self, m: float) -> int:
        return int(round(m * self.px_per_m_x))

    def px_y(self, m: float) -> int:
        return int(round(m * self.px_per_m_y))

    def px_area(self, m2: float) -> float:
        return m2 * self.px_per_m_x * self.px_per_m_y

    # Ảnh BEV là ảnh co giãn đều của mặt đường: cột -> y (m, + trái), hàng -> x (m, tới trước)
    def x_to_row(self, x_m: float) -> float:
        return (self.BEV_FAR - x_m) * self.px_per_m_y

    def col_to_y(self, col: float) -> float:
        return (self.w / 2.0 - col) / self.px_per_m_x

    def _make_thresholds(self):
        t = self.params
        lane = t.lane_width
        self.th_vis_margin = 0.5 * t.line_width
        self.th_tophat_size = 2.0 * t.line_width
        self.th_max_line_w = t.line_width + 0.7 * (
            t.crosswalk_stripe_width - t.line_width
        )
        self.th_min_line_w = 0.5 * t.line_width
        self.th_max_run_len = 4.0 * t.line_width
        self.th_min_blob_area = 0.9 * t.line_width * t.line_width
        self.th_min_blob_len = 0.75 * t.dash_length

        self.th_win_half_w = 3.0 * t.line_width
        self.th_min_line_area = 0.15 * t.line_width * t.line_width
        self.th_min_lane_len = 0.9 * t.dash_length

        self.th_min_lane_w = 0.67 * lane
        self.th_max_lane_w = 1.33 * lane
        self.th_same_line_dist = 0.5 * self.th_min_lane_w

    def _init_ipm(self):
        """Tính ma trận IPM từ nghịch đảo Homography mặt đất đúng theo init_ipm() trong C++."""
        ground_corners = np.array(
            [
                [self.BEV_FAR, self.BEV_HALF_WIDTH],  # Top-Left (y > 0: bên trái)
                [self.BEV_NEAR, self.BEV_HALF_WIDTH],  # Bottom-Left
                [self.BEV_FAR, -self.BEV_HALF_WIDTH],  # Top-Right (y < 0: bên phải)
                [self.BEV_NEAR, -self.BEV_HALF_WIDTH],  # Bottom-Right
            ],
            dtype=np.float32,
        ).reshape(-1, 1, 2)

        h_inv = np.linalg.inv(self.calib.homography)
        src_points = cv2.perspectiveTransform(ground_corners, h_inv).reshape(
            4, 2
        ).astype(np.float32)

        dst_points = np.array(
            [[0, 0], [0, self.h], [self.w, 0], [self.w, self.h]],
            dtype=np.float32,
        )

        self.ipm_matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        self.ipm_inv = np.linalg.inv(self.ipm_matrix)
        # Ma trận chuyển từ pixel BEV sang tọa độ mặt đường (m): bev_to_ground()
        self.bev_to_ground_mat = self.calib.homography @ self.ipm_inv

    # -----------------------------------------------------------------
    # BƯỚC 1: TẠO MASK NHỊ PHÂN (Từ process_mask trong laneDetector.cpp)
    # -----------------------------------------------------------------
    def process_mask(self, bird_eye_view: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(bird_eye_view, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv, (0, 0, self.WHITE_MIN_V), (180, self.WHITE_MAX_S, 255)
        )

        # Lọc Top-Hat trên kênh V để giữ vạch hẹp, loại bỏ vùng lóa rộng
        value = hsv[:, :, 2]
        th_size = (
            max(1, self.px_x(self.th_tophat_size)),
            max(1, self.px_y(self.th_tophat_size)),
        )
        contrast = cv2.morphologyEx(
            value,
            cv2.MORPH_TOPHAT,
            cv2.getStructuringElement(cv2.MORPH_RECT, th_size),
        )
        mask[contrast < self.MIN_LINE_CONTRAST] = 0

        # Xóa mảng dày hơn maxLineWidth (sọc đi bộ, lóa đặc) trên ảnh thu nhỏ 1/2
        half = cv2.resize(
            mask,
            (self.w // 2, self.h // 2),
            interpolation=cv2.INTER_NEAREST,
        )
        thick_sz = (
            max(1, self.px_x(self.th_max_line_w) // 2),
            max(1, self.px_y(self.th_max_line_w) // 2),
        )
        thick = cv2.morphologyEx(
            half,
            cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, thick_sz),
        )
        thick = cv2.resize(
            thick, (self.w, self.h), interpolation=cv2.INTER_NEAREST
        )
        mask[thick > 0] = 0

        # Xóa nét mảnh hơn minLineWidth (1.5 cm)
        thin_sz = (
            max(1, self.px_x(self.th_min_line_w)),
            max(1, self.px_y(self.th_min_line_w)),
        )
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, thin_sz),
        )

        # Xóa các đoạn trắng ngang dài hơn maxRunLength (12 cm - vạch dừng)
        max_run_px = self.px_x(self.th_max_run_len)
        horiz_k = cv2.getStructuringElement(cv2.MORPH_RECT, (max_run_px + 1, 1))
        horiz_runs = cv2.morphologyEx(mask, cv2.MORPH_OPEN, horiz_k)
        mask[horiz_runs > 0] = 0

        # Lọc Blob theo diện tích và độ dài đường chéo
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        filtered = np.zeros_like(mask)
        min_area = self.px_area(self.th_min_blob_area)

        for cnt in contours:
            if cv2.contourArea(cnt) < min_area:
                continue
            _, _, bw, bh = cv2.boundingRect(cnt)
            if (
                np.hypot(bw / self.px_per_m_x, bh / self.px_per_m_y)
                < self.th_min_blob_len
            ):
                continue
            cv2.drawContours(filtered, [cnt], -1, 255, -1)

        return filtered

    # -----------------------------------------------------------------
    # BƯỚC 2: TÌM VẠCH (QUANH VẠCH FRAME TRƯỚC HOẶC SLIDING WINDOW) & KHỚP ĐA THỨC BẬC 2
    # -----------------------------------------------------------------
    def detect_lines(self, mask: np.ndarray) -> dict:
        """Trả về {"left": LaneLine, "right": LaneLine}.

        Bên nào đang có track thì tìm điểm trắng quanh đa thức của track (giữ đúng danh tính vạch kể cả khi vạch đi
        qua trục giữa ảnh lúc xe cắt vạch). Bên nào không có track, hoặc tìm quanh track thất bại, thì dò lại bằng
        histogram + cửa sổ trượt như laneDetector.cpp, sau khi xoá các điểm đã thuộc vạch bên kia.
        """
        nz_y, nz_x = np.nonzero(mask)
        margin = self.px_x(self.th_win_half_w)
        used = np.zeros(len(nz_x), dtype=bool)
        lines = {"left": LaneLine(), "right": LaneLine()}

        for side in ("left", "right"):
            tr = self.tracks[side]
            if tr is None or len(nz_x) == 0:
                continue
            near = np.abs(nz_x - self.line_x_arr(tr.coeffs, tr.rows, nz_y)) < margin
            line = LaneLine()
            line.points = np.column_stack((nz_x[near], nz_y[near]))
            line.tracked = True
            if self._finish_line(line).ok:
                lines[side] = line
                used |= near

        missing = [s for s in ("left", "right") if not lines[s].ok]
        if missing:
            free = mask
            if used.any():
                free = mask.copy()
                free[nz_y[used], nz_x[used]] = 0
            for side in missing:
                lines[side] = self._window_search(free, side)

        self._drop_same_line(lines["left"], lines["right"])
        return lines

    def _window_search(self, mask: np.ndarray, side: str) -> LaneLine:
        win_h = self.h // self.NUM_WINDOWS
        margin = self.px_x(self.th_win_half_w)
        minpix = int(round(self.px_area(self.th_min_line_area)))
        midpoint = self.w // 2

        hist_rows = int(np.clip(self.px_y(0.5 * self.BEV_DEPTH), 1, self.h))
        hist_half = np.sum(
            mask[self.h - hist_rows :, :], axis=0, dtype=np.int32
        )
        hist_full = np.sum(mask, axis=0, dtype=np.int32)

        lo, hi = (0, midpoint) if side == "left" else (midpoint, self.w)
        h_part = hist_half[lo:hi] if np.max(hist_half[lo:hi]) > 0 else hist_full[lo:hi]
        x_cur = int(np.argmax(h_part)) + lo

        line = LaneLine()
        if hist_full[x_cur] == 0:
            return line

        nz_y, nz_x = np.nonzero(mask)
        inds = []
        for w_idx in range(self.NUM_WINDOWS):
            y_low = self.h - (w_idx + 1) * win_h
            y_high = self.h - w_idx * win_h
            good = (
                (nz_y >= y_low)
                & (nz_y < y_high)
                & (nz_x >= x_cur - margin)
                & (nz_x < x_cur + margin)
            ).nonzero()[0]
            inds.append(good)
            if len(good) > minpix:
                x_cur = int(np.mean(nz_x[good]))

        idx = np.concatenate(inds)
        line.points = np.column_stack((nz_x[idx], nz_y[idx]))
        return self._finish_line(line)

    def _finish_line(self, line: LaneLine) -> LaneLine:
        line.ok = line.points is not None and self._is_valid_lane(line.points)
        if line.ok:
            line.coeffs = self._fit_poly(line.points)
            line.rows = (
                int(np.min(line.points[:, 1])),
                int(np.max(line.points[:, 1])),
            )
        return line

    def _is_valid_lane(self, pts: np.ndarray) -> bool:
        if len(pts) == 0 or len(pts) < self.px_area(self.th_min_line_area):
            return False
        len_x = (np.max(pts[:, 1]) - np.min(pts[:, 1])) / self.px_per_m_y
        len_y = (np.max(pts[:, 0]) - np.min(pts[:, 0])) / self.px_per_m_x
        return np.hypot(len_x, len_y) >= self.th_min_lane_len

    def _fit_poly(self, pts: np.ndarray) -> np.ndarray:
        t = pts[:, 1].astype(np.float64) / self.h
        x = pts[:, 0].astype(np.float64)
        A = np.column_stack((t * t, t, np.ones_like(t)))
        q, _, _, _ = np.linalg.lstsq(A, x, rcond=None)
        return np.array([q[0] / (self.h * self.h), q[1] / self.h, q[2]])

    @staticmethod
    def line_x_arr(coeffs: np.ndarray, rows: tuple, y):
        """x trên BEV của vạch tại hàng y. Ngoài khoảng hàng có điểm thì kéo dài theo tiếp tuyến ở mép (tuyến tính)."""
        y = np.asarray(y, dtype=np.float64)
        yc = np.clip(y, float(rows[0]), float(rows[1]))
        x = coeffs[0] * yc * yc + coeffs[1] * yc + coeffs[2]
        slope = 2.0 * coeffs[0] * yc + coeffs[1]
        return x + slope * (y - yc)

    @staticmethod
    def line_x(coeffs: np.ndarray, rows: tuple, y: float) -> float:
        return float(CalibratedLDWS.line_x_arr(coeffs, rows, y))

    def line_ground(self, line: LaneLine, x_m: float):
        """Vạch tại khoảng cách x_m trước cầu sau: (y (m, + trái), độ dốc dy/dx, quãng phải ngoại suy (m)).

        Camera chỉ thấy vạch từ ~0.45-0.85 m trở ra nên vị trí ngang đầu xe (0.36 m) luôn là ngoại suy. Khớp đường
        thẳng qua đoạn LINE_FIT_SPAN_M gần xe nhất ổn định hơn tiếp tuyến của đa thức bậc 2 ở mép: trên ảnh giả lập,
        ngoại suy 0.25-0.45 m sai trung bình ~0.5 cm thay vì 1.2-2.5 cm, ngoại suy > 0.45 m ~2.6 cm thay vì ~9 cm.
        """
        xg = self.BEV_FAR - line.points[:, 1] / self.px_per_m_y
        yg = self.col_to_y(line.points[:, 0].astype(np.float64))
        x_near = float(xg.min())
        sel = xg <= x_near + self.LINE_FIT_SPAN_M
        if np.count_nonzero(sel) < 10 or np.ptp(xg[sel]) < 0.05:
            sel = np.ones_like(sel)
        if np.ptp(xg[sel]) >= 0.02:
            a = np.polyfit(xg[sel], yg[sel], 1)
            y, slope = float(np.polyval(a, x_m)), float(a[0])
        else:  # vạch gần như nằm ngang trên ảnh: dùng tiếp tuyến của đa thức BEV
            row = self.x_to_row(x_m)
            yc = np.clip(row, *line.rows)
            slope_bev = 2.0 * line.coeffs[0] * yc + line.coeffs[1]
            # Cột tăng = y giảm, hàng tăng = x giảm: hai lần đổi dấu triệt tiêu nhau
            y = self.col_to_y(self.line_x(line.coeffs, line.rows, row))
            slope = float(slope_bev * self.px_per_m_y / self.px_per_m_x)
        return y, slope, max(0.0, x_near - x_m)

    def _drop_same_line(self, left: LaneLine, right: LaneLine):
        if not (left.ok and right.ok):
            return
        lo = max(left.rows[0], right.rows[0])
        hi = min(left.rows[1], right.rows[1])
        if lo > hi:
            return
        ys = np.arange(lo, hi + 1, 10)
        diffs = np.abs(
            self.line_x_arr(left.coeffs, left.rows, ys)
            - self.line_x_arr(right.coeffs, right.rows, ys)
        )
        if np.mean(diffs) < self.px_x(self.th_same_line_dist):
            # Ưu tiên giữ vạch bám từ frame trước; cùng loại thì giữ vạch dài hơn
            if left.tracked != right.tracked:
                drop_left = right.tracked
            else:
                drop_left = (left.rows[1] - left.rows[0]) < (right.rows[1] - right.rows[0])
            if drop_left:
                left.ok = False
            else:
                right.ok = False

    # -----------------------------------------------------------------
    # BƯỚC 3: BÁM VẠCH THEO THỜI GIAN (KALMAN, TÀI LIỆU 3)
    # -----------------------------------------------------------------
    def _update_tracks(self, lines: dict, t: float, speed) -> dict:
        """Cập nhật track hai bên từ vạch dò được. Trả về phép đo trực tiếp {side: (y, độ dốc)} tại đầu xe."""
        x_front = self.params.rear_axle_to_front
        meas = {}  # side -> (y, độ dốc, LaneLine, thấy đủ gần để tin)
        for side, line in lines.items():
            if line.ok:
                y, slope, extrap = self.line_ground(line, x_front)
                meas[side] = (y, slope, line, extrap <= self.MAX_EXTRAP_M)
        near = {side for side, m in meas.items() if m[3]}

        if "left" in near and "right" in near:
            cur_w = meas["left"][0] - meas["right"][0]
            if self.th_min_lane_w < cur_w < self.th_max_lane_w:
                self.lane_width_m += self.LANE_WIDTH_ALPHA * (cur_w - self.lane_width_m)

        for side in ("left", "right"):
            tr = self.tracks[side]
            if tr is None:
                continue
            if t < tr.t or t - tr.last_update > self.TRACK_TIMEOUT_S:
                self.tracks[side] = None
                continue
            tr.predict(t, self.KF_ACCEL_STD)
            tr.seen = False

        lane_px = self.lane_width_m * self.px_per_m_x
        for side, other, sign in (("left", "right", 1.0), ("right", "left", -1.0)):
            if side in meas and (side in near or other not in near):
                z, slope, line, is_near = meas[side]
                std, direct = (self.KF_MEAS_STD if is_near else self.KF_FAR_STD), True
                coeffs, rows = line.coeffs, line.rows
            elif other in meas:
                # Không thấy (hoặc chỉ thấy rất xa) vạch bên này: suy từ vạch bên kia lệch đi một bề rộng làn
                z0, slope, oline, other_near = meas[other]
                z = z0 + sign * self.lane_width_m
                std, direct = (self.KF_INFER_STD if other_near else self.KF_FAR_STD), False
                coeffs = oline.coeffs - np.array([0.0, 0.0, sign * lane_px])
                rows = oline.rows
            else:
                continue

            tr = self.tracks[side]
            if tr is None:
                tr = self.tracks[side] = LineTrack(z, t, coeffs, rows)
            elif abs(z - tr.y) > max(self.KF_GATE_M, 4.0 * tr.innovation_std(std)):
                if not direct:
                    continue
                tr.rejects += 1
                if tr.rejects < self.KF_MAX_REJECTS:
                    continue
                tr = self.tracks[side] = LineTrack(z, t, coeffs, rows)
            else:
                tr.rejects = 0
                tr.update(z, std, (1.0, 0.0))
                if self.FUSE_SPEED_HEADING and speed is not None:
                    tr.update(speed * slope, max(speed * self.HEADING_STD, 1e-3), (0.0, 1.0))
                tr.last_update = t
                tr.coeffs, tr.rows = np.array(coeffs, dtype=np.float64), rows
            tr.seen = direct

        # Vạch đi qua trục giữa xe (xe đang cắt vạch sang làn bên): đổi nhãn, bên kia dò lại vạch mới
        left, right = self.tracks["left"], self.tracks["right"]
        if left is not None and left.y < -self.LABEL_HYST_M:
            self.tracks["right"], self.tracks["left"] = left, None
        elif right is not None and right.y > self.LABEL_HYST_M:
            self.tracks["left"], self.tracks["right"] = right, None

        # Góc lệch lấy từ vạch thấy gần (nếu có) vì độ dốc vạch ở xa kém tin cậy
        return {side: (m[0], m[1]) for side, m in meas.items() if m[3] or not near}

    # -----------------------------------------------------------------
    # BƯỚC 4: DỰ BÁO & ĐÁNH GIÁ CẢNH BÁO LỆCH LÀN (LDWS)
    # -----------------------------------------------------------------
    def _tlc(self, clearance: float, approach_speed: float) -> float:
        """Thời gian tới lúc góc đầu xe chạm mép vạch (s). 0 nếu đã chạm, vô cùng nếu không tiến về vạch."""
        if clearance <= 0.0:
            return 0.0
        if approach_speed > self.V_LAT_MIN:
            return clearance / approach_speed
        return float("inf")

    def _side_level(self, clearance: float, tlc: float, lor: float, h1: bool = False, h2: bool = False):
        """Mức cảnh báo của một bên. h1 / h2: nới ngưỡng cấp 1 / cấp 2 (hysteresis)."""
        if clearance <= (self.HYST_CLEARANCE_M if h2 else 0.0):
            return 2, "LINE TOUCH"
        if tlc < self.TLC_LEVEL2_S * (self.HYST_TLC if h2 else 1.0):
            return 2, "TLC"
        if lor > self.LOR_LEVEL2_TH - (self.HYST_LOR if h2 else 0.0):
            return 2, "LOR"
        if tlc < self.TLC_LEVEL1_S * (self.HYST_TLC if h1 else 1.0):
            return 1, "TLC"
        if clearance < self.CLEARANCE_WARN_M + (self.HYST_CLEARANCE_M if h1 else 0.0):
            return 1, "NEAR LINE"
        if lor > self.LOR_LEVEL1_TH - (self.HYST_LOR if h1 else 0.0):
            return 1, "LOR"
        return 0, ""

    def _evaluate(self, meas: dict, t: float, speed) -> dict:
        """Khoảng hở, TLC (Tài liệu 3), LOR (Tài liệu 2), góc lệch (Tài liệu 1), phân cấp + debounce (Tài liệu 3)."""
        k = 0.5 * (self.params.vehicle_width + self.params.line_width)  # trục giữa xe -> mép trong vạch khi chạm
        left, right = self.tracks["left"], self.tracks["right"]
        info = {
            "valid": False, "n_lines": len(meas), "raw_level": 0, "reason": "", "lor": 0.0, "offset_cm": 0.0,
            "clr_left_cm": None, "clr_right_cm": None, "wheel_clearance_cm": 0.0, "tlc_s": float("inf"),
            "v_lat_cm_s": 0.0, "angle_deg": None, "speed": speed, "lane_width_cm": self.lane_width_m * 100.0,
            "narrowing": False,
        }

        if left is None and right is None:
            self._debounce(0, None, t, corrective=False)
            info.update(self._warning_fields())
            return info

        y_l = left.y if left is not None else right.y + self.lane_width_m
        y_r = right.y if right is not None else left.y - self.lane_width_m
        vy_l = left.vy if left is not None else right.vy
        vy_r = right.vy if right is not None else left.vy

        clr_l = y_l - k  # khoảng hở góc trước-trái tới mép trong vạch trái (âm: đã đè vạch)
        clr_r = -y_r - k
        tlc_l = self._tlc(clr_l, -vy_l)
        tlc_r = self._tlc(clr_r, vy_r)
        # Hai vạch cùng tiến vào giữa: làn hẹp lại, không phải xe lệch (Tài liệu 2)
        narrowing = -vy_l > self.NARROW_V_MIN and vy_r > self.NARROW_V_MIN
        if narrowing:
            tlc_l = 0.0 if clr_l <= 0.0 else float("inf")
            tlc_r = 0.0 if clr_r <= 0.0 else float("inf")

        # LOR (Tài liệu 2) tính tại đầu xe: 0 ở giữa làn, 1/SAFE_ZONE_TH khi tâm xe tới tâm vạch
        half_lane = 0.5 * (y_l - y_r)
        left_is_near = y_l <= -y_r
        min_dist = min(y_l, -y_r)
        lor = (half_lane - min_dist) / (self.SAFE_ZONE_TH * max(half_lane, 1e-3))

        # Hysteresis: nới ngưỡng cấp 1 khi cấp 1 đang bật hoặc đang đếm debounce (để nhiễu không đặt lại bộ đếm);
        # chỉ nới ngưỡng cấp 2 khi cấp 2 đã bật (để một đỉnh nhiễu không tự kéo dài thành cảnh báo khẩn cấp)
        relax1 = self.warning_level >= 1 or self.level_since[1] is not None
        relax2 = self.warning_level >= 2
        lev_l, why_l = self._side_level(clr_l, tlc_l, lor if left_is_near else 0.0, relax1, relax2)
        lev_r, why_r = self._side_level(clr_r, tlc_r, lor if not left_is_near else 0.0, relax1, relax2)
        if (lev_l, -tlc_l, -clr_l) >= (lev_r, -tlc_r, -clr_r):
            raw, side, reason = lev_l, "LEFT", why_l
        else:
            raw, side, reason = lev_r, "RIGHT", why_r
        if speed is not None and speed < self.SPEED_MIN_WARN and raw > 0:
            raw, reason = 0, "STOPPED"

        # Lái sửa rõ ràng (xe đang rời xa vạch đang bị cảnh báo) thì tắt cảnh báo ngay (Tài liệu 3)
        ws = self.warning_side
        corrective = (ws == "LEFT" and vy_l > self.V_LAT_MIN and clr_l > 0.0) or (
            ws == "RIGHT" and -vy_r > self.V_LAT_MIN and clr_r > 0.0
        )
        self._debounce(raw, side, t, corrective)

        slopes = [m[1] for m in meas.values()]
        # Vận tốc ngang của xe so với làn (+ sang trái): trung bình hai vạch, vạch nào ước lượng chắc hơn nặng ký hơn
        if left is not None and right is not None:
            w_l, w_r = 1.0 / left.P[1, 1], 1.0 / right.P[1, 1]
            v_car = -(w_l * vy_l + w_r * vy_r) / (w_l + w_r)
        else:
            v_car = -vy_l
        info.update(
            valid=True,
            raw_level=raw,
            reason=reason,
            lor=float(lor),
            offset_cm=50.0 * (y_l + y_r),  # dương: xe lệch phải so với tâm làn
            clr_left_cm=clr_l * 100.0,
            clr_right_cm=clr_r * 100.0,
            wheel_clearance_cm=min(clr_l, clr_r) * 100.0,
            tlc_s=min(tlc_l, tlc_r),
            v_lat_cm_s=100.0 * v_car,
            # Góc đầu xe so với làn (Tài liệu 1), dương: đầu xe chĩa sang trái
            angle_deg=float(-np.degrees(np.arctan(np.mean(slopes)))) if slopes else None,
            narrowing=narrowing,
        )
        info.update(self._warning_fields())
        return info

    def _debounce(self, raw: int, side, t: float, corrective: bool):
        """Cấp k bật khi điều kiện cấp k kéo dài >= 200 ms (mỗi cấp một bộ đếm), xuống cấp khi an toàn >= 300 ms
        hoặc tắt ngay khi lái sửa (Tài liệu 3)."""
        for k in (1, 2):
            if raw < k:
                self.level_since[k] = None
            elif self.level_since[k] is None:
                self.level_since[k] = t
        ready = [k for k in (1, 2)
                 if self.level_since[k] is not None and t - self.level_since[k] >= self.DEBOUNCE_TIME_S]
        target = max(ready, default=0)
        if target > self.warning_level:
            # Hướng cảnh báo chốt lúc bắt đầu cảnh báo và giữ tới khi tắt: lúc xe cắt qua vạch, vạch đổi nhãn
            # trái <-> phải nhưng xe vẫn đang lệch về hướng cũ
            if self.warning_level == 0:
                self.warning_side = side
            self.warning_level = target
            self.safe_start_time = None
            return
        if raw < self.warning_level:
            if raw == 0 and corrective:
                self.warning_level, self.safe_start_time = 0, None
            else:
                if self.safe_start_time is None:
                    self.safe_start_time = t
                if t - self.safe_start_time >= self.RELEASE_TIME_S:
                    self.warning_level, self.safe_start_time = raw, None
        else:
            self.safe_start_time = None
        if self.warning_level == 0:
            self.warning_side = None

    def _warning_fields(self) -> dict:
        if self.warning_level == 0:
            status = "SAFE"
        else:
            kind = "EMERGENCY" if self.warning_level == 2 else "EARLY WARN"
            status = f"{self.warning_side}_DEPARTURE ({kind})"
        return {"status": status, "level": self.warning_level, "direction": self.warning_side or ""}

    # -----------------------------------------------------------------
    # BƯỚC 5: XỬ LÝ FRAME THÔ 640x480 & VẼ HIỂN THỊ KÉP (CAMERA + BEV)
    # -----------------------------------------------------------------
    def process_raw_frame(self, raw_bgr: np.ndarray, t=None, speed=None):
        """t: thời điểm frame (s, mặc định đồng hồ máy); speed: vận tốc dọc (m/s) hoặc None nếu chưa có."""
        if t is None:
            t = time.perf_counter()
        undistorted = cv2.remap(raw_bgr, self.map1, self.map2, cv2.INTER_LINEAR)
        return self.process_undistorted(undistorted, t, speed)

    def process_undistorted(self, undistorted: np.ndarray, t: float, speed=None):
        t0 = time.perf_counter()
        bird_eye_view = cv2.warpPerspective(
            undistorted, self.ipm_matrix, (self.w, self.h)
        )
        mask = self.process_mask(bird_eye_view)
        lines = self.detect_lines(mask)
        meas = self._update_tracks(lines, t, speed)
        info = self._evaluate(meas, t, speed)
        info["n_lines"] = sum(line.ok for line in lines.values())  # số vạch dò thấy trực tiếp frame này
        info["t"] = t
        info["proc_ms"] = (time.perf_counter() - t0) * 1000.0

        vis_bev, vis_cam = self._draw_visualizations(undistorted, bird_eye_view, info)
        return info, vis_cam, vis_bev, mask

    def _draw_visualizations(self, undistorted, bev, info):
        vis_bev = bev.copy()
        vis_cam = undistorted.copy()
        ys = np.arange(0, self.h, 5)

        # Màu sắc theo cấp độ cảnh báo (Xanh lá: An toàn, Vàng: Cấp 1, Đỏ: Cấp 2)
        if info["level"] == 2:
            alert_color = (0, 0, 255)
        elif info["level"] == 1:
            alert_color = (0, 215, 255)
        else:
            alert_color = (0, 255, 0)

        # Vạch đang bám: nét dày nếu frame này dò thấy, nét mảnh nếu chỉ dự đoán/suy ra
        pts_bev = {}
        for side, color in (("left", (0, 255, 0)), ("right", (0, 0, 255))):
            tr = self.tracks[side]
            if tr is None:
                continue
            xs = np.clip(self.line_x_arr(tr.coeffs, tr.rows, ys), -10 * self.w, 10 * self.w)
            pts_bev[side] = np.column_stack((np.round(xs), ys)).astype(np.int32)
            cv2.polylines(vis_bev, [pts_bev[side]], False, color, 3 if tr.seen else 1)

        # Chiếu ngược vùng làn từ BEV về ảnh Camera gốc để quan sát trực quan như Tài liệu 1 & 2
        if len(pts_bev) == 2:
            poly_bev = np.vstack((pts_bev["left"], pts_bev["right"][::-1])).astype(
                np.float32
            ).reshape(-1, 1, 2)
            poly_cam = cv2.perspectiveTransform(poly_bev, self.ipm_inv).astype(
                np.int32
            )
            overlay = vis_cam.copy()
            cv2.fillPoly(overlay, [poly_cam], alert_color)
            cv2.addWeighted(overlay, 0.30, vis_cam, 0.70, 0, vis_cam)
            cv2.polylines(vis_cam, [poly_cam], True, alert_color, 2)

        # Vẽ bề rộng thân xe (21 cm) quy chiếu ở đáy ảnh BEV
        car_half_px = self.px_x(0.5 * self.params.vehicle_width)
        cx = self.w // 2
        cv2.rectangle(
            vis_bev,
            (cx - car_half_px, self.h - 25),
            (cx + car_half_px, self.h - 2),
            alert_color,
            2,
        )
        cv2.line(vis_bev, (cx, self.h - 35), (cx, self.h), (255, 255, 0), 2)

        # Bảng thông tin HUD trên ảnh Camera (chữ không dấu: putText không vẽ được tiếng Việt)
        def fmt(v, spec):
            return "--" if v is None or not np.isfinite(v) else format(v, spec)

        state = info["status"] + (f" [{info['reason']}]" if info["reason"] else "")
        if not info["valid"]:
            state += " | NO LANE"
        angle = info["angle_deg"]
        angle_txt = "--" if angle is None else f"{angle:+.1f} deg ({'L' if angle >= 0 else 'R'})"
        rows = [
            (state, alert_color, 0.55, 2),
            (f"Offset: {fmt(info['offset_cm'], '+.1f')} cm | LOR: {info['lor']:.2f}", (255, 255, 255), 0.5, 1),
            (f"Gap L/R: {fmt(info['clr_left_cm'], '+.1f')} / {fmt(info['clr_right_cm'], '+.1f')} cm",
             (255, 255, 0), 0.5, 1),
            (f"TLC: {fmt(info['tlc_s'], '.2f')} s | V_lat: {info['v_lat_cm_s']:+.1f} cm/s", (255, 255, 0), 0.5, 1),
            (f"Heading: {angle_txt} | v: {fmt(info['speed'], '.2f')} m/s | {info['proc_ms']:.1f} ms",
             (200, 200, 200), 0.5, 1),
        ]
        box_h = 20 + 26 * len(rows)
        cv2.rectangle(vis_cam, (10, 10), (420, box_h), (0, 0, 0), -1)
        cv2.rectangle(vis_cam, (10, 10), (420, box_h), alert_color, 2)
        for i, (text, color, scale, thick) in enumerate(rows):
            cv2.putText(vis_cam, text, (20, 35 + 26 * i), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick)

        return vis_bev, vis_cam


# =====================================================================
# 3. NHẬN DỮ LIỆU, GHI KẾT QUẢ, VÒNG LẶP CHÍNH
# =====================================================================
def parse_speed(packet: bytes):
    """Vận tốc dọc (m/s) từ một gói UDP: văn bản ("0.52") hoặc float32/float64 little-endian. None nếu không đọc được."""
    try:
        v = float(packet.decode("ascii").strip())
    except (UnicodeDecodeError, ValueError):
        if len(packet) == 4:
            v = struct.unpack("<f", packet)[0]
        elif len(packet) == 8:
            v = struct.unpack("<d", packet)[0]
        else:
            return None
    return v if np.isfinite(v) else None


class SpeedReceiver:
    """Nhận vận tốc dọc từ Jetson qua UDP (không chặn). Quá timeout_s không có gói mới thì coi như chưa biết vận tốc."""

    def __init__(self, ip: str, port: int, timeout_s: float = 0.5):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((ip, port))
        self.sock.setblocking(False)
        self.timeout_s = timeout_s
        self.speed = None
        self.t = None

    def poll(self, now: float):
        while True:
            try:
                packet, _ = self.sock.recvfrom(64)
            except OSError:  # hết gói (BlockingIOError) hoặc lỗi socket tạm thời
                break
            v = parse_speed(packet)
            if v is not None:
                self.speed, self.t = v, now
        if self.t is None or now - self.t > self.timeout_s:
            return None
        return self.speed

    def close(self):
        self.sock.close()


class ResultLog:
    """Ghi kết quả từng frame ra CSV (bỏ qua nếu không có đường dẫn)."""

    FIELDS = ["t", "status", "level", "direction", "reason", "raw_level", "valid", "n_lines", "offset_cm", "lor",
              "clr_left_cm", "clr_right_cm", "tlc_s", "v_lat_cm_s", "angle_deg", "speed", "lane_width_cm",
              "narrowing", "proc_ms"]

    def __init__(self, path):
        # buffering=1: ghi xuống đĩa từng dòng, tắt ngang chương trình vẫn còn dữ liệu
        self.file = open(path, "w", newline="", encoding="utf-8", buffering=1) if path else None
        self.writer = csv.DictWriter(self.file, self.FIELDS, extrasaction="ignore") if self.file else None
        if self.writer:
            self.writer.writeheader()

    def write(self, info: dict):
        if not self.writer:
            return
        row = {}
        for k in self.FIELDS:
            v = info.get(k)
            row[k] = "" if v is None else (f"{v:.4f}" if isinstance(v, float) else v)
        self.writer.writerow(row)

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
    """Nhận frame thô 640x480 (JPEG, UDP 7000) và vận tốc dọc (UDP 7001) từ Jetson."""
    ldws = CalibratedLDWS()
    speed_rx = SpeedReceiver(args.listen_ip, args.speed_port)
    log = ResultLog(args.csv)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    sock.bind((args.listen_ip, args.port))
    sock.settimeout(0.5)
    print(f"[PC LDWS] Dang lang nghe anh tho 640x480 tai UDP {args.listen_ip}:{args.port}, "
          f"van toc doc tai UDP {args.speed_port}... (Ctrl+C hoac q de thoat)", flush=True)

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

            info, vis_cam, vis_bev, mask = ldws.process_raw_frame(raw_frame, now, speed_rx.poll(now))
            log.write(info)
            if not args.no_show and not show_frames(vis_cam, vis_bev, mask):
                break
    finally:
        sock.close()
        speed_rx.close()
        log.close()
        cv2.destroyAllWindows()


def iter_replay(args):
    """(t, frame) từ video hoặc thư mục/mẫu glob ảnh. t tính theo fps để debounce, Kalman và TLC chạy như thật."""
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
    ldws = CalibratedLDWS()
    log = ResultLog(args.csv)
    counts, proc = [0, 0, 0], []
    try:
        for t, frame in iter_replay(args):
            if (frame.shape[1], frame.shape[0]) != (640, 480):
                frame = cv2.resize(frame, (640, 480))
            info, vis_cam, vis_bev, mask = ldws.process_raw_frame(frame, t, args.speed)
            log.write(info)
            counts[info["level"]] += 1
            proc.append(info["proc_ms"])
            if not args.no_show and not show_frames(vis_cam, vis_bev, mask, args.delay):
                break
    finally:
        log.close()
        cv2.destroyAllWindows()
    n = sum(counts)
    if n:
        print(f"[PC LDWS] {n} frame | SAFE {counts[0]} | canh bao som {counts[1]} | khan cap {counts[2]} | "
              f"xu ly trung binh {np.mean(proc):.1f} ms, lon nhat {np.max(proc):.1f} ms")


def main():
    parser = argparse.ArgumentParser(description="Canh bao lech lan (LDWS) tren PC cho xe mo hinh")
    src = parser.add_mutually_exclusive_group()
    src.add_argument("--video", help="chay lai tu file video thay vi nhan UDP")
    src.add_argument("--images", help="chay lai tu thu muc anh hoac mau glob (sap xep theo ten)")
    parser.add_argument("--speed", type=float, default=None,
                        help="van toc doc gia dinh khi chay lai (m/s); bo trong = khong biet van toc")
    parser.add_argument("--fps", type=float, default=30.0, help="fps cua chuoi anh (hoac video khong ghi fps)")
    parser.add_argument("--delay", type=int, default=1, help="ms cho giua 2 frame khi chay lai co hien thi")
    parser.add_argument("--csv", help="ghi ket qua tung frame ra file CSV")
    parser.add_argument("--no-show", action="store_true", help="khong mo cua so hien thi")
    parser.add_argument("--listen-ip", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=FRAME_PORT, help="cong UDP nhan anh JPEG")
    parser.add_argument("--speed-port", type=int, default=SPEED_PORT, help="cong UDP nhan van toc doc")
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
