"""Phần 1 — xử lý ảnh: từ frame camera tính các thông số hình học của làn tại đầu xe.

Khử méo -> Bird's-Eye View (homography mặt đường) -> mask vạch trắng (HSV + Top-Hat + lọc hình thái) -> dò vạch (quanh
vạch frame trước hoặc cửa sổ trượt) -> khớp đa thức -> ngoại suy về đầu xe -> bám vạch bằng bộ lọc trượt. Kết quả mỗi
frame là một LaneGeometry (vị trí hai vạch, bề rộng làn, độ lệch tâm, khoảng hở, góc lệch) dùng để ghi log và làm đầu
vào cho phần 2 (lane_warning.py). Phần này không biết gì về ngưỡng cảnh báo; nó chỉ vẽ màu theo mức mà phần 2 đưa vào.

Phần dò làn được port từ laneDetector.cpp / main.cpp của BrainUp; comment ghi tên hàm C++ gốc để đối chiếu.
"""

import time
from dataclasses import dataclass
from typing import NamedTuple

import cv2
import numpy as np


@dataclass
class LaneGeometry:
    """Thông số hình học của làn tại mặt cắt đầu xe (x = rear_axle_to_front), hệ toạ độ xe: y dương sang trái."""

    t: float  # thời điểm frame (s)
    valid: bool = False  # có ít nhất một vạch đang được bám
    n_lines: int = 0  # số vạch dò thấy trực tiếp ở frame này
    y_left: float = None  # m, tâm vạch trái (suy từ vạch phải + bề rộng làn nếu không bám được vạch trái)
    y_right: float = None  # m, tâm vạch phải (âm)
    left_tracked: bool = False  # vạch trái đang được bám (False: y_left là giá trị suy ra)
    right_tracked: bool = False
    lane_width_m: float = 0.0  # bề rộng làn trung bình trượt mũ (dùng để suy vạch thiếu)
    offset_cm: float = 0.0  # độ lệch tâm, dương: xe lệch phải so với tâm làn
    clr_left_cm: float = None  # khoảng hở góc trước-trái tới mép trong vạch trái (âm: đã đè vạch)
    clr_right_cm: float = None
    angle_deg: float = None  # góc đầu xe so với làn, dương: chĩa sang trái (chỉ để hiển thị)
    proc_ms: float = 0.0  # thời gian xử lý ảnh của frame (không gồm khử méo và vẽ)

    @property
    def lane_width_cm(self) -> float:
        return self.lane_width_m * 100.0


class VisionImages(NamedTuple):
    undistorted: np.ndarray  # ảnh camera đã khử méo
    bev: np.ndarray  # ảnh Bird's-Eye View
    mask: np.ndarray  # mask nhị phân vạch trắng trên BEV


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
    """Bộ lọc trượt bám một vạch qua các frame: vị trí vạch là trung bình có trọng số của các phép đo trong cửa sổ
    thời gian gần nhất (phép đo chắc hơn nặng ký hơn).

    y: vị trí ngang của vạch tại đầu xe (m, hệ toạ độ xe, + sang trái). Trễ trung bình khoảng nửa cửa sổ.
    """

    def __init__(self, y, t, coeffs, rows):
        self.samples = [(t, float(y), 1.0)]  # (thời điểm, vị trí, trọng số)
        self.y = float(y)
        self.t = t
        self.last_update = t
        self.rejects = 0  # số frame liên tiếp phép đo trực tiếp bị loại vì lệch quá xa
        self.coeffs = np.array(coeffs, dtype=np.float64)  # đa thức BEV gần nhất: vùng tìm vạch cho frame sau
        self.rows = rows
        self.seen = True  # frame này có dò thấy trực tiếp vạch này không

    def update(self, z, meas_std, t, window_s):
        self.samples.append((t, float(z), 1.0 / meas_std**2))
        self.samples = [m for m in self.samples if t - m[0] <= window_s]
        w = sum(m[2] for m in self.samples)
        self.y = sum(m[1] * m[2] for m in self.samples) / w
        self.t = t


class LaneVision:
    """Xử lý ảnh -> LaneGeometry. calib: GroundCalibration, params: TrackAndVehicleParams (khai báo trong main.py)."""

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

    # Bám vạch bằng bộ lọc trượt, chỉ để làm mượt vị trí
    SMOOTH_WINDOW_S = 0.15  # s: cửa sổ trung bình trượt (~5 frame ở 30 fps), trễ ~ nửa cửa sổ
    KF_MEAS_STD = 0.005  # m: sai số đo vị trí vạch dò trực tiếp
    KF_INFER_STD = 0.015  # m: sai số vạch suy từ vạch bên kia + bề rộng làn
    KF_FAR_STD = 0.03  # m: sai số vạch chỉ thấy ở xa, phải ngoại suy dài về đầu xe
    LINE_FIT_SPAN_M = 0.25  # m: khớp thẳng qua đoạn vạch dài chừng này gần xe nhất để ngoại suy về đầu xe
    MAX_EXTRAP_M = 0.45  # m: điểm vạch gần nhất thấy được cách đầu xe xa hơn mức này thì coi là kém tin cậy
    KF_GATE_M = 0.04  # m: loại phép đo lệch vị trí đang bám quá mức này
    KF_MAX_REJECTS = 5  # bị loại liên tiếp chừng này frame thì khởi tạo lại track theo phép đo
    TRACK_TIMEOUT_S = 0.5  # không cập nhật được quá lâu thì bỏ track
    LABEL_HYST_M = 0.02  # vạch phải vượt qua trục giữa xe 2 cm mới đổi nhãn trái <-> phải

    def __init__(self, calib, params):
        self.calib = calib
        self.params = params
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

        # 4. Tạo mặt nạ vùng nhìn thấy hợp lệ (visible_ trong laneDetector.cpp; hiện chưa dùng)
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

        # Trạng thái bám vạch
        self.lane_width_m = self.params.lane_width
        self.tracks = {"left": None, "right": None}

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
        return float(LaneVision.line_x_arr(coeffs, rows, y))

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
    # BƯỚC 3: BÁM VẠCH THEO THỜI GIAN (BỘ LỌC TRƯỢT)
    # -----------------------------------------------------------------
    def _update_tracks(self, lines: dict, t: float) -> dict:
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
            elif abs(z - tr.y) > self.KF_GATE_M:
                if not direct:
                    continue
                tr.rejects += 1
                if tr.rejects < self.KF_MAX_REJECTS:
                    continue
                tr = self.tracks[side] = LineTrack(z, t, coeffs, rows)
            else:
                tr.rejects = 0
                tr.update(z, std, t, self.SMOOTH_WINDOW_S)
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
    # BƯỚC 4: THÔNG SỐ HÌNH HỌC TẠI ĐẦU XE (để log và cho phần 2)
    # -----------------------------------------------------------------
    def _geometry(self, meas: dict, n_lines: int, t: float) -> LaneGeometry:
        """Vị trí hai vạch tại đầu xe (vạch không bám được thì suy từ vạch kia + bề rộng làn), khoảng hở, độ lệch tâm,
        góc lệch."""
        k = 0.5 * (self.params.vehicle_width + self.params.line_width)  # trục giữa xe -> mép trong vạch khi chạm
        left, right = self.tracks["left"], self.tracks["right"]
        geom = LaneGeometry(t=t, n_lines=n_lines, lane_width_m=self.lane_width_m)
        if left is None and right is None:
            return geom

        y_l = left.y if left is not None else right.y + self.lane_width_m
        y_r = right.y if right is not None else left.y - self.lane_width_m
        slopes = [m[1] for m in meas.values()]
        geom.valid = True
        geom.y_left, geom.y_right = y_l, y_r
        geom.left_tracked, geom.right_tracked = left is not None, right is not None
        geom.offset_cm = 50.0 * (y_l + y_r)  # dương: xe lệch phải so với tâm làn
        geom.clr_left_cm = (y_l - k) * 100.0
        geom.clr_right_cm = (-y_r - k) * 100.0
        # Góc đầu xe so với làn, chỉ để hiển thị (dương: đầu xe chĩa sang trái)
        geom.angle_deg = float(-np.degrees(np.arctan(np.mean(slopes)))) if slopes else None
        return geom

    # -----------------------------------------------------------------
    # BƯỚC 5: XỬ LÝ FRAME THÔ 640x480 & VẼ HIỂN THỊ KÉP (CAMERA + BEV)
    # -----------------------------------------------------------------
    def process_raw_frame(self, raw_bgr: np.ndarray, t=None):
        """Frame thô -> (LaneGeometry, VisionImages). t: thời điểm frame (s, mặc định đồng hồ máy)."""
        if t is None:
            t = time.perf_counter()
        undistorted = cv2.remap(raw_bgr, self.map1, self.map2, cv2.INTER_LINEAR)
        return self.process_undistorted(undistorted, t)

    def process_undistorted(self, undistorted: np.ndarray, t: float):
        """Ảnh đã khử méo -> (LaneGeometry, VisionImages). t truyền từ ngoài vào để chạy lại / giả lập cho kết quả
        giống hệt chạy thật (bộ lọc trượt theo t, không đọc đồng hồ máy)."""
        t0 = time.perf_counter()
        bird_eye_view = cv2.warpPerspective(
            undistorted, self.ipm_matrix, (self.w, self.h)
        )
        mask = self.process_mask(bird_eye_view)
        lines = self.detect_lines(mask)
        meas = self._update_tracks(lines, t)
        geom = self._geometry(meas, sum(line.ok for line in lines.values()), t)
        geom.proc_ms = (time.perf_counter() - t0) * 1000.0
        return geom, VisionImages(undistorted, bird_eye_view, mask)

    def draw(self, images: VisionImages, geom: LaneGeometry, level: int, top_rows):
        """Vẽ vạch đang bám lên BEV, vùng làn lên ảnh camera (màu theo mức cảnh báo level của phần 2) và bảng HUD.
        top_rows: các dòng chữ đầu bảng do phần 2 cung cấp (trạng thái, LOR). Trả về (vis_cam, vis_bev)."""
        vis_bev = images.bev.copy()
        vis_cam = images.undistorted.copy()
        ys = np.arange(0, self.h, 5)

        # Màu sắc theo cấp độ cảnh báo (Xanh lá: An toàn, Vàng: Cấp 1, Đỏ: Cấp 2)
        if level == 2:
            alert_color = (0, 0, 255)
        elif level == 1:
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

        angle = geom.angle_deg
        angle_txt = "--" if angle is None else f"{angle:+.1f} deg ({'L' if angle >= 0 else 'R'})"
        rows = [(top_rows[0], alert_color, 0.55, 2)] + [(r, (255, 255, 255), 0.5, 1) for r in top_rows[1:]] + [
            (f"Offset: {fmt(geom.offset_cm, '+.1f')} cm | Lane: {geom.lane_width_cm:.1f} cm",
             (255, 255, 255), 0.5, 1),
            (f"Gap L/R: {fmt(geom.clr_left_cm, '+.1f')} / {fmt(geom.clr_right_cm, '+.1f')} cm",
             (255, 255, 0), 0.5, 1),
            (f"Heading: {angle_txt} | {geom.proc_ms:.1f} ms",
             (200, 200, 200), 0.5, 1),
        ]
        box_h = 20 + 26 * len(rows)
        cv2.rectangle(vis_cam, (10, 10), (420, box_h), (0, 0, 0), -1)
        cv2.rectangle(vis_cam, (10, 10), (420, box_h), alert_color, 2)
        for i, (text, color, scale, thick) in enumerate(rows):
            cv2.putText(vis_cam, text, (20, 35 + 26 * i), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick)

        return vis_cam, vis_bev
