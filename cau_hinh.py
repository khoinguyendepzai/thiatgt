"""FILE CẤU HÌNH (Đã tối ưu cho Main Master)"""
from __future__ import annotations
from dataclasses import dataclass

@dataclass(frozen=True)
class ClassCfg:
    key: str
    model: str
    names: tuple[str, ...] = ()
    real_height_m: float | None = None
    real_width_m: float | None = None
    conf: float = 0.5
    color: tuple[int, int, int] = (0, 255, 0)

# ================= CÁC VẬT THỂ =================
# CẢNH BÁO (kiểm tra 21/11/2026): ba lớp đang KHÁC tỉ lệ. Người đi bộ 0.1 m là cỡ mô hình, còn xe máy 1.10 m và
# ô tô 1.50 m là cỡ thật. Trên sa bàn dùng mô hình xe máy / ô tô thì khoảng cách D = f·H/h tính ra XA GẤP ~10 lần
# (planner kích hoạt ở 1.1 m sẽ không bao giờ chạy). Đo chiều cao / rộng thật của từng mô hình vật cản rồi sửa ở đây.
CLASSES = (
    ClassCfg(key="xe_may", model="models/xe_may.pt", names=(), real_height_m=1.10, real_width_m=0.69, conf=0.5, color=(255, 128, 0)),
    ClassCfg(key="nguoi_di_bo", model="models/nguoi_di_bo.pt", names=(), real_height_m=0.1, real_width_m=None, conf=0.5, color=(0, 200, 255)),
    ClassCfg(key="o_to", model="models/o_to.pt", names=(), real_height_m=1.50, real_width_m=1.73, conf=0.5, color=(0, 255, 0)),
)

# ================= KHOẢNG CÁCH =================
FRAME_W, FRAME_H = 1280, 720
# Tiêu cự theo pixel ở 1280x720. Đo với camera cũ: phải đo lại cho IMX219-160 trên Jetson (camera nhìn đường gửi
# 640x360 rồi được phóng lên 1280x720): đặt vật cao H ở khoảng cách D đo bằng thước, đọc chiều cao khung bao h (px ở
# 1280x720), FOCAL_LENGTH = h·D/H; lấy trung bình vài khoảng cách.
FOCAL_LENGTH = 1452

# ================= CHỐNG NHẬN NHẦM =================
IMGSZ = 640
NMS_IOU = 0.5
DEVICE = ""
CROSS_CLASS_IOU = 0.7
RIDER_OVERLAP = 0.5
MIN_BOX_HEIGHT_PX = 12
DISTANCE_RANGE_M = (0.05, 30.0)

# ================= THEO DÕI TỪNG VẬT THỂ =================
MATCH_IOU = 0.3
CONFIRM_FRAMES = 2
MAX_MISSED_S = 0.3
SMOOTH_FRAMES = 5