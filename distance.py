"""Khoảng cách theo mô hình camera lỗ kim — cùng công thức code cũ: D = f · H_thật / h_ảnh."""

from __future__ import annotations

from typing import NamedTuple

from cau_hinh import ClassCfg

EDGE_PX = 2.0     # hộp cách mép ảnh ≤ 2 px coi như bị mép cắt


class Distance(NamedTuple):
    value: float          # mét
    truncated: bool       # hộp bị mép ảnh cắt (vật quá gần / ra khỏi khung)


def estimate_distance(box, cls: ClassCfg, focal_px: float, frame_w: int, frame_h: int) -> Distance | None:
    x1, y1, x2, y2 = box
    h, w = y2 - y1, x2 - x1
    if h <= 0 or w <= 0 or not cls.real_height_m:
        return None
    d = focal_px * cls.real_height_m / h
    cut_v = y1 <= EDGE_PX or y2 >= frame_h - EDGE_PX
    cut_h = x1 <= EDGE_PX or x2 >= frame_w - EDGE_PX
    if cut_v and cls.real_width_m:
        # chiều cao bị cắt -> d theo chiều cao bị ƯỚC LỚN (nguy hiểm khi tính va chạm); bề rộng cho ước lượng thứ
        # hai, lấy giá trị NGẮN hơn cho an toàn
        d = min(d, focal_px * cls.real_width_m / w)
    return Distance(d, cut_v or cut_h)
