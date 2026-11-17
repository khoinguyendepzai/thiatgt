"""Lọc kết quả YOLO trước khi tính khoảng cách — giảm nhận nhầm mà không cần train lại."""

from __future__ import annotations

from typing import NamedTuple

from cau_hinh import ClassCfg


class Detection(NamedTuple):
    slot: str                                    # key của vật thể trong cau_hinh.CLASSES
    box: tuple[float, float, float, float]      # x1, y1, x2, y2 (pixel)
    conf: float
    label: str                                   # tên lớp gốc trong file .pt


def area(b) -> float:
    return max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])


def intersection(a, b) -> float:
    return area((max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])))


def iou(a, b) -> float:
    inter = intersection(a, b)
    union = area(a) + area(b) - inter
    return inter / union if union > 0 else 0.0


def plausible(det: Detection, cls: ClassCfg, min_height_px: float) -> bool:
    return det.box[3] - det.box[1] >= min_height_px and det.box[2] > det.box[0]


def suppress_cross_class(dets: list[Detection], min_iou: float) -> list[Detection]:
    """Hai model KHÁC lớp cùng khoanh một vật (IoU ≥ ngưỡng) -> giữ hộp tin cậy hơn. Cùng lớp thì không đụng tới
    (YOLO đã NMS trong từng model), nên hai vật cạnh nhau cùng loại không bị gộp."""
    kept: list[Detection] = []
    for d in sorted(dets, key=lambda x: -x.conf):
        if all(k.slot == d.slot or iou(k.box, d.box) < min_iou for k in kept):
            kept.append(d)
    return [d for d in dets if d in kept]


def drop_riders(dets: list[Detection], pedestrian: str, vehicle: str, overlap: float | None) -> list[Detection]:
    """Người có ≥ overlap diện tích nằm trong hộp xe máy là người lái/ngồi sau, không phải người đi bộ."""
    if overlap is None:
        return dets
    bikes = [d.box for d in dets if d.slot == vehicle]

    def rider(d: Detection) -> bool:
        a = area(d.box)
        return a > 0 and any(intersection(d.box, b) / a >= overlap for b in bikes)
    return [d for d in dets if not (d.slot == pedestrian and rider(d))]
