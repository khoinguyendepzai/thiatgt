"""Nạp các file .pt và chạy YOLO. Hỗ trợ cả hai cách:
  * mỗi vật thể một file .pt (models/xe_may.pt, models/nguoi_di_bo.pt, ...);
  * một file .pt nhiều lớp dùng chung (điền cùng tên file cho nhiều vật thể, phân biệt bằng `names`).
Mỗi file chỉ chạy MỘT lần mỗi frame dù dùng cho nhiều vật thể.
"""

from __future__ import annotations

import os

from cau_hinh import ClassCfg
from postprocess import Detection

HERE = os.path.dirname(os.path.abspath(__file__))


def find_model_file(model: str, base: str = HERE) -> str | None:
    """Tìm file .pt: đúng đường dẫn trong cấu hình (models/...), không có thì tìm cùng tên ở thư mục dự án."""
    path = os.path.normpath(model if os.path.isabs(model) else os.path.join(base, model))
    for p in (path, os.path.join(base, os.path.basename(model))):
        if os.path.isfile(p):
            return p
    return None


class MultiModelDetector:
    def __init__(self, classes: tuple[ClassCfg, ...], imgsz: int, nms_iou: float, device: str) -> None:
        from ultralytics import YOLO
        self.imgsz, self.nms_iou, self.device = imgsz, nms_iou, device or None
        self.groups: dict[str, tuple[object, list[ClassCfg]]] = {}
        self.status: dict[str, str] = {}
        for c in classes:
            if not c.real_height_m:
                self.status[c.key] = "TẮT - chưa nhập real_height_m (chiều cao thật) trong cau_hinh.py"
                continue
            path = find_model_file(c.model)
            if path is None:
                self.status[c.key] = f"TẮT - chưa có file {c.model} (chép file .pt vào thư mục models/)"
                continue
            if path not in self.groups:
                self.groups[path] = (YOLO(path), [])
            model, cfgs = self.groups[path]
            available = set(model.names.values())
            missing = [n for n in c.names if n not in available]
            if missing:
                self.status[c.key] = f"TẮT - {c.model} không có lớp {missing}; các lớp có trong file: {sorted(available)}"
                continue
            cfgs.append(c)
            shown = list(c.names) or sorted(available)
            if len(shown) > 6:
                shown = shown[:6] + ["..."]
            self.status[c.key] = (f"BẬT - {os.path.relpath(path, HERE)}, lớp {shown}, "
                                  f"cao {c.real_height_m} m, conf ≥ {c.conf}")
            if not c.names and len(available) > 1:
                self.status[c.key] += (f"\n      [CANH BAO] file có {len(available)} lớp, đang coi TẤT CẢ là {c.key}"
                                       f" -> điền names=(\"<tên lớp>\",) cho {c.key} trong cau_hinh.py")

    @property
    def active(self) -> list[str]:
        return [c.key for _, cfgs in self.groups.values() for c in cfgs]

    def detect(self, frame) -> list[Detection]:
        out: list[Detection] = []
        for model, cfgs in self.groups.values():
            if not cfgs:
                continue
            res = model.predict(frame, conf=min(c.conf for c in cfgs), iou=self.nms_iou, imgsz=self.imgsz,
                                device=self.device, verbose=False)[0]
            if res.boxes is None or len(res.boxes) == 0:
                continue
            xyxy = res.boxes.xyxy.cpu().numpy()
            confs = res.boxes.conf.cpu().numpy()
            cls = res.boxes.cls.cpu().numpy().astype(int)
            for box, conf, k in zip(xyxy, confs, cls):
                label = model.names[int(k)]
                for c in cfgs:                       # lớp không thuộc vật thể nào trong cấu hình -> bỏ (vd. "cốc")
                    if (not c.names or label in c.names) and conf >= c.conf:
                        out.append(Detection(c.key, tuple(float(v) for v in box), float(conf), label))
                        break
        return out
