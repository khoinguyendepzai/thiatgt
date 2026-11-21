"""Đóng gói module Nhận diện & Khoảng cách để chạy chung trong luồng Main Master."""
import time
import cv2
import cau_hinh as C

from detector import MultiModelDetector
from distance import estimate_distance
from postprocess import drop_riders, plausible, suppress_cross_class
from tracker import Tracker

CFG = {c.key: c for c in C.CLASSES}

class ObstacleDetector:
    def __init__(self):
        print("[VISION] Đang nạp model YOLO...")
        self.detector = MultiModelDetector(C.CLASSES, C.IMGSZ, C.NMS_IOU, C.DEVICE)
        self.tracker = Tracker(C.MATCH_IOU, C.CONFIRM_FRAMES, C.MAX_MISSED_S, C.SMOOTH_FRAMES)
        self.order = tuple(c.key for c in C.CLASSES)

    def process_frame(self, frame, t_now=None):
        """Đầu vào là khung hình. Đầu ra là dictionary khoảng cách các vật thể."""
        if t_now is None:
            t_now = time.monotonic()

        # Đưa ảnh về chuẩn 1280x720 theo đúng focal_length đã đo
        if frame.shape[1] != C.FRAME_W or frame.shape[0] != C.FRAME_H:
            frame = cv2.resize(frame, (C.FRAME_W, C.FRAME_H))

        # 1. Nhận diện và lọc nhiễu
        dets = [d for d in self.detector.detect(frame) if plausible(d, CFG[d.slot], C.MIN_BOX_HEIGHT_PX)]
        dets = suppress_cross_class(dets, C.CROSS_CLASS_IOU)
        dets = drop_riders(dets, pedestrian="nguoi_di_bo", vehicle="xe_may", overlap=C.RIDER_OVERLAP)

        # 2. Tính khoảng cách
        items, cut = [], []
        for d in dets:
            dist = estimate_distance(d.box, CFG[d.slot], C.FOCAL_LENGTH, C.FRAME_W, C.FRAME_H)
            if dist is not None and C.DISTANCE_RANGE_M[0] <= dist.value <= C.DISTANCE_RANGE_M[1]:
                items.append((d, dist.value))
                cut.append(dist.truncated)

        # 3. Cập nhật Tracker
        self.tracker.update(t_now, items, cut)
        nearest = self.tracker.nearest()

        # 4. Chuẩn hóa format (-1.0 nếu không có vật thể trên khung hình)
        distances_out = {key: nearest.get(key, -1.0) for key in self.order}
        return distances_out, frame

    def draw_bboxes(self, frame):
        """Vẽ hộp Bounding Box YOLO lên ảnh nếu cần hiển thị debug."""
        for tr in self.tracker.confirmed():
            c = CFG[tr.slot]
            x1, y1, x2, y2 = map(int, tr.box)
            cv2.rectangle(frame, (x1, y1), (x2, y2), c.color, 2)
            label = f"{tr.slot} {tr.conf:.2f} | {tr.distance:.2f}m" + (" (cat mep)" if tr.truncated else "")
            cv2.putText(frame, label, (x1, max(18, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, c.color, 2)
        return frame