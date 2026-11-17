"""Theo dõi TỪNG vật thể qua các frame: mỗi vật có ID và lịch sử khoảng cách riêng -> không lẫn vào nhau.

Ghép hộp frame mới với vật thể cũ theo độ trùng (IoU), chỉ trong cùng một lớp. Vật mới phải thấy liên tiếp
confirm_frames frame mới được báo (lọc nhận nhầm thoáng qua); mất dấu ngắn hơn max_missed_s vẫn giữ.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from statistics import median

from postprocess import Detection, iou


@dataclass
class Track:
    id: int
    slot: str
    box: tuple[float, float, float, float]
    conf: float
    last_seen: float
    hits: int = 1
    confirmed: bool = False
    truncated: bool = False
    history: deque = field(default_factory=deque)

    @property
    def distance(self) -> float:
        return median(self.history)


class Tracker:
    def __init__(self, match_iou: float, confirm_frames: int, max_missed_s: float, smooth_frames: int) -> None:
        self.match_iou, self.confirm_frames = match_iou, confirm_frames
        self.max_missed_s, self.smooth_frames = max_missed_s, smooth_frames
        self.tracks: list[Track] = []
        self._next_id = 1

    def update(self, t: float, items: list[tuple[Detection, float]], truncated: list[bool] | None = None) -> list[Track]:
        """items: [(hộp, khoảng cách thô)] của frame hiện tại. Trả về các vật thể đã xác nhận."""
        truncated = truncated or [False] * len(items)
        pairs = sorted(((iou(tr.box, det.box), ti, di) for ti, tr in enumerate(self.tracks)
                        for di, (det, _) in enumerate(items) if tr.slot == det.slot), reverse=True)
        used_t, used_d = set(), set()
        for score, ti, di in pairs:
            if score < self.match_iou:
                break
            if ti in used_t or di in used_d:
                continue
            used_t.add(ti)
            used_d.add(di)
            tr, (det, dist) = self.tracks[ti], items[di]
            tr.box, tr.conf, tr.last_seen, tr.truncated = det.box, det.conf, t, truncated[di]
            tr.hits += 1
            tr.history.append(dist)
            while len(tr.history) > self.smooth_frames:
                tr.history.popleft()
            if tr.hits >= self.confirm_frames:
                tr.confirmed = True
        alive = []
        for ti, tr in enumerate(self.tracks):
            if ti in used_t:
                alive.append(tr)
            elif tr.confirmed and t - tr.last_seen <= self.max_missed_s:
                alive.append(tr)                 # mất dấu ngắn: giữ lại với khoảng cách cuối
            # vật CHƯA xác nhận mà mất dấu -> bỏ ngay (nhận nhầm thoáng qua)
        for di, (det, dist) in enumerate(items):
            if di not in used_d:
                tr = Track(self._next_id, det.slot, det.box, det.conf, t, truncated=truncated[di],
                           history=deque([dist]))
                tr.confirmed = self.confirm_frames <= 1
                self._next_id += 1
                alive.append(tr)
        self.tracks = alive
        return self.confirmed()

    def confirmed(self) -> list[Track]:
        return [tr for tr in self.tracks if tr.confirmed]

    def nearest(self) -> dict[str, float]:
        """Khoảng cách GẦN NHẤT của mỗi lớp (vật gần nhất quyết định thời gian va chạm)."""
        out: dict[str, float] = {}
        for tr in self.confirmed():
            if tr.slot not in out or tr.distance < out[tr.slot]:
                out[tr.slot] = tr.distance
        return out
