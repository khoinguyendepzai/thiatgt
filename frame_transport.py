"""Jetson JPEG transport (>IHH, <=1300-byte payload), no vehicle I/O.

Timestamps are server first-chunk receive times, not camera capture times.
Frame IDs are independent per camera; never pair cameras by frame ID alone.
"""
from collections import OrderedDict, deque
from dataclasses import dataclass
import math
import socket
import struct
import threading
import time

import cv2
import numpy as np


@dataclass(frozen=True)
class AssembledFrame:
    jpeg: bytes
    frame_id: int
    t: float
    source: tuple


class FrameAssembler:
    HEADER = struct.Struct('>IHH')
    MAX_FRAME_BYTES = 4 * 1024 * 1024
    MAX_PAYLOAD = 1300

    def __init__(self, stale_s=1.0, max_partial=8):
        self.stale_s = stale_s
        self.max_partial = max_partial
        self.partial = OrderedDict()
        self.sources = OrderedDict()
        self.invalid = self.duplicates = self.expired = self.completed = 0

    def expire(self, now):
        for key, entry in list(self.partial.items()):
            if now - entry['t'] > self.stale_s:
                self.partial.pop(key)
                self.expired += 1

    def push(self, packet, now, source):
        self.expire(now)
        if not self.HEADER.size < len(packet) <= self.HEADER.size + self.MAX_PAYLOAD:
            self.invalid += 1
            return None
        fid, idx, total = self.HEADER.unpack_from(packet)
        payload = packet[self.HEADER.size:]
        if (not total or idx >= total or total*self.MAX_PAYLOAD > self.MAX_FRAME_BYTES
                or (idx < total-1 and len(payload) != self.MAX_PAYLOAD)):
            self.invalid += 1
            return None
        state = self.sources.get(source)
        if state is None:
            if len(self.sources) >= 8:
                old, _ = self.sources.popitem(last=False)
                for key in list(self.partial):
                    if key[0] == old:
                        self.partial.pop(key)
                        self.expired += 1
            state = self.sources[source] = {'last': None, 'activity': now}
        self.sources.move_to_end(source)
        # The protocol carries no boot epoch. A lower ID is accepted after an
        # interval with no accepted progress; rejected old IDs must not refresh it.
        if state['last'] is not None and fid <= state['last']:
            if now - state['activity'] > self.stale_s:
                state['last'] = None
                for key in list(self.partial):
                    if key[0] == source:
                        self.partial.pop(key)
                        self.expired += 1
            else:
                self.duplicates += 1
                return None
        state['activity'] = now
        key = source, fid
        entry = self.partial.get(key)
        if entry is None:
            while len(self.partial) >= self.max_partial:
                self.partial.popitem(last=False)
                self.expired += 1
            entry = self.partial[key] = {'total': total, 'parts': {}, 't': now}
        if entry['total'] != total:
            self.partial.pop(key)
            self.invalid += 1
            return None
        if idx in entry['parts']:
            self.duplicates += 1
            return None
        entry['parts'][idx] = payload
        if len(entry['parts']) != total:
            return None
        jpeg = b''.join(entry['parts'][i] for i in range(total))
        state['last'] = fid
        self.completed += 1
        for pending in list(self.partial):
            if pending[0] == source and pending[1] <= fid:
                self.partial.pop(pending)
                if pending != key:
                    self.expired += 1
        return AssembledFrame(jpeg, fid, entry['t'], source)


class FrameReceiver:
    """Continuously assemble/decode in background; consumers get latest full image.

    frame_size=None preserves dimensions. A specified size is validated, never
    resized silently (camera calibration depends on the original geometry).
    """
    WAIT_S = 0.1

    def __init__(self, port, frame_size=None, *, source_ip=None, listen_ip='0.0.0.0'):
        self.frame_size = frame_size
        self.source_ip = source_ip
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4*1024*1024)
        self.sock.bind((listen_ip, port))
        self.sock.settimeout(.05)
        self.assembler = FrameAssembler()
        self._cv = threading.Condition()
        self._stop = threading.Event()
        self._latest = None
        self._sequence = self._delivered_sequence = 0
        self.last_frame_id = None
        self.last_source = None
        self._counts = dict(packets=0, decoded=0, delivered=0, skipped=0,
                            decode_errors=0, size_errors=0, filtered=0)
        self.error = None
        self._thread = threading.Thread(target=self._run, name=f'udp-rx-{port}', daemon=True)
        self._thread.start()

    @property
    def skipped(self):
        with self._cv:
            return self._counts['skipped']

    def _run(self):
        try:
            while not self._stop.is_set():
                try:
                    packet, source = self.sock.recvfrom(65535)
                except socket.timeout:
                    with self._cv:
                        self.assembler.expire(time.perf_counter())
                    continue
                except ConnectionResetError:
                    continue
                except OSError:
                    if self._stop.is_set():
                        break
                    raise
                now = time.perf_counter()
                with self._cv:
                    self._counts['packets'] += 1
                    if self.source_ip is not None and source[0] != self.source_ip:
                        self._counts['filtered'] += 1
                        continue
                    assembled = self.assembler.push(packet, now, source)
                    if assembled is None:
                        continue
                    try:
                        image = cv2.imdecode(np.frombuffer(assembled.jpeg, np.uint8), cv2.IMREAD_COLOR)
                    except cv2.error:
                        image = None
                    if image is None:
                        self._counts['decode_errors'] += 1
                        continue
                    if self.frame_size is not None and (image.shape[1], image.shape[0]) != self.frame_size:
                        self._counts['size_errors'] += 1
                        continue
                    if self._sequence > self._delivered_sequence:
                        self._counts['skipped'] += 1
                    self._sequence += 1
                    self._counts['decoded'] += 1
                    self._latest = image, assembled
                    self._cv.notify_all()
        except Exception as exc:
            with self._cv:
                self.error = repr(exc)
                self._cv.notify_all()

    def latest(self, timeout=None):
        deadline = time.perf_counter() + (self.WAIT_S if timeout is None else timeout)
        with self._cv:
            while self._sequence == self._delivered_sequence and not self._stop.is_set():
                if self.error:
                    raise RuntimeError(f'UDP receiver failed: {self.error}')
                remaining = deadline - time.perf_counter()
                if remaining <= 0:
                    return None, None
                self._cv.wait(remaining)
            if self._sequence == self._delivered_sequence:
                return None, None
            image, info = self._latest
            self._delivered_sequence = self._sequence
            self.last_frame_id, self.last_source = info.frame_id, info.source
            self._counts['delivered'] += 1
            return image, info.t

    def stats(self):
        with self._cv:
            info = self._latest[1] if self._latest is not None else None
            return dict(self._counts, assembled=self.assembler.completed,
                        invalid=self.assembler.invalid, duplicates=self.assembler.duplicates,
                        expired=self.assembler.expired, partial=len(self.assembler.partial),
                        last_frame_id=info.frame_id if info else None,
                        last_source=info.source if info else None,
                        last_frame_age_s=time.perf_counter()-info.t if info else None,
                        error=self.error)

    def close(self):
        self._stop.set()
        with self._cv:
            self._cv.notify_all()
        self.sock.close()
        self._thread.join(timeout=1)


class ResultHistory:
    """Bounded causal pairing based on server receive time, with wall-clock expiry."""
    def __init__(self, max_skew_s=.1, max_age_s=.5):
        if not (math.isfinite(max_skew_s) and math.isfinite(max_age_s)
                and 0 < max_skew_s <= max_age_s):
            raise ValueError('Require 0 < max_skew_s <= max_age_s')
        self.max_skew_s, self.max_age_s = max_skew_s, max_age_s
        self._items = deque(maxlen=64)
        self._lock = threading.Lock()

    def add(self, result):
        if not math.isfinite(result.t):
            return
        with self._lock:
            self._items.append(result)

    def pick(self, lane_t, now=None):
        now = time.perf_counter() if now is None else now
        if not (math.isfinite(lane_t) and math.isfinite(now)) or not 0 <= now-lane_t <= self.max_age_s:
            return None
        with self._lock:
            valid = [r for r in self._items
                     if 0 <= lane_t-r.t <= self.max_skew_s and 0 <= now-r.t <= self.max_age_s]
        return max(valid, key=lambda r: r.t) if valid else None
