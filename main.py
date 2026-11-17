"""LDWS trên PC cho xe mô hình 1:10: nhận ảnh camera từ Jetson, cảnh báo hai mức theo vị trí — LỆCH LÀN (vàng) và CHẠM
VẠCH (đỏ). Không dùng vận tốc hay dự báo.

    main.py          thông số đầu vào (calib, sa bàn và xe, ISO), nhận frame (UDP / video / ảnh), nối hai phần, ghi CSV
    lane_vision.py   phần 1: xử lý ảnh -> thông số hình học của làn (LaneGeometry), vẽ hiển thị
    lane_warning.py  phần 2: LaneGeometry -> LOR -> ngưỡng, debounce, hysteresis -> mức cảnh báo (WarningResult)

Jetson (ThreadUdpServer) gửi hai luồng ảnh JPEG, mỗi ảnh cắt thành nhiều gói UDP:
    cổng 7000: camera bám làn 640x480 ĐÃ KHỬ MÉO trên Jetson -> dò làn + cảnh báo (PC không khử méo lại)
    cổng 7001: camera nhìn đường (người lái) -> chỉ hiển thị
Chạy với xe:
    python main.py
Chạy lại video hoặc thư mục ảnh đã ghi (ảnh đã khử méo; thời gian lấy theo fps):
    python main.py --video run.mp4 --csv ketqua.csv
    python main.py --images thu_muc_anh --fps 30 --no-show --csv ketqua.csv
"""

import argparse
import csv
import glob
import os
import select
import socket
import struct
import time
from dataclasses import dataclass, field

import cv2
import numpy as np

from lane_vision import LaneVision
from lane_warning import LaneWarning

LANE_PORT = 7000  # camera bám làn 640x480 đã khử méo (lanePort của ThreadUdpServer::add_client)
ROAD_PORT = 7001  # camera nhìn đường (port của ThreadUdpServer::add_client)


# =====================================================================
# 1. THÔNG SỐ ĐẦU VÀO: CALIBRATION, SA BÀN & XE, TIÊU CHÍ ĐÁNH GIÁ
# =====================================================================
@dataclass
class GroundCalibration:
    """Calib mặt đường cho ảnh ĐÃ KHỬ MÉO. K và hệ số méo nằm ở Jetson (kCameraCalibration trong main.cpp, Jetson khử
    méo trước khi gửi); homography ở đây phải khớp đúng bộ K đó."""

    width: int = 640
    height: int = 480

    # Ma trận Homography: pixel ảnh đã khử méo -> mặt đường (m), gốc tại tâm cầu sau
    # Mảng numpy làm mặc định của dataclass phải qua default_factory (Python >= 3.11 báo lỗi nếu gán thẳng)
    homography: np.ndarray = field(
        default_factory=lambda: np.array(
            [
                [1.38382102e-05, 0.000345778475, 1.08880963],
                [-0.00153570693, 3.85623778e-05, 0.442112946],
                [2.11989362e-05, 0.00410399309, 1.0],
            ],
            dtype=np.float64,
        )
    )


@dataclass
class TrackAndVehicleParams:
    # Số đo sa bàn từ kTrackParams (main.cpp)
    lane_gap: float = 0.35  # Khoảng trống giữa mép trong 2 vạch (m)
    line_width: float = 0.03  # Bề rộng vạch làn (m)
    dash_length: float = 0.13  # Chiều dài 1 nét đứt (m)
    dash_gap: float = 0.20  # Khe hở giữa 2 nét đứt (m)
    crosswalk_stripe_width: float = 0.05  # Bề rộng sọc đi bộ (m)

    # Kích thước xe từ kLaneParams & kLaneChangeParams (main.cpp)
    wheelbase: float = 0.258  # Chiều dài cơ sở (m)
    vehicle_width: float = 0.185 # Chiều rộng xe (m)
    rear_axle_to_front: float = 0.35  # Cầu sau -> đầu xe (m)

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

    def process_undistorted(self, undistorted, t):
        """Frame đã khử méo (từ Jetson) -> (dòng log, WarningResult, vis_cam, vis_bev, mask)."""
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


def show_ldws(vis_cam, vis_bev, mask):
    cv2.imshow("Camera Undistorted + LDWS Overlay", vis_cam)
    cv2.imshow("Bird's-Eye View (BEV)", vis_bev)
    cv2.imshow("Filtered Lane Mask", mask)


def poll_keys(wait_ms=1) -> bool:
    """Xử lý phím. Trả về False khi người dùng bấm q/ESC."""
    key = cv2.waitKey(wait_ms) & 0xFF
    if key == ord(" "):  # tạm dừng, bấm phím bất kỳ để chạy tiếp
        key = cv2.waitKey(0) & 0xFF
    return key not in (27, ord("q"))


def show_frames(vis_cam, vis_bev, mask, wait_ms=1) -> bool:
    """Hiện 3 cửa sổ LDWS. Trả về False khi người dùng bấm q/ESC."""
    show_ldws(vis_cam, vis_bev, mask)
    return poll_keys(wait_ms)


class FrameAssembler:
    """Ghép các gói UDP của một luồng ảnh thành ảnh JPEG (giao thức của ThreadUdpServer trên Jetson).

    Mỗi gói: [frameId u32][chunkIndex u16][totalChunks u16][payload], big-endian. Ảnh nào thiếu gói thì bỏ (quá
    STALE_S hoặc đã có ảnh mới hơn hoàn chỉnh). frameId nhỏ hơn hẳn ảnh vừa ghép xong nghĩa là Jetson vừa chạy lại.
    """

    HEADER = struct.Struct(">IHH")
    STALE_S = 1.0
    RESTART_GAP = 1000  # frameId lùi quá chừng này thì coi như Jetson chạy lại, đếm lại từ đầu

    def __init__(self):
        self.partial = {}  # frameId -> [total, {index: payload}, thời điểm gói đầu]
        self.last_done = 0

    def push(self, packet, now):
        """Thêm một gói. Trả về bytes JPEG khi ghép đủ một ảnh mới, ngược lại None."""
        if len(packet) <= self.HEADER.size:
            return None
        fid, idx, total = self.HEADER.unpack_from(packet)
        if total == 0 or idx >= total:
            return None
        if fid <= self.last_done:
            if self.last_done - fid < self.RESTART_GAP:
                return None  # gói của ảnh cũ hơn ảnh đã hiện
            self.last_done, self.partial = 0, {}

        entry = self.partial.get(fid)
        if entry is None or entry[0] != total:
            entry = self.partial[fid] = [total, {}, now]
        entry[1][idx] = packet[self.HEADER.size:]
        if len(entry[1]) < total:
            if len(self.partial) > 8:
                self._drop_stale(now)
            return None

        jpeg = b"".join(entry[1][i] for i in range(total))
        self.last_done = fid
        self.partial = {k: v for k, v in self.partial.items() if k > fid}
        return jpeg

    def _drop_stale(self, now):
        self.partial = {k: v for k, v in self.partial.items() if now - v[2] < self.STALE_S}


def drain_socket(sock, assembler, now):
    """Đọc hết gói đang chờ trên socket không chặn. Trả về JPEG hoàn chỉnh mới nhất (hoặc None)."""
    latest = None
    while True:
        try:
            packet = sock.recv(65535)
        except (BlockingIOError, InterruptedError):
            return latest
        except ConnectionResetError:  # Windows báo ICMP port unreachable qua recv: bỏ qua
            continue
        jpeg = assembler.push(packet, now)
        if jpeg is not None:
            latest = jpeg


def open_udp(ip, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    sock.bind((ip, port))
    sock.setblocking(False)
    return sock


def run_udp_ldws_receiver(args):
    """Nhận hai luồng ảnh từ Jetson: camera bám làn (đã khử méo) -> LDWS, camera nhìn đường -> chỉ hiển thị."""
    ldws = Ldws()
    log = ResultLog(args.csv)
    size = (ldws.calib.width, ldws.calib.height)

    lane_sock = open_udp(args.listen_ip, args.port)
    road_sock = open_udp(args.listen_ip, args.road_port)
    lane_asm, road_asm = FrameAssembler(), FrameAssembler()
    size_warned = False
    print(f"[PC LDWS] Dang lang nghe UDP {args.listen_ip}: cam bam lan (da khu meo) cong {args.port}, "
          f"cam nhin duong cong {args.road_port}... (Ctrl+C hoac q de thoat)", flush=True)

    try:
        while True:
            ready, _, _ = select.select([lane_sock, road_sock], [], [], 0.05)
            now = time.perf_counter()
            lane_jpeg = drain_socket(lane_sock, lane_asm, now) if lane_sock in ready else None
            road_jpeg = drain_socket(road_sock, road_asm, now) if road_sock in ready else None

            # Chỉ xử lý ảnh làn mới nhất: đang chậm thì bỏ ảnh cũ, không để trễ dồn lại
            if lane_jpeg is not None:
                frame = cv2.imdecode(np.frombuffer(lane_jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    if (frame.shape[1], frame.shape[0]) != size:
                        if not size_warned:
                            print(f"[PC LDWS] CANH BAO: anh cam bam lan {frame.shape[1]}x{frame.shape[0]}, can "
                                  f"{size[0]}x{size[1]} (Jetson khong khu meo duoc anh sai co calib)", flush=True)
                            size_warned = True
                        frame = cv2.resize(frame, size)
                    row, _, vis_cam, vis_bev, mask = ldws.process_undistorted(frame, now)
                    log.write(row)
                    if not args.no_show:
                        show_ldws(vis_cam, vis_bev, mask)

            if road_jpeg is not None and not args.no_show:
                road = cv2.imdecode(np.frombuffer(road_jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if road is not None:
                    cv2.imshow("Road Camera", road)

            if not args.no_show and not poll_keys(1):
                break
    finally:
        lane_sock.close()
        road_sock.close()
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
            row, res, vis_cam, vis_bev, mask = ldws.process_undistorted(frame, t)
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
    src.add_argument("--video", help="chay lai tu file video (anh da khu meo) thay vi nhan UDP")
    src.add_argument("--images", help="chay lai tu thu muc anh hoac mau glob (sap xep theo ten)")
    parser.add_argument("--fps", type=float, default=30.0, help="fps cua chuoi anh (hoac video khong ghi fps)")
    parser.add_argument("--delay", type=int, default=1, help="ms cho giua 2 frame khi chay lai co hien thi")
    parser.add_argument("--csv", help="ghi ket qua tung frame ra file CSV")
    parser.add_argument("--no-show", action="store_true", help="khong mo cua so hien thi")
    parser.add_argument("--listen-ip", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=LANE_PORT, help="cong UDP nhan anh cam bam lan (da khu meo)")
    parser.add_argument("--road-port", type=int, default=ROAD_PORT, help="cong UDP nhan anh cam nhin duong")
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
