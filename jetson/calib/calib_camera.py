"""Calib ống kính camera bird's-eye bằng bàn cờ: tìm cameraMatrix (fx, fy, cx, cy) và distCoeffs (k1, k2, p1, p2, k3)
để chỉnh méo, rồi in ra hai dòng C++ dán vào kGroundCalibration trong main.cpp.

Chụp rồi tính (chạy trên Jetson, cần màn hình hoặc ssh -X, TẮT braincar trước vì camera không mở được hai lần):
    python3 calib/calib_camera.py --cols 9 --rows 6 --square 25
Chỉ tính lại từ ảnh đã chụp (không mở camera, chạy được trên PC sau khi scp thư mục ảnh về):
    python3 calib/calib_camera.py --cols 9 --rows 6 --square 25 --calib_only

--cols/--rows là số GÓC TRONG của bàn cờ (giao điểm của 4 ô), không phải số ô: bàn 10x7 ô có 9x6 góc trong.
--square là cạnh một ô (mm); nó chỉ ảnh hưởng vị trí bàn cờ, không ảnh hưởng cameraMatrix và distCoeffs.

Trong cửa sổ chụp:
- Góc bàn cờ vẽ màu khi thấy đủ bàn cờ. Vùng tô xanh mờ là chỗ các ảnh đã lưu đã phủ: cố phủ hết, nhất là 4 góc
  và mép dưới (vùng IPM dùng).
- s: lưu frame gốc (không có hình vẽ) thành <dir>/test<N>.jpg, chỉ khi đang thấy đủ bàn cờ. Chụp bao nhiêu ảnh cũng được.
- z: xoá ảnh vừa lưu | q hoặc ESC: dừng chụp, đóng camera và tính từ TẤT CẢ ảnh test*.jpg có trong thư mục.
Thư mục ảnh đã có test*.jpg thì s chụp tiếp từ số kế tiếp (test1, test2, ... tăng dần); bấm q ngay, không chụp, thì
vẫn tính từ các ảnh có sẵn. Muốn chụp lại từ đầu thì xoá thư mục.

Kết quả: in K, hệ số méo, sai số RMS, góc nhìn, sai số từng ảnh; ghi camera_calib.yaml (cv::FileStorage đọc được) và
undistort_check.jpg (ảnh trước | sau chỉnh méo, đúng như C++ làm: ma trận mới = K). Cạnh bàn cờ phải thẳng ở ảnh sau.
Cần numpy và opencv-python (trên Jetson: OpenCV có GStreamer). Thông báo in ra không dấu để console nào cũng đọc được.
"""

import argparse
import glob
import os
import re
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

# Giữ giống kLaneCameraSource trong main.cpp: K và hệ số méo gắn với độ phân giải, sensor mode và flip-method của
# pipeline này. Đổi một bên thì đổi bên kia và calib lại.
LANE_CAMERA_SOURCE = (
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM), width=1920, height=1080, format=NV12, framerate=30/1 ! "
    "nvvidconv flip-method=2 ! "
    "video/x-raw, width=640, height=480, format=BGRx ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! "
    "appsink drop=true max-buffers=1 sync=false"
)
CALIB_SIZE  = (640, 480)    # kGroundCalibration.imageSize trong main.cpp (ảnh bàn cờ chụp ở 960x540 trước 05/11/2026)
WINDOW      = "calib_camera"
LIVE_SCALE  = 0.5           # dò bàn cờ trên ảnh thu nhỏ lúc xem trực tiếp cho mượt; lúc tính dò lại trên ảnh gốc
BAD_IMAGE_PX = 1.0          # ảnh có sai số trên mức này (hoặc gấp 2 lần trung bình) bị đánh dấu chụp lại

RED, GREEN, WHITE, BLACK = (0, 0, 255), (0, 200, 0), (255, 255, 255), (0, 0, 0)

FIND_FLAGS = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


# ===================================== DÒ BÀN CỜ ====================================
def find_corners(gray, pattern, fast=False):
    """Góc trong của bàn cờ (N x 1 x 2, float32) hoặc None. fast: chỉ kiểm tra nhanh, không tinh chỉnh dưới pixel."""
    flags = FIND_FLAGS | (cv2.CALIB_CB_FAST_CHECK if fast else 0)
    found, corners = cv2.findChessboardCorners(gray, pattern, flags)
    if not found:
        return None
    if not fast:
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), SUBPIX_CRITERIA)
    return corners


def image_paths(folder):
    """test1.jpg, test2.jpg, ... xếp theo số (không theo chữ: test10 sau test9)."""
    paths = glob.glob(os.path.join(folder, "test*.jpg"))
    numbered = []
    for p in paths:
        m = re.fullmatch(r"test(\d+)\.jpg", os.path.basename(p))
        if m:
            numbered.append((int(m.group(1)), p))
    return [p for _, p in sorted(numbered)]


def next_index(folder):
    paths = image_paths(folder)
    if not paths:
        return 1
    return int(re.search(r"(\d+)", os.path.basename(paths[-1])).group(1)) + 1


# ===================================== VẼ ===========================================
def put_text(img, text, org, color, scale=0.6):
    """Chữ có viền đen để đọc được trên nền sáng lẫn tối."""
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, BLACK, 3, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def add_coverage(coverage, corners):
    """Tô vùng bàn cờ (bao lồi các góc) vào mặt nạ phủ."""
    hull = cv2.convexHull(corners.reshape(-1, 2).astype(np.int32))
    cv2.fillConvexPoly(coverage, hull, 255)


def coverage_of(folder, pattern, size):
    """Mặt nạ phủ từ các ảnh đã lưu trong thư mục (khi chụp tiếp lần sau)."""
    coverage = np.zeros((size[1], size[0]), np.uint8)
    for p in image_paths(folder):
        img = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        if img is None or (img.shape[1], img.shape[0]) != size:
            continue
        corners = find_corners(img, pattern, fast=True)
        if corners is not None:
            add_coverage(coverage, corners)
    return coverage


# ===================================== CHỤP ==========================================
def capture(args, pattern):
    """Chụp tới khi bấm q/ESC. Trả về True nếu dừng bình thường (tính tiếp), False nếu lỗi camera."""
    os.makedirs(args.dir, exist_ok=True)
    cap = cv2.VideoCapture(args.source, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("[calib_camera] ERROR: khong mo duoc camera. Da tat braincar chua? OpenCV co GStreamer khong?")
        return False

    saved = []   # (đường dẫn, mặt nạ phủ trước khi lưu ảnh đó) để z hoàn tác được
    index = next_index(args.dir)
    have = len(image_paths(args.dir))
    coverage = None
    print(f"[calib_camera] Da co {have} anh trong {args.dir}. s: luu | z: xoa anh vua luu | "
          "q/ESC: dung chup va tinh tu tat ca anh trong thu muc")
    print("[calib_camera] Phu kin khung hinh (nhat la 4 goc va mep duoi), nghieng ban co nhieu kieu, "
          "xa gan khac nhau, giu yen roi moi nhan s.")

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                print("[calib_camera] ERROR: mat frame camera")
                return False
            size = (frame.shape[1], frame.shape[0])
            if coverage is None:
                if size != CALIB_SIZE:
                    print(f"[calib_camera] WARNING: anh {size[0]}x{size[1]}, khac {CALIB_SIZE[0]}x{CALIB_SIZE[1]} "
                          "cua kGroundCalibration")
                coverage = coverage_of(args.dir, pattern, size)

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            small = cv2.resize(gray, None, fx=LIVE_SCALE, fy=LIVE_SCALE, interpolation=cv2.INTER_AREA)
            corners = find_corners(small, pattern, fast=True)
            if corners is not None:
                corners = corners / LIVE_SCALE

            view = frame.copy()
            tint = view.copy()
            tint[coverage > 0] = GREEN
            view = cv2.addWeighted(tint, 0.25, view, 0.75, 0)
            if corners is not None:
                cv2.drawChessboardCorners(view, pattern, corners.astype(np.float32), True)
                put_text(view, f"{have} anh  thay ban co - nhan s de luu, q de tinh", (10, 25), GREEN)
            else:
                put_text(view, f"{have} anh  KHONG thay du ban co", (10, 25), RED)
            cv2.imshow(WINDOW, view)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                if corners is None:
                    print("[calib_camera] Khong thay du ban co, khong luu.")
                    continue
                path = os.path.join(args.dir, f"test{index}.jpg")
                if not cv2.imwrite(path, frame):
                    print(f"[calib_camera] ERROR: khong ghi duoc {path}")
                    continue
                saved.append((path, coverage.copy()))
                add_coverage(coverage, corners)
                index += 1
                have += 1
                print(f"[calib_camera] Luu {path} ({have} anh)")
            elif key == ord("z") and saved:
                path, coverage = saved.pop()
                os.remove(path)
                index -= 1
                have -= 1
                print(f"[calib_camera] Xoa {path} ({have} anh)")
    finally:
        cap.release()
        cv2.destroyAllWindows()
    return True


# ===================================== TÍNH ==========================================
def calibrate(args, pattern):
    paths = image_paths(args.dir)
    if not paths:
        print(f"[calib_camera] ERROR: khong co anh test*.jpg trong {args.dir}")
        return 1

    # Toạ độ các góc trên bàn cờ (mm), mặt phẳng Z = 0. Cỡ ô chỉ đổi tvec, không đổi K và hệ số méo.
    board = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    board[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2) * args.square

    obj_points, img_points, used, size = [], [], [], None
    for p in paths:
        gray = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            print(f"[calib_camera] Bo {os.path.basename(p)}: khong doc duoc")
            continue
        s = (gray.shape[1], gray.shape[0])
        if size is None:
            size = s
        elif s != size:
            print(f"[calib_camera] Bo {os.path.basename(p)}: kich thuoc {s[0]}x{s[1]} khac {size[0]}x{size[1]}")
            continue
        corners = find_corners(gray, pattern)
        if corners is None:
            print(f"[calib_camera] Bo {os.path.basename(p)}: khong thay du ban co {pattern[0]}x{pattern[1]}")
            continue
        obj_points.append(board)
        img_points.append(corners)
        used.append(p)

    print(f"[calib_camera] Dung {len(used)}/{len(paths)} anh")
    if len(used) < 10:
        print("[calib_camera] ERROR: can it nhat 10 anh thay du ban co (nen 20-30)")
        return 1
    if size != CALIB_SIZE:
        print(f"[calib_camera] WARNING: anh {size[0]}x{size[1]} khac {CALIB_SIZE[0]}x{CALIB_SIZE[1]} cua "
              "kGroundCalibration: so lieu duoi day KHONG dung cho main.cpp")

    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, size, None, None)
    dist = dist.ravel()[:5]

    # Sai số từng ảnh: chiếu lại các góc bằng mô hình vừa tìm, so với góc đo được (px, RMS)
    errors = []
    for obj, img, rvec, tvec in zip(obj_points, img_points, rvecs, tvecs):
        proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
        errors.append(float(np.sqrt(np.mean(np.sum((proj - img) ** 2, axis=2)))))
    mean_err = float(np.mean(errors))

    print("\n=== Sai so tung anh (px) ===")
    for p, e in zip(used, errors):
        bad = e > BAD_IMAGE_PX or e > 2 * mean_err
        print(f"  {os.path.basename(p):>10}  {e:.3f}" + ("   <-- chup lai" if bad else ""))

    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    hfov = 2 * np.degrees(np.arctan(size[0] / (2 * fx)))
    vfov = 2 * np.degrees(np.arctan(size[1] / (2 * fy)))
    k1, k2, p1, p2, k3 = dist
    print("\n=== Ket qua ===")
    print(f"  RMS          = {rms:.3f} px   (< 0.5 tot, > 1 co anh hong)")
    print(f"  fx, fy       = {fx:.2f}, {fy:.2f} px   (pixel vuong nen gan bang nhau)")
    print(f"  cx, cy       = {cx:.2f}, {cy:.2f} px   (giua anh: {size[0] / 2:.0f}, {size[1] / 2:.0f})")
    print(f"  HFOV, VFOV   = {hfov:.1f}, {vfov:.1f} do")
    print(f"  k1 k2 p1 p2 k3 = {k1:.6f} {k2:.6f} {p1:.6f} {p2:.6f} {k3:.6f}   (k1 < 0: meo thung)")

    print("\n=== Dan vao kGroundCalibration trong main.cpp (thay 2 khoi 0.0 cua cameraMatrix va distCoeffs) ===")
    print(f"    cv::Matx33d({fx:.6f}, 0.0, {cx:.6f},    // fx, 0, cx")
    print(f"                0.0, {fy:.6f}, {cy:.6f},    // 0, fy, cy")
    print("                0.0, 0.0, 1.0),")
    print(f"    cv::Vec<double, 5>({k1:.8f}, {k2:.8f}, {p1:.8f}, {p2:.8f}, {k3:.8f}), // k1, k2, p1, p2, k3")

    yaml_path = os.path.join(args.out, "camera_calib.yaml")
    fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_WRITE)
    fs.write("image_width", size[0])
    fs.write("image_height", size[1])
    fs.write("camera_matrix", K)
    fs.write("dist_coeffs", dist.reshape(1, 5))
    fs.write("rms", float(rms))
    fs.release()

    # Ảnh kiểm tra: giống C++ (initUndistortRectifyMap với ma trận mới = K, remap)
    check = cv2.imread(used[int(np.argmax([np.max(np.abs(c[:, 0, 0] - cx)) for c in img_points]))])
    map1, map2 = cv2.initUndistortRectifyMap(K, dist, None, K, size, cv2.CV_16SC2)
    undist = cv2.remap(check, map1, map2, cv2.INTER_LINEAR)
    put_text(check, "goc (meo)", (10, 25), RED)
    put_text(undist, "da chinh meo (giong C++)", (10, 25), GREEN)
    check_path = os.path.join(args.out, "undistort_check.jpg")
    cv2.imwrite(check_path, np.hstack([check, undist]))
    print(f"\n[calib_camera] Ghi {yaml_path} va {check_path}: canh ban co o anh ben phai phai thang.")
    return 0


def main():
    parser = argparse.ArgumentParser(description="Calib ong kinh camera bird's-eye bang ban co (chinh meo)")
    parser.add_argument("--cols", type=int, required=True, help="so goc trong theo chieu ngang (so o - 1)")
    parser.add_argument("--rows", type=int, required=True, help="so goc trong theo chieu doc (so o - 1)")
    parser.add_argument("--square", type=float, default=25.0, help="canh mot o (mm), khong anh huong K va he so meo")
    parser.add_argument("--dir", default=os.path.join(HERE, "chessboard"), help="thu muc chi chua anh chup")
    parser.add_argument("--out", default=HERE, help="thu muc ghi camera_calib.yaml va undistort_check.jpg")
    parser.add_argument("--source", default=LANE_CAMERA_SOURCE, help="pipeline GStreamer (mac dinh = kLaneCameraSource)")
    parser.add_argument("--calib_only", action="store_true", help="chi tinh tu anh da co, khong mo camera")
    args = parser.parse_args()
    pattern = (args.cols, args.rows)

    if not args.calib_only and not capture(args, pattern):
        return 1
    return calibrate(args, pattern)


if __name__ == "__main__":
    sys.exit(main())
