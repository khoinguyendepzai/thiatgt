"""Chọn mốc calib mặt đường bằng chuột trên test.jpg, rồi tính kGroundCalibration (pure_pursit.md mục 7.8).

Thay cho bước tra pixel bằng Paint và gõ file CSV tay:
    python calib/pick_marks.py test.jpg marks.csv [--lane_width 0.43]

Trong cửa sổ ảnh:
- Click chuột trái vào tâm một mốc, rồi gõ x y của mốc đó vào cửa sổ dòng lệnh (m, đo từ tâm cầu sau, x tới trước,
  y sang trái, ví dụ "0.75 -0.12"). Enter bỏ trống = bỏ click đó. Trong lúc gõ, cửa sổ ảnh đứng yên.
- Ô phóng to ở góc trên cho thấy vùng quanh con trỏ (x5) để click đúng tâm mốc.
- z: xoá mốc vừa thêm | q hoặc ESC: xong.
File CSV được ghi lại sau mỗi lần thêm/xoá. Chạy lại với file đã có thì các mốc cũ được giữ và vẽ lên ảnh, click để thêm.

Khử méo: LaneDetector khử méo ảnh (cv::remap với cameraMatrix, distCoeffs của kGroundCalibration) trước IPM, nên
homography phải tính trên ảnh ĐÃ khử méo. test.jpg (phím s) là ảnh thô, nên trước khi chọn mốc script khử méo nó đúng
như C++ (initUndistortRectifyMap với ma trận mới = cameraMatrix, rồi remap), lưu thêm <ảnh>_undist.jpg, và mọi toạ độ
u, v trong CSV là toạ độ trên ảnh đã khử méo. Bộ số lấy từ main.cpp (mặc định, để luôn khớp với C++) hoặc từ
camera_calib.yaml của calib_camera.py (--camera_calib). --no_undistort: dùng ảnh thô (khi main.cpp chưa có bộ số).

Xong mà có từ 4 mốc: in kết quả như calib_homography.py (sai số từng mốc, dòng cv::Matx33d để dán vào main.cpp), rồi
hiện và lưu <csv>_check.jpg: lưới mặt đường mỗi 10 cm đổi ngược lên ảnh (trục giữa xe màu đỏ, hai vạch của làn
có xe nằm giữa màu vàng). Ma trận đúng thì lưới nằm phẳng trên sàn, khớp với các vạch thật.
Cần numpy và opencv-python. Thông báo in ra không dấu để console nào cũng đọc được.
"""

import argparse
import os
import re
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import calib_homography as ch  # noqa: E402

WINDOW      = "pick_marks"
MAX_VIEW_W  = 1280   # ảnh hiển thị thu nhỏ cho vừa màn hình; toạ độ click được đổi về pixel ảnh gốc
ZOOM        = 5      # ô phóng to: (2·ZOOM_HALF + 1)² pixel ảnh gốc quanh con trỏ, phóng ZOOM lần
ZOOM_HALF   = 20
CALIB_SIZE  = (640, 480)    # kGroundCalibration.imageSize trong main.cpp
MAIN_CPP    = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "main.cpp")
UNDIST_TAG  = "anh da khu meo"   # có trong dòng đầu CSV khi u, v là toạ độ trên ảnh đã khử méo

RED, GREEN, YELLOW, WHITE, BLACK = (0, 0, 255), (0, 200, 0), (0, 255, 255), (255, 255, 255), (0, 0, 0)
MAGENTA, BLUE = (255, 0, 255), (255, 128, 0)


# ===================================== FILE MỐC =====================================
def load_marks(path):
    """Danh sách (u, v, x, y) từ file CSV, rỗng nếu chưa có file."""
    if not os.path.exists(path):
        return []
    pix, ground = ch.read_marks(path)
    return [(float(u), float(v), float(x), float(y)) for (u, v), (x, y) in zip(pix, ground)]


def marks_undistorted(path):
    """True nếu CSV ghi u, v là toạ độ trên ảnh đã khử méo (dòng đầu do save_marks ghi)."""
    with open(path, encoding="utf-8") as f:
        return UNDIST_TAG in f.readline()


def save_marks(path, marks, undistorted):
    where = UNDIST_TAG if undistorted else "anh tho test.jpg"
    with open(path, "w", encoding="utf-8") as f:
        f.write(f"# u, v (pixel tren {where}), x, y (m, do tu tam cau sau; x toi truoc, y sang trai)\n")
        for u, v, x, y in marks:
            f.write(f"{u:.1f}, {v:.1f}, {x:.3f}, {y:.3f}\n")


# ===================================== KHỬ MÉO ======================================
def lens_from_main_cpp(path):
    """(K, dist) trong kGroundCalibration của main.cpp: dòng cv::Matx33d(...) có chú thích "fx, 0, cx" và dòng
    cv::Vec<double, 5>(...). None nếu không thấy, hoặc fx/fy còn bằng 0 (chưa calib: C++ không khử méo)."""
    with open(path, encoding="utf-8") as f:
        text = f.read()
    m = re.search(r"cv::Matx33d\(([^)]*fx, 0, cx[^)]*)\)", text)
    d = re.search(r"cv::Vec<double,\s*5>\(([^)]*)\)", text)
    if not m or not d:
        return None
    num = r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?"
    k_vals = [float(t) for t in re.findall(num, re.sub(r"//[^\n]*", "", m.group(1)))]
    d_vals = [float(t) for t in re.findall(num, d.group(1))]
    if len(k_vals) != 9 or len(d_vals) != 5:
        return None
    K = np.array(k_vals).reshape(3, 3)
    if K[0, 0] <= 0 or K[1, 1] <= 0:
        return None
    return K, np.array(d_vals)


def lens_from_yaml(path):
    """(K, dist) từ camera_calib.yaml của calib_camera.py, None nếu không đọc được."""
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        return None
    K, dist = fs.getNode("camera_matrix").mat(), fs.getNode("dist_coeffs").mat()
    fs.release()
    if K is None or dist is None:
        return None
    return K, dist.ravel()[:5]


def undistort(image, K, dist):
    """Giống LaneDetector: initUndistortRectifyMap với ma trận mới = K, rồi remap (ảnh cùng cỡ, cùng K)."""
    size = (image.shape[1], image.shape[0])
    map1, map2 = cv2.initUndistortRectifyMap(K, dist, None, K, size, cv2.CV_16SC2)
    return cv2.remap(image, map1, map2, cv2.INTER_LINEAR)


# ===================================== VẼ ===========================================
def put_text(img, text, org, color, scale=0.5):
    """Chữ có viền đen để đọc được trên nền sáng lẫn tối."""
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, BLACK, 3, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def draw_marks(view, marks, scale):
    for i, (u, v, x, y) in enumerate(marks, 1):
        p = (int(round(u * scale)), int(round(v * scale)))
        cv2.drawMarker(view, p, MAGENTA, cv2.MARKER_CROSS, 16, 2)
        put_text(view, f"{i}: ({x:.2f}, {y:.2f})", (p[0] + 8, p[1] - 8), MAGENTA)


def draw_zoom(view, image, u, v, scale):
    """Ô phóng to vùng quanh (u, v) (pixel ảnh gốc), đặt ở góc trên phía không có con trỏ."""
    u0 = int(np.clip(round(u), 0, image.shape[1] - 1))
    v0 = int(np.clip(round(v), 0, image.shape[0] - 1))
    padded = cv2.copyMakeBorder(image, ZOOM_HALF, ZOOM_HALF, ZOOM_HALF, ZOOM_HALF, cv2.BORDER_CONSTANT)
    patch = padded[v0:v0 + 2 * ZOOM_HALF + 1, u0:u0 + 2 * ZOOM_HALF + 1]
    big = cv2.resize(patch, None, fx=ZOOM, fy=ZOOM, interpolation=cv2.INTER_NEAREST)
    c = ZOOM_HALF * ZOOM + ZOOM // 2
    cv2.line(big, (c, 0), (c, big.shape[0] - 1), GREEN, 1)
    cv2.line(big, (0, c), (big.shape[1] - 1, c), GREEN, 1)
    size = big.shape[0]
    x0 = view.shape[1] - size - 10 if u * scale < view.shape[1] / 2 else 10
    view[10:10 + size, x0:x0 + size] = big
    cv2.rectangle(view, (x0, 10), (x0 + size - 1, 10 + size - 1), WHITE, 1)
    put_text(view, f"u={u:.0f} v={v:.0f}", (x0 + 4, 10 + size + 16), WHITE)


def project_to_image(H, ground_pts):
    """Điểm mặt đường (m) -> pixel; NaN cho điểm sau đường chân trời (camera không thấy)."""
    G = np.linalg.inv(H)
    q = np.hstack([ground_pts, np.ones((len(ground_pts), 1))]) @ G.T
    # Điểm camera thấy được có w cùng dấu với tâm ảnh đổi ra mặt đường
    centre = H @ np.array([CALIB_SIZE[0] / 2.0, CALIB_SIZE[1] - 1.0, 1.0])
    sign = np.sign((G @ (centre / centre[2]))[2])
    pix = q[:, :2] / q[:, 2:3]
    pix[q[:, 2] * sign <= 1e-9] = np.nan
    return pix


def draw_ground_line(img, H, p0, p1, color, thickness):
    pts = project_to_image(H, np.linspace(p0, p1, 60))
    limit = 4 * max(img.shape[:2])   # bỏ đoạn gần chân trời: toạ độ quá lớn
    for a, b in zip(pts[:-1], pts[1:]):
        if np.isnan(a).any() or np.isnan(b).any() or np.abs(np.r_[a, b]).max() > limit:
            continue
        cv2.line(img, tuple(int(round(t)) for t in a), tuple(int(round(t)) for t in b), color, thickness, cv2.LINE_AA)


def draw_check(image, H, marks, lane_width):
    """Ảnh kiểm tra: lưới mặt đường 10 cm, trục giữa xe, hai vạch làn, mốc click (hồng) và vị trí H đoán (xanh)."""
    out = image.copy()
    xs = np.round(np.arange(0.3, 1.51, 0.1), 2)
    ys = np.round(np.arange(-0.6, 0.61, 0.1), 2)
    for x in xs:
        draw_ground_line(out, H, (x, ys[0]), (x, ys[-1]), GREEN, 1)
    for y in ys:
        if abs(y) > 1e-9:
            draw_ground_line(out, H, (xs[0], y), (xs[-1], y), GREEN, 1)
    draw_ground_line(out, H, (xs[0], 0.0), (xs[-1], 0.0), RED, 2)
    for y in (lane_width / 2.0, -lane_width / 2.0):
        draw_ground_line(out, H, (xs[0], y), (xs[-1], y), YELLOW, 2)
    for x, p in zip(xs, project_to_image(H, np.c_[xs, np.zeros_like(xs)])):
        if not np.isnan(p).any() and 0 <= p[0] < out.shape[1] and 0 <= p[1] < out.shape[0]:
            put_text(out, f"x={x:.1f}", (int(p[0]) + 6, int(p[1]) - 4), RED, 0.6)

    predicted = project_to_image(H, np.array([[x, y] for _, _, x, y in marks]))
    for i, ((u, v, x, y), p) in enumerate(zip(marks, predicted), 1):
        cv2.drawMarker(out, (int(round(u)), int(round(v))), MAGENTA, cv2.MARKER_CROSS, 24, 2)
        if not np.isnan(p).any():
            cv2.circle(out, (int(round(p[0])), int(round(p[1]))), 9, BLUE, 2)
        put_text(out, f"{i}", (int(round(u)) + 10, int(round(v)) + 20), MAGENTA, 0.7)
    put_text(out, "luoi 10 cm | do: truc giua xe (y=0) | vang: vach lan +-%.3f m | hong: moc click | "
                  "tron xanh: H doan" % (lane_width / 2.0), (10, out.shape[0] - 12), WHITE, 0.7)
    return out


def show_fit(title, img):
    """Hiện ảnh thu nhỏ vừa màn hình, chờ một phím."""
    scale = min(1.0, MAX_VIEW_W / img.shape[1])
    cv2.imshow(title, cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA))
    cv2.waitKey(0)
    cv2.destroyWindow(title)


# ===================================== CHỌN MỐC =====================================
def ask_xy(n, u, v):
    """Hỏi x y (m) của mốc thứ n; None nếu bỏ trống."""
    while True:
        text = input(f"Moc {n} (u={u:.0f}, v={v:.0f}): nhap x y (m, tu tam cau sau), Enter de bo: ").strip()
        if not text:
            return None
        parts = text.replace(";", " ").split()
        if len(parts) == 2:
            try:
                return float(parts[0]), float(parts[1])
            except ValueError:
                pass
        print("  Can 2 so cach nhau dau cach, dung dau cham thap phan. Vi du: 0.75 -0.12")


def pick(image, marks, csv_path, undistorted):
    """Vòng lặp cửa sổ chọn mốc. Sửa marks tại chỗ và ghi csv_path sau mỗi thay đổi."""
    scale = min(1.0, MAX_VIEW_W / image.shape[1])
    base = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    state = {"cursor": None, "click": None}

    def on_mouse(event, x, y, flags, param):
        p = ((x + 0.5) / scale - 0.5, (y + 0.5) / scale - 0.5)   # pixel hiển thị -> pixel ảnh gốc
        if event == cv2.EVENT_MOUSEMOVE:
            state["cursor"] = p
        elif event == cv2.EVENT_LBUTTONDOWN:
            state["click"] = p

    cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(WINDOW, on_mouse)
    print("Click vao tam tung moc roi go x y o day. z: xoa moc vua them | q/ESC: xong.")
    while True:
        view = base.copy()
        draw_marks(view, marks, scale)
        put_text(view, f"{len(marks)} moc | click: them | z: xoa moc cuoi | q/ESC: xong",
                 (10, view.shape[0] - 12), WHITE, 0.6)
        if state["cursor"] is not None:
            draw_zoom(view, image, *state["cursor"], scale)
        cv2.imshow(WINDOW, view)
        key = cv2.waitKey(30) & 0xFF

        if key in (ord("q"), 27):
            break
        if key == ord("z") and marks:
            u, v, x, y = marks.pop()
            save_marks(csv_path, marks, undistorted)
            print(f"Da xoa moc {len(marks) + 1} (u={u:.0f}, v={v:.0f}, x={x}, y={y})")
        if state["click"] is not None:
            u, v = state["click"]
            state["click"] = None
            cv2.drawMarker(view, (int(round(u * scale)), int(round(v * scale))), YELLOW, cv2.MARKER_CROSS, 20, 2)
            cv2.imshow(WINDOW, view)
            cv2.waitKey(1)
            xy = ask_xy(len(marks) + 1, u, v)
            if xy is not None:
                marks.append((u, v, xy[0], xy[1]))
                save_marks(csv_path, marks, undistorted)
        if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:   # đóng bằng nút X
            break
    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Click cac moc calib tren test.jpg roi tinh kGroundCalibration")
    parser.add_argument("image", help="anh camera luu bang phim s cua braincar (test.jpg)")
    parser.add_argument("marks", help="file CSV u, v, x, y: tao moi hoac them vao file da co")
    parser.add_argument("--lane_width", type=float, default=0.43, help="be rong lan (m, giua tam hai vach; mep trong 0.40 + vach 0.03) de ve vach tren anh kiem tra")
    parser.add_argument("--camera_calib", default=MAIN_CPP,
                        help="bo so khu meo: main.cpp (mac dinh) hoac camera_calib.yaml cua calib_camera.py")
    parser.add_argument("--no_undistort", action="store_true",
                        help="khong khu meo (chi dung khi main.cpp chua co bo so khu meo)")
    args = parser.parse_args()

    image = cv2.imread(args.image)
    if image is None:
        sys.exit(f"Khong doc duoc anh {args.image}")
    if (image.shape[1], image.shape[0]) != CALIB_SIZE:
        print(f"CANH BAO: anh {image.shape[1]}x{image.shape[0]}, kGroundCalibration can anh "
              f"{CALIB_SIZE[0]}x{CALIB_SIZE[1]} (anh camera luu bang phim s, khong phai anh bird's-eye)")

    undistorted = not args.no_undistort
    if undistorted:
        src = args.camera_calib
        lens = lens_from_yaml(src) if src.lower().endswith((".yaml", ".yml")) else lens_from_main_cpp(src)
        if lens is None:
            sys.exit(f"Khong doc duoc bo so khu meo tu {src} (hoac fx/fy = 0: chua calib ong kinh). "
                     "Calib bang calib_camera.py, hoac chay voi --no_undistort neu C++ chua khu meo.")
        K, dist = lens
        print(f"Khu meo bang bo so tu {src}: fx={K[0, 0]:.2f} fy={K[1, 1]:.2f} cx={K[0, 2]:.2f} cy={K[1, 2]:.2f} "
              f"dist={np.array2string(dist, precision=5)}")
        image = undistort(image, K, dist)
        undist_path = os.path.splitext(args.image)[0] + "_undist.jpg"
        cv2.imwrite(undist_path, image)
        print(f"Anh da khu meo: {undist_path}. Moi toa do u, v ben duoi la tren anh nay.")
    else:
        print("CANH BAO: khong khu meo. Chi dung khi cameraMatrix trong main.cpp con bang 0 (C++ khong khu meo).")

    marks = load_marks(args.marks)
    if marks:
        print(f"Da doc {len(marks)} moc tu {args.marks}")
        if marks_undistorted(args.marks) != undistorted:
            sys.exit(f"{args.marks} duoc tao {'tren anh tho' if undistorted else 'tren anh da khu meo'}, khac che do "
                     "lan nay: toa do u, v khong dung chung duoc. Dung file CSV moi.")
    pick(image, marks, args.marks, undistorted)

    print(f"\nDa luu {len(marks)} moc vao {args.marks}")
    if len(marks) < 4:
        sys.exit("Can it nhat 4 moc de tinh ma tran (nen 8-10 moc). Chay lai lenh nay de them.")
    pix = np.array([(u, v) for u, v, _, _ in marks])
    ground = np.array([(x, y) for _, _, x, y in marks])
    H = ch.report(pix, ground, image.shape[1], image.shape[0])

    check = draw_check(image, H, marks, args.lane_width)
    check_path = os.path.splitext(args.marks)[0] + "_check.jpg"
    cv2.imwrite(check_path, check)
    print(f"\nAnh kiem tra: {check_path} (nhan phim bat ky de dong)")
    show_fit("kiem tra calib", check)


if __name__ == "__main__":
    main()
