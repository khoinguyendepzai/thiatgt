"""Tính homography calib mặt đường (kGroundCalibration trong main.cpp) từ các mốc đo trên sàn.

Homography đổi pixel trên ảnh camera (test.jpg, phím s của braincar, 640x480) thành tọa độ mặt đường tính
bằng mét, gốc ở giữa cầu sau, x hướng tới trước, y sang trái (pure_pursit.md mục 7.2 và 7.8).

File mốc (CSV), mỗi dòng một mốc, dòng trống và dòng bắt đầu bằng # bị bỏ qua:
    # u, v (pixel trên test.jpg), x, y (m, đo từ giữa cầu sau)
    812, 905, 0.40, 0.10

Cách dùng:
    python calib/calib_homography.py marks.csv [--image 640x480]

Kết quả: sai số từng mốc (khớp và bỏ-ra), cảnh báo chiều trục, rồi dòng cv::Matx33d để dán vào main.cpp.
Không muốn tra pixel tay: pick_marks.py cho click các mốc trên test.jpg, ghi file CSV này rồi gọi report().
Chỉ cần numpy. Thông báo in ra không dấu để console nào cũng đọc được.
"""

import argparse
import sys

import numpy as np

WARN_CM = 2.0   # sai số một mốc lớn hơn (cm) thì cảnh báo: đo lại mốc đó


def read_marks(path):
    """Trả về (pixel Nx2, mặt đường Nx2)."""
    pix, ground = [], []
    with open(path, encoding="utf-8") as f:
        for lineNo, line in enumerate(f, 1):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            fields = [s for s in line.replace(";", ",").split(",") if s.strip()]
            if len(fields) != 4:
                sys.exit(f"Loi dong {lineNo}: can 4 so u, v, x, y, co {len(fields)}: {line}")
            try:
                u, v, x, y = (float(s) for s in fields)
            except ValueError:
                sys.exit(f"Loi dong {lineNo}: khong phai so: {line}")
            pix.append((u, v))
            ground.append((x, y))
    return np.array(pix, dtype=float), np.array(ground, dtype=float)


def normalizer(pts):
    """Ma trận dời trọng tâm về gốc và co giãn để khoảng cách trung bình tới gốc là sqrt(2) (Hartley)."""
    c = pts.mean(axis=0)
    d = np.sqrt(((pts - c) ** 2).sum(axis=1)).mean()
    s = np.sqrt(2.0) / d if d > 0 else 1.0
    return np.array([[s, 0.0, -s * c[0]], [0.0, s, -s * c[1]], [0.0, 0.0, 1.0]])


def transform(H, pts):
    """Áp homography lên các điểm Nx2."""
    p = np.hstack([pts, np.ones((len(pts), 1))]) @ H.T
    return p[:, :2] / p[:, 2:3]


def dlt(src, dst):
    """Nghiệm bình phương tối thiểu đại số của dst ~ H·src (src, dst đã chuẩn hoá)."""
    rows = []
    for (u, v), (x, y) in zip(src, dst):
        rows.append([-u, -v, -1.0, 0.0, 0.0, 0.0, x * u, x * v, x])
        rows.append([0.0, 0.0, 0.0, -u, -v, -1.0, y * u, y * v, y])
    _, _, vt = np.linalg.svd(np.array(rows))
    H = vt[-1].reshape(3, 3)
    return H / H[2, 2]


def refine(H, src, dst, iterations=30):
    """Gauss-Newton trên 8 hệ số (H[2,2] = 1), giảm sai số trên mặt đường như cv::findHomography."""
    params = H.ravel()[:8].copy()

    def residual(p):
        return (transform(np.append(p, 1.0).reshape(3, 3), src) - dst).ravel()

    r = residual(params)
    for _ in range(iterations):
        J = np.empty((len(r), 8))
        for k in range(8):
            step = 1e-7 * max(1.0, abs(params[k]))
            p = params.copy()
            p[k] += step
            J[:, k] = (residual(p) - r) / step
        delta = np.linalg.lstsq(J, -r, rcond=None)[0]
        trial = params + delta
        rTrial = residual(trial)
        if rTrial @ rTrial >= r @ r:
            break
        params, r = trial, rTrial
        if np.abs(delta).max() < 1e-12:
            break
    return np.append(params, 1.0).reshape(3, 3)


def fit(pix, ground):
    """Homography pixel -> mặt đường (m), H[2,2] = 1."""
    Tp, Tg = normalizer(pix), normalizer(ground)
    src, dst = transform(Tp, pix), transform(Tg, ground)   # tọa độ chuẩn hoá: các hệ số cùng cỡ
    Hn = refine(dlt(src, dst), src, dst)
    H = np.linalg.inv(Tg) @ Hn @ Tp
    return H / H[2, 2]


def errors_cm(H, pix, ground):
    return np.sqrt(((transform(H, pix) - ground) ** 2).sum(axis=1)) * 100.0


def leave_one_out_cm(pix, ground):
    """Sai số khi khớp bằng các mốc còn lại rồi đoán mốc bị bỏ ra. Mốc đo nhầm lộ rõ ở đây."""
    out = []
    for i in range(len(pix)):
        keep = np.arange(len(pix)) != i
        H = fit(pix[keep], ground[keep])
        out.append(errors_cm(H, pix[i:i + 1], ground[i:i + 1])[0])
    return np.array(out)


def axis_warnings(H, pix):
    """Kiểm tra chiều trục theo H tại giữa các mốc (camera nhìn thẳng về trước): đi lên trên ảnh (v giảm) thì x tăng,
    sang trái ảnh (u giảm) thì y tăng, và mỗi trục đổi chủ yếu theo một chiều của ảnh. Sai thường do đảo cột x/y
    hoặc sai dấu y. Không dùng tương quan giữa các mốc: mốc rải đối xứng thì đảo x/y vẫn cho tương quan 0."""
    c = pix.mean(axis=0)
    g0 = transform(H, c[None])[0]
    du = transform(H, (c + [1.0, 0.0])[None])[0] - g0   # sang phải 1 px
    dv = transform(H, (c + [0.0, 1.0])[None])[0] - g0   # xuống 1 px
    warnings = []
    if not (dv[0] < 0 and abs(dv[0]) > abs(du[0])):
        warnings.append("CANH BAO chieu truc: moc xa hon (x lon) phai nam cao hon tren anh. Kiem tra cot x/y co bi dao.")
    if not (du[1] < 0 and abs(du[1]) > abs(dv[1])):
        warnings.append("CANH BAO chieu truc: moc ben trai (y > 0) phai nam ben trai anh. Kiem tra dau cua y, "
                        "hoac cot x/y bi dao.")
    return warnings


def cpp_literal(H):
    rows = [", ".join(f"{H[i, j]: .9g}" for j in range(3)) for i in range(3)]
    return ("    cv::Matx33d(" + rows[0] + ",\n"
            "                " + rows[1] + ",\n"
            "                " + rows[2] + "),")


def report(pix, ground, width, height):
    """Khớp H, in sai số từng mốc, cảnh báo và dòng cv::Matx33d; trả về H. Thoát nếu mốc không dùng được.
    pick_marks.py cũng gọi hàm này."""
    n = len(pix)
    if n < 4:
        sys.exit(f"Can it nhat 4 moc, co {n}")
    centered = ground - ground.mean(axis=0)
    sv = np.linalg.svd(centered, compute_uv=False)
    if sv[1] < 1e-3 * sv[0]:
        sys.exit("Cac moc gan nhu thang hang tren san: rai moc ca gan-xa lan trai-phai")

    outside = [i + 1 for i, (u, v) in enumerate(pix) if not (0 <= u < width and 0 <= v < height)]
    H = fit(pix, ground)
    fitErr = errors_cm(H, pix, ground)
    looErr = leave_one_out_cm(pix, ground) if n >= 5 else None

    print(f"{n} moc, anh {width}x{height}")
    print(" moc       u       v    x (m)    y (m)   sai so khop (cm)   sai so bo-ra (cm)")
    for i in range(n):
        loo = f"{looErr[i]:10.2f}" if looErr is not None else "         -"
        flag = "  <-- do lai" if (looErr is not None and looErr[i] > WARN_CM) or fitErr[i] > WARN_CM else ""
        print(f"{i + 1:4d} {pix[i, 0]:7.1f} {pix[i, 1]:7.1f} {ground[i, 0]:8.3f} {ground[i, 1]:8.3f}"
              f" {fitErr[i]:14.2f} {loo:>19}{flag}")
    print(f"Sai so khop: trung binh {fitErr.mean():.2f} cm, lon nhat {fitErr.max():.2f} cm "
          f"(moc {fitErr.argmax() + 1})")
    if looErr is not None:
        print(f"Sai so bo-ra: trung binh {looErr.mean():.2f} cm, lon nhat {looErr.max():.2f} cm "
              f"(moc {looErr.argmax() + 1})")
    else:
        print("Chi co 4 moc: khop dung tuyet doi nen khong kiem tra duoc sai so. Nen dung 8-10 moc.")

    for w in axis_warnings(H, pix):
        print(w)
    if outside:
        print(f"CANH BAO: moc {outside} nam ngoai anh {width}x{height}")

    print("\nDan vao kGroundCalibration trong main.cpp (thay ma tran toan 0):")
    print(cpp_literal(H))
    return H


def main():
    parser = argparse.ArgumentParser(description="Tinh homography calib mat duong cho kGroundCalibration")
    parser.add_argument("marks", help="file CSV: u, v, x, y moi dong")
    parser.add_argument("--image", default="640x480", help="kich thuoc test.jpg, mac dinh 640x480")
    args = parser.parse_args()

    width, height = (int(s) for s in args.image.lower().split("x"))
    pix, ground = read_marks(args.marks)
    report(pix, ground, width, height)


if __name__ == "__main__":
    main()
