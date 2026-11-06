# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Dự án

Hệ thống cảnh báo / dự báo lệch làn (LDWS) chạy trên PC cho xe mô hình trên sa bàn. Xe (Jetson, dự án BrainUp) gửi
frame camera thô 640x480 dạng JPEG qua UDP; PC dò làn, bám vạch, dự báo thời gian tới lúc cắt vạch (TLC) và phát cảnh
báo 2 cấp. Phần dò làn được port từ `laneDetector.cpp` / `main.cpp` của BrainUp (mã C++ đó **không** có trong repo
này); comment trong code ghi tên hàm C++ gốc (`process_mask`, `make_thresholds`, `init_ipm`, `kTrackParams`...) để
đối chiếu.

Phạm vi đã chốt với nhóm:
- Toàn bộ code LDWS nằm trong **một file `main.py`**; không tách module.
- Xe chạy 0.3–0.8 m/s. Jetson gửi **vận tốc dọc** của xe (m/s) qua một cổng UDP **riêng** (7001, tách khỏi luồng ảnh
  7000). Vận tốc ngang so với làn PC tự tính.
- Chưa cần gửi cảnh báo ngược về xe; mục tiêu trước mắt là PC tính và hiển thị đúng kết quả cảnh báo.
- Homography hiện tại chưa đúng (xem mục Calib); nhóm sẽ calib lại sau.

Comment và docstring viết tiếng Việt có dấu. Chuỗi in ra console và chữ vẽ bằng `cv2.putText` cố ý viết **không
dấu** (console nào cũng đọc được; putText không vẽ được tiếng Việt), giữ nguyên quy ước đó.

## Lệnh

Python 3.11, phụ thuộc: `opencv-python`, `numpy`. Chưa có test suite, linter hay file requirements.

```bash
python main.py                                       # nhận UDP: ảnh 7000, vận tốc dọc 7001; q/ESC/Ctrl+C để thoát
python main.py --video run.mp4 --speed 0.5 --csv out.csv          # chạy lại video, vận tốc giả định không đổi
python main.py --images thu_muc_anh --fps 30 --no-show --csv out.csv   # chạy lại ảnh, không mở cửa sổ

# Calib (chi tiết trong docstring đầu mỗi file)
python calib/calib_camera.py --cols 9 --rows 6 --square 25 --calib_only   # K + hệ số méo từ ảnh bàn cờ
python calib/pick_marks.py <anh_tho.jpg> <marks.csv> --camera_calib calib/camera_calib.yaml   # click mốc -> homography
python calib/calib_homography.py calib/marks.csv --image 640x480          # tính homography từ CSV có sẵn
```

Gói vận tốc: mỗi gói UDP một số, dạng văn bản (`"0.52"`) hoặc float32/float64 little-endian (`parse_speed`). Quá
0.5 s không có gói mới thì coi như chưa biết vận tốc. Mỗi ảnh JPEG phải nằm gọn trong một gói UDP (< 65 507 byte).

## Kiến trúc `main.py`

Toàn bộ pipeline nằm trong lớp `CalibratedLDWS`. `process_raw_frame(raw, t, speed)` khử méo rồi gọi
`process_undistorted()`; `t` là thời điểm frame (s) — truyền từ ngoài vào để chạy lại video và giả lập cho kết quả
giống hệt chạy thật (debounce, Kalman, TLC đều theo `t`, không đọc đồng hồ máy).

1. `warpPerspective` sang ảnh BEV 640x480 → `process_mask()`: HSV trắng + Top-Hat kênh V, rồi lọc hình thái (bỏ mảng
   dày, nét quá mảnh, đoạn ngang dài, blob nhỏ/ngắn).
2. `detect_lines()`: bên nào đang có track thì lấy điểm trắng quanh đa thức của track (giữ danh tính vạch khi vạch
   đi qua giữa ảnh); bên còn lại dò bằng histogram + 15 cửa sổ trượt (`_window_search`) sau khi xoá điểm đã thuộc vạch
   kia. Khớp `x = a·y² + b·y + c` theo pixel BEV; `_drop_same_line` ưu tiên giữ vạch đang bám.
3. `_update_tracks()`: `line_ground()` ngoại suy vị trí ngang vạch về **đầu xe** (`x = rear_axle_to_front`) bằng
   đường thẳng qua 25 cm vạch gần xe nhất. Mỗi bên một `LineTrack` (Kalman, trạng thái `[y, ẏ]`). Vạch không thấy, hoặc
   chỉ thấy rất xa (ngoại suy > 0.45 m), được suy từ vạch bên kia ± bề rộng làn. Vạch đi qua trục giữa xe quá 2 cm thì
   đổi nhãn trái ↔ phải (xe đang cắt vạch).
4. `_evaluate()`: khoảng hở hai góc đầu xe tới mép trong vạch, TLC = khoảng hở / tốc độ tiến về vạch, LOR, góc lệch,
   phân cấp (`_side_level`), rồi `_debounce()`: mỗi cấp một bộ đếm 200 ms (`level_since`), xuống cấp sau 300 ms, tắt
   ngay khi lái sửa; hướng cảnh báo chốt lúc bắt đầu tới khi tắt. Hysteresis: nới ngưỡng cấp 1 khi cấp 1 đang đếm hoặc
   đang bật, nới ngưỡng cấp 2 **chỉ khi** cấp 2 đã bật (nới sớm hơn thì một đỉnh nhiễu tự kéo dài thành cảnh báo khẩn
   cấp giả). `KF_ACCEL_STD`, `V_LAT_MIN` và các hệ số hysteresis được chọn bằng giả lập nhiều chuỗi nhiễu; đổi chúng
   thì chạy lại các kịch bản trôi chậm 2–3 cm/s, vì ở tốc độ đó nhiễu vận tốc ngang (±1 cm/s) gần bằng tín hiệu.
5. `_draw_visualizations()`: vạch đang bám trên BEV (nét mảnh = chỉ dự đoán/suy ra), vùng làn chiếu ngược về ảnh
   camera, HUD.

Ngoài lớp chính: `SpeedReceiver` (UDP không chặn), `ResultLog` (CSV, ghi từng dòng), `run_udp_ldws_receiver`,
`run_replay`.

Các ngưỡng hình học (`th_*`) đều suy ra từ số đo sa bàn trong `TrackAndVehicleParams` qua `_make_thresholds()`.
Muốn đổi kích thước làn/vạch/xe thì sửa `TrackAndVehicleParams`, không sửa từng ngưỡng.

### Hệ toạ độ và dấu (dễ nhầm)

- **Mặt đường / hệ xe (m)**: gốc tại tâm cầu sau, `x` hướng tới trước, `y` sang **trái**. Homography `H` đổi pixel ảnh
  **đã khử méo** → mặt đường.
- **BEV**: phủ vùng `x ∈ [0.45, 1.05] m`, `y ∈ [-0.5, 0.5] m`; hàng dưới cùng = `x = 0.45 m`, hàng trên = 1.05 m; cột
  320 = trục giữa xe; trái BEV = trái xe. BEV là ảnh co giãn đều theo mét (640 px/m ngang, 800 px/m dọc — **không**
  vuông), nên `x_to_row()` / `col_to_y()` là phép đổi tuyến tính. Hệ số góc BEV cùng dấu với `dy/dx` mặt đường.
- Camera chỉ thấy vạch từ khoảng `x ≈ 0.45–0.85 m` trở ra, nên mọi giá trị tại đầu xe (0.36 m) là ngoại suy.
- `LineTrack.y` > 0 là vạch ở bên trái xe. Xe trôi sang trái thì `ẏ < 0` với mọi vạch.
- `offset_cm` dương = xe lệch phải so với tâm làn. `v_lat_cm_s` dương = xe trôi sang trái. `angle_deg` dương = đầu xe
  chĩa sang trái so với làn. `clr_left_cm` / `clr_right_cm` âm = góc đầu xe đã đè lên vạch.

### Calib

- `fx ≠ fy` (≈524 vs ≈703) là đúng: pipeline GStreamer co 1920x1080 → 640x480 không đều hai chiều. K và hệ số méo
  chỉ đúng với độ phân giải và pipeline đó.
- `calib/test.jpg`, `calib/marks.csv` là dữ liệu cũ ở **960x540**. `homography` trong `main.py` chính là H khớp từ
  `marks.csv` rồi co về 640x480, trong khi K đã calib lại ở 640x480. Hệ quả đo được trên ảnh thật: hai vạch trên BEV
  không song song (bề rộng làn 0.50 m ở 0.6 m → 0.59 m ở 1.0 m, thông số là 0.43 m), nên **góc lệch chưa tin được**.
  Vì vậy góc lệch hiện chỉ để hiển thị; `FUSE_SPEED_HEADING` (đo ẏ = v·độ dốc vạch từ vận tốc dọc) để `False` cho tới
  khi calib lại.
- Các script calib vẫn mang dấu vết BrainUp: in ra literal C++ để dán vào `kGroundCalibration` trong `main.cpp`, và
  `pick_marks.py` mặc định đọc K/dist từ `../main.cpp` (không tồn tại ở đây) — luôn truyền `--camera_calib`. Ở repo
  này, kết quả calib phải chép tay vào `GroundCalibration` trong `main.py`.

### Kiểm tra thuật toán không cần xe

Tạo frame giả lập xe dịch ngang `dy` và xoay `ψ` từ một ảnh thật đã khử méo:
`cv2.warpPerspective(und, inv(H) @ A @ H, (640, 480), flags=cv2.WARP_INVERSE_MAP)` với
`A = [[cosψ, -sinψ, 0], [sinψ, cosψ, dy], [0, 0, 1]]`. Vạch làn nằm trên mặt đường nên được biến đổi đúng, và giá trị
đúng tính từ vạch của ảnh gốc qua cùng phép biến đổi. Gọi `process_undistorted(frame, t, speed)` với `t` tăng đều
để có chuỗi theo thời gian. Phép thử này kiểm tra thuật toán, không phát hiện được sai số của chính bộ calib.

### Lưu ý khi sửa code

- Trường `np.ndarray` trong `@dataclass` phải dùng `field(default_factory=...)`; Python ≥ 3.11 báo `ValueError:
  mutable default` ngay lúc import nếu gán mảng trực tiếp.
- `self.visible_mask` (vùng camera thấy được trên BEV) được tính trong `__init__` nhưng hiện chưa được dùng ở đâu.

## Ba bài báo tham chiếu ("Tài liệu 1/2/3" trong comment)

| Comment | Bài báo (file PDF ở thư mục gốc) | Phần được dùng |
|---|---|---|
| Tài liệu 1 | Anbalagan et al., *Vision-Based Ingenious LDWS for AVs*, Sustainability 2023 (`sustainability-15-03535-v2.pdf`) | Góc và hướng lệch của xe so với vạch |
| Tài liệu 2 | Sukumar & Sumathi, *An Improved Lane Detection and LDW Framework for ADAS*, IEEE TCE 2024 | LOR, ngưỡng `TH` = 80% nửa làn, cảnh báo khi LOR > 0.6; hai vạch cùng tiến vào giữa = làn hẹp lại, không phải lệch |
| Tài liệu 3 | Yichen Zhong, *Design and Simulation of LDW Assistant Driving System*, JTIE 2026 | Cảnh báo 2 cấp theo TLC (1.0–1.8 s: cấp 1, < 1.0 s: cấp 2), debounce 200 ms, nhả 300 ms, tắt ngay khi lái sửa, Kalman bám vạch, các KPI (< 50 ms/frame, báo nhầm < 10%, bỏ sót < 5%) |

Khác biệt cần biết: LOR trong code là `(nửa_làn − khoảng_cách_gần_nhất) / (0.8·nửa_làn)` tính tại đầu xe (= 0 ở
giữa làn, 1.25 khi tâm xe chạm tâm vạch), không chép nguyên công thức (11) của Tài liệu 2. Tài liệu 1 không đưa ra
ngưỡng góc cố định; code hiện không dùng góc để phân cấp cảnh báo.
