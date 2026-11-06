# thiatgt — Cảnh báo lệch làn cho xe mô hình

Hệ thống cảnh báo và dự báo lệch làn (Lane Departure Warning System, LDWS) dùng camera đơn, chạy trên PC cho xe mô
hình trên sa bàn. Phần dò làn được kế thừa từ bộ lane detector của dự án BrainUp (C++) và viết lại bằng Python, phần
bám vạch, dự báo và cảnh báo xây dựng theo ba bài báo bên dưới.

## Cách hoạt động

```
Xe (Jetson) ──ảnh JPEG 640x480, UDP :7000──▶ PC: main.py
            ──vận tốc dọc (m/s), UDP :7001──▶
                                                     │
   khử méo ống kính ─▶ chiếu Bird's-Eye View (IPM) ─▶ mask vạch trắng (HSV + Top-Hat + lọc hình thái)
   ─▶ tìm vạch (quanh vạch frame trước, hoặc cửa sổ trượt) + khớp đa thức
   ─▶ bám từng vạch bằng bộ lọc Kalman: vị trí ngang tại đầu xe và tốc độ thay đổi
   ─▶ khoảng hở góc đầu xe – vạch, thời gian tới lúc cắt vạch (TLC), LOR, góc lệch
   ─▶ cảnh báo 2 cấp (vàng: sớm, đỏ: khẩn cấp), debounce 200 ms / nhả 300 ms ─▶ hiển thị, ghi CSV
```

Mọi phép đo đều tính theo mét trên mặt đường nhờ calib camera (khử méo) và homography mặt đường. Gốc toạ độ là tâm
cầu sau của xe, `x` hướng tới trước, `y` sang trái.

**Điều kiện cảnh báo** (xét riêng từng bên, khoảng hở đo tại góc đầu xe tới mép trong vạch):

| Mức | Điều kiện |
|---|---|
| Cấp 2 — khẩn cấp (đỏ) | Góc đầu xe đã chạm/đè vạch, hoặc TLC < 1.0 s, hoặc LOR > 0.60 |
| Cấp 1 — sớm (vàng) | TLC < 1.8 s, hoặc khoảng hở < 2.5 cm, hoặc LOR > 0.45 |
| Không báo | Xe gần như đứng yên (vận tốc dọc < 0.1 m/s), hoặc hai vạch cùng tiến vào giữa (làn hẹp lại) |

Mỗi cấp chỉ bật khi điều kiện của chính cấp đó kéo dài ≥ 200 ms. Cảnh báo tắt sau khi an toàn ≥ 300 ms, hoặc tắt
ngay khi xe đã rời khỏi vạch và đang đi ra xa vạch. Để nhiễu quanh ngưỡng không làm cảnh báo chập chờn (hysteresis),
ngưỡng được nới ra (TLC × 1.25, khoảng hở + 0.5 cm, LOR − 0.05) trong hai trường hợp: ngưỡng cấp 1 khi cấp 1 đang đếm
hoặc đang bật, ngưỡng cấp 2 khi cấp 2 đã bật.

Thông số sa bàn và xe đang dùng (`TrackAndVehicleParams` trong `main.py`):

| Thông số | Giá trị |
|---|---|
| Khoảng trống giữa mép trong hai vạch | 0.40 m |
| Bề rộng vạch | 0.03 m |
| Nét đứt / khe hở | 0.13 m / 0.20 m |
| Bề rộng xe / chiều dài cơ sở / cầu sau → đầu xe | 0.21 m / 0.258 m / 0.36 m |
| Tốc độ xe | 0.3–0.8 m/s |

## Cơ sở lý thuyết

| Trong code | Bài báo | Ý tưởng sử dụng |
|---|---|---|
| Tài liệu 1 | S. Anbalagan et al., "Vision-Based Ingenious Lane Departure Warning System for Autonomous Vehicles", *Sustainability*, 2023. DOI: [10.3390/su15043535](https://doi.org/10.3390/su15043535) | Góc và hướng lệch của xe so với vạch làn |
| Tài liệu 2 | N. Sukumar, P. Sumathi, "An Improved Lane Detection and Lane Departure Warning Framework for ADAS", *IEEE Trans. Consumer Electronics*, 2024. DOI: [10.1109/TCE.2024.3387708](https://doi.org/10.1109/TCE.2024.3387708) | Tỉ số lệch ngang LOR, ngưỡng cảnh báo 80% nửa làn, LOR > 0.6, phân biệt làn hẹp lại với xe lệch |
| Tài liệu 3 | Yichen Zhong, "Design and Simulation of Lane Departure Warning Assistant Driving System", *JTIE*, 2026. DOI: [10.63887/jtie.2026.2.4.29](https://doi.org/10.63887/jtie.2026.2.4.29) | Cảnh báo 2 cấp theo TLC, debounce/nhả cảnh báo, tắt khi lái sửa, bám vạch bằng Kalman, các chỉ tiêu đánh giá |

## Cài đặt

Python 3.11 trở lên.

```bash
pip install opencv-python numpy
```

## Chạy

**Với xe:**

```bash
python main.py                    # thêm --csv ketqua.csv để ghi kết quả từng frame
```

PC lắng nghe hai cổng UDP trên mọi giao diện mạng:

| Cổng | Nội dung mỗi gói |
|---|---|
| 7000 | Một ảnh JPEG 640x480. Phải nhỏ hơn 65 507 byte (giới hạn một gói UDP); ảnh khác cỡ sẽ bị co về 640x480 |
| 7001 | Vận tốc dọc của xe (m/s): văn bản như `0.52`, hoặc số float32/float64 little-endian |

Nếu quá 0.5 s không có gói vận tốc mới, hệ thống vẫn chạy nhưng không áp dụng điều kiện "xe đứng yên". Đổi cổng bằng
`--port`, `--speed-port`.

**Chạy lại dữ liệu đã ghi** (không cần xe):

```bash
python main.py --video run.mp4 --speed 0.5 --csv ketqua.csv
python main.py --images thu_muc_anh --fps 30 --no-show --csv ketqua.csv
```

Thời gian của từng frame tính theo fps nên debounce, Kalman và TLC chạy như thật. `--speed` là vận tốc dọc giả định
không đổi. Khi chạy xong, chương trình in số frame ở từng mức cảnh báo và thời gian xử lý.

**Cửa sổ hiển thị:** ảnh camera kèm vùng làn tô màu theo mức cảnh báo, ảnh Bird's-Eye View với vạch đang bám (nét mảnh
là vạch chỉ dự đoán hoặc suy ra), và mask nhị phân. Bảng thông số gồm trạng thái và lý do, độ lệch tâm, LOR, khoảng hở
trái/phải, TLC, vận tốc ngang, góc lệch, vận tốc dọc, thời gian xử lý. `Space` để tạm dừng, `q` hoặc `ESC` để thoát.

## Calib

Số calib nằm trong `GroundCalibration` của `main.py` và chỉ đúng với camera, độ phân giải (640x480) và pipeline
GStreamer hiện tại. Khi gắn lại camera hoặc đổi độ phân giải thì calib lại theo thứ tự:

1. **Ống kính** — chụp bàn cờ, tính ma trận camera và hệ số méo:
   ```bash
   python calib/calib_camera.py --cols 9 --rows 6 --square 25            # chụp trên Jetson rồi tính
   python calib/calib_camera.py --cols 9 --rows 6 --square 25 --calib_only   # chỉ tính từ ảnh đã chụp
   ```
   Kết quả ghi vào `calib/camera_calib.yaml` và ảnh kiểm tra `calib/undistort_check.jpg`.
2. **Mặt đường** — đặt mốc trên sàn, đo toạ độ (m) từ tâm cầu sau, chụp một ảnh 640x480 rồi click các mốc:
   ```bash
   python calib/pick_marks.py anh.jpg marks.csv --camera_calib calib/camera_calib.yaml
   ```
   Nên dùng 8–10 mốc rải cả gần–xa lẫn trái–phải. Script in sai số từng mốc và lưu ảnh lưới kiểm tra
   `marks_check.jpg`: lưới phải nằm phẳng trên sàn và trùng vạch thật.
3. Chép ma trận camera, hệ số méo và homography vào `GroundCalibration` trong `main.py`.
4. Kiểm tra: đặt xe thẳng giữa một đoạn làn thẳng. Trên ảnh BEV, hai vạch phải song song, bề rộng làn khoảng 43 cm
   và góc lệch gần 0°.

Các script calib được mang từ BrainUp nên vẫn in kết quả dạng C++ (`cv::Matx33d`, cho `main.cpp`); ở dự án này chỉ
cần lấy các con số.

## Cấu trúc thư mục

```
main.py      # toàn bộ LDWS: dò làn, bám vạch, TLC, cảnh báo, nhận UDP, chạy lại video/ảnh
calib/       # script calib ống kính và homography mặt đường, ảnh và mốc calib mẫu
ket_qua/     # hình kết quả: ảnh thật, các bước cảnh báo khi xe trôi ra khỏi làn, biểu đồ TLC
CLAUDE.md    # ghi chú kỹ thuật cho Claude Code
```

## Trạng thái

Đã kiểm tra trên ảnh thật của sa bàn, có giả lập xe trôi ngang và xoay với giá trị đúng biết trước (4 chuỗi nhiễu
đo, trôi 2/3/5 cm/s). Hình kết quả nằm trong `ket_qua/`.
- Khoảng hở ở bên vạch xe đang tiến tới sai dưới 1–2 cm.
- Cảnh báo cấp 1 bật lúc còn 1.4–1.9 s nữa mới chạm vạch, cấp 2 lúc còn 0.4–0.8 s.
- Đứng yên giữa làn hoặc đứng sát vạch 3.6 cm: không báo nhầm.
- Cảnh báo không tắt giữa chừng, không nhảy thẳng lên cấp 2.
- Lái sửa: cảnh báo tắt ~0.1 s sau khi xe đã an toàn.
- Cắt hẳn qua vạch vẫn giữ đúng hướng cảnh báo.
- Xử lý ~12 ms/frame.

- [x] Sửa lỗi `main.py` không import được trên Python 3.11.
- [x] Nhận vận tốc dọc của xe từ Jetson qua cổng UDP riêng (7001).
- [x] Đo khoảng hở tại hai góc đầu xe thay vì tại đáy ảnh BEV.
- [x] Bám vạch theo thời gian bằng bộ lọc Kalman, giữ đúng vạch trái/phải khi xe cắt vạch (Tài liệu 3).
- [x] Dự báo TLC và cảnh báo 2 cấp theo TLC (Tài liệu 3).
- [x] Chạy lại video/ảnh, ghi kết quả từng frame ra CSV.
- [ ] Calib lại homography: trên ảnh BEV hai vạch chưa song song, bề rộng làn đo được 0.50–0.60 m thay vì 0.43 m.
      Trước khi calib lại, góc lệch chỉ để tham khảo.
- [ ] Sau khi calib: bật `FUSE_SPEED_HEADING` để dùng thêm vận tốc dọc × góc lệch, giúp TLC phản ứng nhanh hơn.
- [ ] Ghi video các lượt chạy thật, gán nhãn và đo tỉ lệ báo nhầm, bỏ sót, độ trễ.
