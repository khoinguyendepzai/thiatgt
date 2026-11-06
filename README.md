# thiatgt — Cảnh báo lệch làn cho xe mô hình

Hệ thống cảnh báo và dự báo lệch làn (Lane Departure Warning System, LDWS) dùng camera đơn, chạy trên PC cho xe mô
hình trên sa bàn. Phần dò làn được kế thừa từ bộ lane detector của dự án BrainUp (C++) và viết lại bằng Python, phần
bám vạch, dự báo và cảnh báo xây dựng theo ba bài báo bên dưới.

## Cách hoạt động

```
Xe (Jetson) ──ảnh JPEG 640x480, UDP :7000──▶ PC: main.py
                                                     │
   khử méo ống kính ─▶ chiếu Bird's-Eye View (IPM) ─▶ mask vạch trắng (HSV + Top-Hat + lọc hình thái)
   ─▶ tìm vạch (quanh vạch frame trước, hoặc cửa sổ trượt) + khớp đa thức
   ─▶ bám từng vạch bằng bộ lọc trượt: vị trí ngang tại đầu xe
   ─▶ khoảng hở góc đầu xe – vạch, LOR
   ─▶ cảnh báo 2 mức (vàng: lệch làn, đỏ: chạm vạch), debounce 200 ms / nhả 300 ms ─▶ hiển thị, ghi CSV
```

Hệ thống chỉ xét **vị trí** của xe so với vạch. Cố ý không dự báo và không dùng vận tốc: không có TLC, vận tốc ngang
hay gia tốc ngang. Mọi phép đo tính theo mét trên mặt đường nhờ calib camera (khử méo) và homography mặt đường. Gốc
toạ độ là tâm cầu sau của xe, `x` hướng tới trước, `y` sang trái.

**Điều kiện cảnh báo** (xét riêng từng bên, khoảng hở đo tại góc đầu xe tới mép trong vạch):

| Mức | Điều kiện |
|---|---|
| Cấp 2 — chạm vạch (đỏ) | Khoảng hở ≤ 0: góc đầu xe đã chạm hoặc đè vạch |
| Cấp 1 — lệch làn (vàng) | LOR > 0.45 (tương đương góc đầu xe cách vạch ~1.8 cm) |

Mỗi cấp chỉ bật khi điều kiện của chính cấp đó kéo dài ≥ 200 ms, và tắt sau khi an toàn ≥ 300 ms. Để nhiễu quanh
ngưỡng không làm cảnh báo chập chờn (hysteresis), ngưỡng được nới ra (LOR − 0.05, khoảng hở + 0.5 cm): ngưỡng cấp 1
khi cấp 1 đang đếm hoặc đang bật, ngưỡng cấp 2 khi cấp 2 đã bật. Hướng cảnh báo (trái/phải) được chốt lúc bắt đầu và
giữ tới khi tắt, kể cả khi xe cắt hẳn qua vạch.

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
| Tài liệu 2 | N. Sukumar, P. Sumathi, "An Improved Lane Detection and Lane Departure Warning Framework for ADAS", *IEEE Trans. Consumer Electronics*, 2024. DOI: [10.1109/TCE.2024.3387708](https://doi.org/10.1109/TCE.2024.3387708) | Tỉ số lệch ngang LOR và ngưỡng 80% nửa làn: LOR vượt ngưỡng là lệch làn |
| Tài liệu 3 | Yichen Zhong, "Design and Simulation of Lane Departure Warning Assistant Driving System", *JTIE*, 2026. DOI: [10.63887/jtie.2026.2.4.29](https://doi.org/10.63887/jtie.2026.2.4.29) | Debounce 200 ms, nhả 300 ms, hysteresis, bám vạch qua các frame, các chỉ tiêu đánh giá. Phần cảnh báo theo TLC của bài không dùng |

**Báo cáo chi tiết:** [BaoCao_LDWS.pdf](BaoCao_LDWS.pdf) (17 trang) trình bày đầy đủ cơ sở lý thuyết áp dụng (camera và méo ống kính,
homography và IPM, Top-Hat, khớp đa thức, lọc trượt, LOR, debounce/hysteresis), định nghĩa từng đại lượng, bộ tiêu chí đánh giá và kết quả đo.
File PDF của ba bài báo không đưa vào repo; tra theo DOI ở bảng trên.

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

PC lắng nghe UDP cổng 7000 trên mọi giao diện mạng (đổi bằng `--port`). Mỗi gói là một ảnh JPEG 640x480, phải nhỏ hơn
65 507 byte (giới hạn một gói UDP): nén chất lượng ~80 là khoảng 50 KB. Ảnh khác cỡ sẽ bị co về 640x480.

**Chạy lại dữ liệu đã ghi** (không cần xe):

```bash
python main.py --video run.mp4 --csv ketqua.csv
python main.py --images thu_muc_anh --fps 30 --no-show --csv ketqua.csv
```

Thời gian của từng frame tính theo fps nên debounce và bộ lọc trượt chạy như thật. Khi chạy xong, chương trình in số
frame ở từng mức cảnh báo và thời gian xử lý.

**Cửa sổ hiển thị:** ảnh camera kèm vùng làn tô màu theo mức cảnh báo, ảnh Bird's-Eye View với vạch đang bám (nét mảnh
là vạch chỉ dự đoán hoặc suy ra), và mask nhị phân. Bảng thông số gồm trạng thái, độ lệch tâm, LOR, khoảng hở trái/phải, góc lệch, thời gian xử lý. `Space` để tạm dừng, `q` hoặc `ESC` để thoát.

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
main.py      # toàn bộ LDWS: dò làn, bám vạch, cảnh báo lệch làn / chạm vạch, nhận UDP, chạy lại video/ảnh
calib/       # script calib ống kính và homography mặt đường, ảnh và mốc calib mẫu
BaoCao_LDWS.pdf  # báo cáo kỹ thuật dạng bài báo đánh giá
ket_qua/     # hình kết quả: ảnh thật, các mức cảnh báo khi xe lệch dần khỏi làn, khoảng hở theo thời gian
CLAUDE.md    # ghi chú kỹ thuật cho Claude Code
```

## Trạng thái

Đã kiểm tra trên ảnh thật của sa bàn, có giả lập xe dịch ngang và xoay với giá trị đúng biết trước. Hình kết quả nằm
trong `ket_qua/`.
- Khoảng hở xe – vạch sai dưới ~2 cm (trung bình dưới 1 cm).
- Lệch làn (vàng) bật khi góc đầu xe còn cách vạch khoảng 1–2 cm, chạm vạch (đỏ) bật ngay sau khi chạm vạch.
  Mỗi cấp trễ ~0.2 s do debounce và ~0.1 s do bộ lọc trượt.
- Xe đứng giữa làn hoặc đứng cách vạch 3.6 cm: không báo nhầm.
- Cắt hẳn qua vạch vẫn giữ đúng hướng cảnh báo; quay lại làn thì cảnh báo tắt sau khi an toàn ≥ 300 ms.
- Xử lý ~12 ms/frame.

- [x] Sửa lỗi `main.py` không import được trên Python 3.11.
- [x] Đo khoảng hở tại hai góc đầu xe thay vì tại đáy ảnh BEV.
- [x] Bám vạch qua các frame bằng bộ lọc trượt, giữ đúng vạch trái/phải khi xe cắt vạch.
- [x] Cảnh báo hai mức theo vị trí: lệch làn và chạm vạch.
- [x] Chạy lại video/ảnh, ghi kết quả từng frame ra CSV.
- [ ] Calib lại homography: trên ảnh BEV hai vạch chưa song song, bề rộng làn đo được 0.50–0.60 m thay vì 0.43 m.
      Trước khi calib lại, góc lệch chỉ để tham khảo.
- [ ] Ghi video các lượt chạy thật, gán nhãn và đo tỉ lệ báo nhầm, bỏ sót, độ trễ.
