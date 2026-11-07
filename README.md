# thiatgt — Cảnh báo lệch làn cho xe mô hình 1:10

Hệ thống cảnh báo lệch làn (Lane Departure Warning System, LDWS) dùng camera đơn, chạy trên PC cho xe mô hình 1:10
trên sa bàn tự vẽ. Xe (Jetson) gửi ảnh; PC tính vị trí xe trong làn và phát cảnh báo hai mức: **lệch làn** (vàng)
và **chạm vạch** (đỏ).

- **Phạm vi cố định:** xe 1:10 rộng 0.21 m; làn 0.43 m giữa tâm hai vạch; vạch rộng 3 cm.
- **Một đại lượng quyết định:** cả hai mức cảnh báo cùng so sánh **LOR** (tỉ số lệch ngang), không dùng khoảng hở
  (clearance). Lý do: LOR là tỉ số của các độ dài nên **không thứ nguyên**. Mọi sai số **tỉ lệ** của camera và
  homography (co giãn ngang, kể cả co giãn tăng theo khoảng cách như homography hiện tại) bị triệt tiêu, nên **không
  cần xét tới**. Khoảng hở tính bằng cm nên sai đúng phần tỉ lệ đó. Chứng minh và số đo ở [mục 5](#5-vì-sao-dùng-lor-thay-cho-khoảng-hở).
  LOR không bù được sai số dịch chuyển, ví dụ camera lắp lệch ngang.
- **Chỉ xét vị trí:** không dùng vận tốc ở bất kỳ đâu, kể cả khi đặt ngưỡng. Không dự báo, không có TLC.
- **ISO 17361 là thước đo đánh giá:** tiêu chuẩn được thu nhỏ 1:10 theo tỉ lệ xe để đánh giá hệ thống và để chứng
  minh vì sao LOR tốt hơn khoảng hở (clearance). Tiêu chuẩn không dùng để quyết định cảnh báo.

Báo cáo đầy đủ (lý thuyết, chứng minh, tiêu chí, kết quả): [BaoCao_LDWS.pdf](BaoCao_LDWS.pdf) (24 trang).

## 1. Xử lý ảnh (tóm tắt)

```
ảnh JPEG 640x480 (UDP :7000) → khử méo → Bird's-Eye View (homography mặt đường)
  → mask vạch trắng (HSV + Top-Hat + lọc hình thái) → dò vạch (cửa sổ trượt / quanh vạch frame trước)
  → khớp đa thức → ngoại suy vị trí vạch về đầu xe (x = 0.36 m) → bộ lọc trượt 0.15 s
  → y_L, y_R: vị trí tâm vạch trái / phải tại đầu xe (m, hệ toạ độ xe)
```

Gốc toạ độ ở tâm cầu sau, `x` hướng tới trước, `y` sang trái. Chi tiết có trong báo cáo, mục 4.1–4.5.

Code chia ba phần, ranh giới giữa chúng là thông số hình học `LaneGeometry`:

```
main.py          thông số đầu vào (calib, sa bàn và xe, ISO) → nhận frame (UDP :7000 / video / ảnh) → ghi CSV
   │ frame
   ▼
lane_vision.py   phần 1: xử lý ảnh → LaneGeometry (y_L, y_R, bề rộng làn, độ lệch tâm, khoảng hở, góc lệch)
   │ LaneGeometry                                              ↘ ghi log
   ▼
lane_warning.py  phần 2: LOR → ngưỡng, hysteresis, debounce → WarningResult (mức, hướng)  ↘ ghi log
```

Phần 1 không biết ngưỡng cảnh báo; phần 2 không đụng tới ảnh.

## 2. Đại lượng

| Đại lượng | Công thức | Vai trò |
|---|---|---|
| Bề rộng làn đo được | `W = y_L − y_R` | Dùng trong LOR ở từng frame |
| Độ lệch tâm | `e = (y_L + y_R)/2` | Hiển thị (dương: xe lệch phải) |
| **LOR** (Tài liệu 2) | `LOR = (W/2 − d_min) / (T_H · W/2)`, `d_min = min(y_L, −y_R)`, `T_H = 0.8` | **Quyết định cảnh báo** |
| Khoảng hở | `c = d_min − (w_xe + w_vạch)/2` | Hiển thị / ghi CSV (cm; ≤ 0 là đã đè vạch) |
| Góc lệch | `ψ = −arctan(dy/dx)` | Chỉ hiển thị (homography chưa đúng) |

`LOR = 0` khi xe ở giữa làn, `= 1.25` khi tâm xe nằm trên tâm vạch. Khi đo được cả hai vạch,
`LOR = |e| / (T_H · W/2)`, tức độ lệch tâm chia cho nửa làn **đo được**. Hướng cảnh báo là phía vạch gần hơn.

## 3. Thông số cảnh báo

| Thông số | Giá trị | Ý nghĩa trên sa bàn | Nguồn |
|---|---|---|---|
| Cấp 1 — lệch làn (vàng) | LOR > **0.45** | góc đầu xe cách mép vạch ≤ 1.8 cm | Lựa chọn của dự án, nằm trong vùng ISO (mục 4) |
| Cấp 2 — chạm vạch (đỏ) | LOR > **0.55** | góc đầu xe chạm mép trong vạch (khoảng hở = 0) | Hình học: (0.215 − 0.105 − 0.015)/(0.8·0.215) = 0.552 |
| T_H của LOR | 0.8 | — | Tài liệu 2 |
| Debounce | ≥ 200 ms mới bật (mỗi cấp một bộ đếm) | — | Tài liệu 3 |
| Nhả cảnh báo | an toàn ≥ 300 ms mới hạ cấp | — | Tài liệu 3 |
| Hysteresis | −0.05 LOR (≈ 0.9 cm) cho cả hai cấp: cấp 1 còn 0.40 khi đang đếm hoặc đang bật; cấp 2 còn 0.50 chỉ khi đã bật | — | Ý tưởng từ Tài liệu 3, giá trị tự chọn |

Cấp 1 = 0.45 được giữ vì hai lý do không liên quan tới vận tốc. Thứ nhất, nó nằm trong vùng ISO (mục 4), cách vạch sớm
nhất 0.25 LOR. Thứ hai, nó cách cấp 2 là 0.10 LOR = 1.7 cm, lớn hơn dung sai ±1 cm, nên hai cấp vẫn tách được trong sai
số đo cho phép. Phiên bản trước suy giá trị này từ tốc độ lệch 0.5 m/s; cách suy đó đã bỏ.

Ngưỡng nằm trong `LaneWarning` ([lane_warning.py](lane_warning.py): `LOR_DEPART_TH`, `_make_lor_thresholds()`). Muốn
đổi kích thước sa bàn hay xe thì sửa `TrackAndVehicleParams` trong [main.py](main.py).

## 4. Tiêu chuẩn dùng để đánh giá

### 4.1 Nội dung lấy từ tiêu chuẩn

| Nội dung | Giá trị | Nguồn | Mức đối chiếu |
|---|---|---|---|
| Lane boundary | Nằm tại **tâm vạch sơn** | ISO 17361:2017, mục 3.4 | Chính thức (bản xem trước) |
| Departure | Xét tại **mép ngoài bánh trước** | ISO 17361:2017, mục 3.6 | Chính thức (bản xem trước) |
| Vùng đặt ngưỡng, vạch sớm nhất / muộn nhất, vùng không cảnh báo | Cảnh báo phải phát giữa vạch sớm nhất và muộn nhất; không cảnh báo giữa hai vạch sớm nhất | ISO 17361:2017, mục 3.12, 3.16–3.18 | Chính thức (bản xem trước) |
| Vạch cảnh báo sớm nhất | 0.75 m phía trong lane boundary (giá trị chặt nhất, đúng với mọi tốc độ lệch) | ISO 17361:2007, trích qua TRL PPR 374, Bảng 3-1 | Thứ cấp |
| Vạch cảnh báo muộn nhất | 0.3 m phía ngoài lane boundary (xe con) | ISO 17361:2007, trích qua TRL PPR 374, mục 3.1.3.2 | Thứ cấp |
| Độ lặp lại | Cảnh báo của mỗi nhóm thử (4 lượt) nằm trong vùng rộng 30 cm | ISO 17361:2007, trích qua TRL PPR 374, phụ lục C mục 5.2 | Thứ cấp |
| Kịch bản thử | Lệch trái và phải, 0.1–0.8 m/s | UN R130, mục 6.5.1 | Chính thức |
| Dung sai vị trí cảnh báo | ±0.1 m | FMCSA-MCRR-05-005, trích qua TRL PPR 374, mục 3.1.3.2 | Thứ cấp |

"Chính thức" là đã đọc trực tiếp trên văn bản của tổ chức ban hành. "Thứ cấp" là trích qua báo cáo của TRL cho Uỷ ban
châu Âu, chưa đối chiếu với toàn văn ISO (toàn văn phải mua). Theo lời nói đầu, ISO 17361:2017 chỉ là sửa đổi nhỏ của
bản 2007 nên các số của bản 2007 được giả định vẫn đúng. Nên đối chiếu với toàn văn trước khi công bố chính thức.

### 4.2 Thu nhỏ 1:10 và đổi sang LOR

Mọi khoảng cách của ISO được nhân 1/10 theo tỉ lệ xe. Sau đó đổi sang LOR bằng chính hình học của xe và sa bàn:
`LOR(D) = (h − b − D) / (T_H·h)`, với `h = 0.215 m`, `b = 0.105 m`, và `D` là khoảng cách mép xe tới tâm vạch (dương:
phía trong làn). Không cần xe tham chiếu, không có giả thiết đồng dạng.

| Mốc | ISO, xe thật | Sa bàn: mép xe – tâm vạch | Khoảng hở tới mép trong vạch | LOR |
|---|---|---|---|---|
| Vạch cảnh báo sớm nhất | 0.75 m trong | +7.5 cm | +6.0 cm | **0.20** |
| Cấp 1 — lệch làn | — | +3.3 cm | +1.8 cm | **0.45** |
| Cấp 2 — chạm vạch | — | +1.5 cm | 0 | **0.55** |
| Mép xe tới tâm vạch (lane departure theo ISO) | 0 | 0 | −1.5 cm | 0.64 |
| Vạch cảnh báo muộn nhất | 0.3 m ngoài | −3.0 cm | −4.5 cm | **0.81** |

Các tiêu chí đánh giá suy ra:
- **Vị trí phát cảnh báo:** LOR thật lúc bật cảnh báo (cả hai cấp) phải nằm trong [0.20; 0.81].
- **Độ lặp lại:** trong mỗi nhóm 4 lượt, khoảng LOR lúc bật ≤ 0.17 (30 cm × 1/10 = 3 cm).
- **Vùng không cảnh báo:** không có cảnh báo nào khi LOR thật < 0.20.
- **Dung sai:** ±1 cm (±0.1 m × 1/10), tức 0.058 LOR.
- **Kịch bản thử:** xe dịch ngang 1 / 3 / 5 / 8 cm/s, cả hai phía (0.1–0.8 m/s của UN R130 × 1/10). Tốc độ này chỉ là
  đầu vào của phép thử; hệ thống không đo và không dùng nó.

## 5. Vì sao dùng LOR thay cho khoảng hở

**Tóm tắt:** LOR không thứ nguyên nên không đổi khi camera hoặc homography sai tỉ lệ; vì vậy có thể bỏ qua loại sai số
này của cảm biến. Khoảng hở có thứ nguyên nên mang nguyên sai số đó vào cảnh báo.

Trên làn có bề rộng cho trước `W₀`, nếu chia cho `W₀` danh định thì LOR chỉ là khoảng hở đổi thang tuyến tính:
`LOR = (c₀ − c)/(T_H·W₀/2)`. Vậy **toàn bộ khác biệt giữa hai đại lượng nằm ở phép chia cho bề rộng làn đo được.**

**Chứng minh.** Mọi phép đo đi qua homography H. Giả sử H sai tỉ lệ theo phương ngang, tức tại mặt cắt đo mọi toạ độ
ngang bị nhân với cùng hệ số g. Khi đó `y_L`, `y_R` và `W` đều bị nhân g. LOR là tỉ số của các độ dài nên **không
đổi**. Khoảng hở thì sai đúng phần tỉ lệ: `ĉ − c = (g − 1)·d_min ≈ (g − 1)·12 cm` khi xe sát vạch. Điều này vẫn đúng
khi g thay đổi theo khoảng cách, vì hai vạch được ngoại suy về cùng một mặt cắt. Homography hiện tại mắc đúng dạng sai
số này: làn đo được rộng dần từ 0.43 m lên 0.58 m theo khoảng cách.

**Thực nghiệm.** Dùng một sa bàn tổng hợp đúng thông số (làn thẳng 0.43 m, vạch 3 cm, vẽ vào ảnh camera qua H nên giá
trị đúng biết chính xác), cho LDWS dùng homography có sai số đã biết. Hai phương pháp chạy trên **cùng** dữ liệu bám
vạch; khoảng hở dùng ngưỡng 1.76 / 0.04 cm, trùng hệt LOR khi calib đúng. Mỗi điều kiện gồm 32 lượt dịch ngang và các
vị trí đứng yên.

| Điều kiện calib | Vị trí bật cấp 1 dịch (trái / phải): LOR | Vị trí bật cấp 1 dịch: khoảng hở | Báo nhầm khi xe đứng cách vạch 2.5 cm (LOR thật 0.41): LOR | Báo nhầm: khoảng hở |
|---|---|---|---|---|
| Đúng | 0 / 0 | 0 / 0 | 0/240 | 0/240 |
| Tỉ lệ ngang −5% | +0.03 / +0.02 cm | +0.71 / +0.81 cm | 0/240 | **240/240** |
| Tỉ lệ ngang +5% | 0 / 0 | −0.80 / −0.55 cm (bật muộn) | 0/240 | 0/240 |
| Phối cảnh (giống lỗi trên ảnh thật) | −0.12 / −0.24 cm | **−1.21 / −1.05 cm** (bật muộn, vượt ±1 cm) | 0/240 | 0/240 |
| Camera lệch ngang 1 cm (đối chứng) | −0.94 / +1.08 cm | −1.05 / +1.16 cm | 120/240 | 120/240 |

(Dấu + nghĩa là cảnh báo bật khi xe còn xa vạch hơn so với khi calib đúng.)

Kết luận:
1. **Bất biến với sai số tỉ lệ của calib.** Dưới sai số tỉ lệ ±5% và sai số phối cảnh, vị trí cảnh báo theo LOR dịch
   tối đa 0.25 cm; theo khoảng hở dịch tới 1.21 cm, vượt dung sai ±1 cm. Với sai số phối cảnh, khoảng hở đo xa vạch
   hơn thật nên cảnh báo bật muộn, tức sai về phía nguy hiểm. Với tỉ lệ −5%, khoảng hở báo nhầm suốt khi xe đứng yên
   cách vạch 2.5 cm.
2. **Cùng hệ quy chiếu với ISO.** LOR đo từ tâm vạch, đúng là lane boundary của ISO. Nhờ vậy vùng ISO, độ lặp lại, dung
   sai và cả hai ngưỡng nằm trên một trục duy nhất. "Vùng không cảnh báo" chỉ là một điều kiện LOR < 0.20 cho cả hai
   phía.
3. **Không thứ nguyên.** Đổi tiêu chí ISO sang sa bàn chỉ là một phép chia; kết quả so được trực tiếp với tiêu chuẩn.

Nhược điểm, cũng đo được:
- LOR **không** bù được camera lệch ngang: cả hai phương pháp cùng sai khoảng 1 cm (dòng đối chứng).
- LOR phải dùng thêm vạch xa nên nhiễu hơn khi calib đúng: RMSE 0.36 cm so với 0.27 cm của khoảng hở, vẫn trong
  ±1 cm.
- Cả hai phương pháp đều qua các phép thử ISO ở mọi điều kiện: 64/64 lượt bật trong vùng, độ lặp lại ≤ 0.031, không
  có cảnh báo trong vùng không cảnh báo. Vùng ISO đủ rộng, nên khác biệt chỉ lộ ra ở **độ chính xác của vị trí cảnh
  báo** và ở **báo nhầm gần ngưỡng**.

## 6. Kết quả theo ISO trên mô phỏng từ ảnh thật

Hiện chưa có video chạy thật có gán nhãn. Các frame được tạo từ ảnh sa bàn thật `glare.jpg` bằng phép biến đổi mặt
đường qua homography, thêm nhiễu 2 mm / 0.2° mỗi frame, 30 fps, 5 lượt mỗi nhóm.

| Tiêu chí | Mục tiêu (nguồn) | Kết quả | Kết luận |
|---|---|---|---|
| Sai số LOR, vùng quyết định | ≤ ±1 cm (FMCSA × 1/10) | 0.50 cm (tĩnh), 0.39 cm (chuỗi) | Đạt |
| Vị trí phát cảnh báo | LOR thật lúc bật ∈ [0.20; 0.81] (ISO 17361) | Cấp 1: 40/40, cấp 2: 40/40; muộn nhất 0.716 | Đạt |
| Độ lặp lại | ≤ 0.17 (ISO 17361) | 0.019 (cấp 1), 0.037 (cấp 2) | Đạt |
| Vùng không cảnh báo | 0 frame khi LOR thật < 0.20 (ISO 17361) | 0/4215 | Đạt |
| Báo nhầm (FAR); frame báo động khi an toàn | < 10% (Tài liệu 3); 0 | 0/50; 0/3450 | Đạt |
| Bỏ sót (MAR) | < 5% (Tài liệu 3) | 0/50 | Chưa đủ mẫu: cận trên 95% ≈ 6% |
| Thời gian xử lý | < 50 ms/frame (Tài liệu 3) | TB 19.3 ms, lớn nhất 26.2 ms | Đạt |
| Hiệu chuẩn | Làn 0.43 ± 0.02 m, hai vạch song song (tự đặt) | Làn 0.50–0.60 m, hai vạch chênh góc 12–13° | Không đạt |

LOR thật lúc bật cảnh báo, trung bình hai phía × 5 lượt:

| Dịch ngang (sa bàn) | Cấp 1 | Cấp 2 | Độ trễ cấp 1 (trái / phải) |
|---|---|---|---|
| 1 cm/s | 0.475 | 0.578 | 0.17 / 0.65 s |
| 3 cm/s | 0.505 | 0.606 | 0.26 / 0.36 s |
| 5 cm/s | 0.547 | 0.644 | 0.29 / 0.35 s |
| 8 cm/s | 0.600 | 0.695 | 0.29 / 0.32 s |

- LOR lúc bật tăng theo tốc độ dịch ngang của kịch bản. Lý do: debounce và bộ lọc tạo độ trễ cố định khoảng 0.29 s,
  nên dịch càng nhanh thì xe đi càng xa trong khoảng trễ đó. Phần rộng của vùng ISO chính là chỗ dành cho độ trễ này.
- Phía phải trễ hơn ở tốc độ thấp vì LOR đo phía phải thấp hơn thật khoảng 0.02: vạch phải trong ảnh gốc ở xa nên phải
  ngoại suy dài. Đây chính là nhược điểm "LOR dùng cả vạch xa".

Trên ảnh thật: `test.jpg` (xe gần giữa làn) cho LOR 0.02 và trạng thái an toàn; `glare.jpg` (xe sát vạch trái) cho
LOR 0.46 và cảnh báo lệch làn. Hình kết quả nằm trong [ket_qua/](ket_qua/).

**Giới hạn:**
- Mô phỏng từ ảnh thật dùng chính homography đang sai làm giá trị đúng, nên không phát hiện được sai số calib.
- Sa bàn tổng hợp sạch hơn thực tế; cả hai mô phỏng cho xe dịch ngang mà không xoay.
- Một phần số liệu ISO lấy qua nguồn thứ cấp.
- Hệ thống đo tại góc thân xe (0.21 m) thay cho mép ngoài bánh trước như ISO; nên đo lại bề rộng mép ngoài hai bánh
  trước của xe.

## 7. Chạy

Python 3.11, `pip install opencv-python numpy`.

```bash
python main.py                                        # nhận ảnh JPEG qua UDP 7000 (mỗi ảnh < 65 507 byte)
python main.py --video run.mp4 --csv ketqua.csv       # chạy lại video
python main.py --images thu_muc_anh --fps 30 --no-show --csv ketqua.csv
```

CSV ghi từng frame:
- Kết quả của phần 2: `t, status, level, direction, reason, raw_level, lor`.
- Thông số hình học của phần 1: `valid, n_lines, y_left_cm, y_right_cm, lane_width_cm, offset_cm, clr_left_cm,
  clr_right_cm, angle_deg`.
- Thời gian xử lý: `proc_ms`.

Ô trống nghĩa là chưa bám được vạch nào. Bấm `Space` để tạm dừng, `q`/`ESC` để thoát.

**Calib** (chi tiết trong docstring của từng script trong [calib/](calib/)):
- `calib_camera.py` tính K và hệ số méo từ ảnh bàn cờ.
- `pick_marks.py`: click các mốc trên sàn để tính homography (luôn truyền `--camera_calib`).
- Chép kết quả vào `GroundCalibration` trong `main.py`.
- Kiểm tra: đặt xe thẳng giữa làn; trên BEV hai vạch phải song song và làn rộng khoảng 43 cm.

## 8. Tài liệu tham khảo

| Ký hiệu trong code | Tài liệu | Phần sử dụng |
|---|---|---|
| Tài liệu 1 | S. Anbalagan et al., "Vision-Based Ingenious Lane Departure Warning System for Autonomous Vehicles", *Sustainability*, 2023. DOI: [10.3390/su15043535](https://doi.org/10.3390/su15043535) | Góc lệch (hiển thị) |
| Tài liệu 2 | N. Sukumar, P. Sumathi, "An Improved Lane Detection and Lane Departure Warning Framework for ADAS", *IEEE Trans. Consumer Electronics*, 2024. DOI: [10.1109/TCE.2024.3387708](https://doi.org/10.1109/TCE.2024.3387708) | LOR, T_H = 0.8 |
| Tài liệu 3 | Y. Zhong, "Design and Simulation of Lane Departure Warning Assistant Driving System", *JTIE*, 2026. DOI: [10.63887/jtie.2026.2.4.29](https://doi.org/10.63887/jtie.2026.2.4.29) | Debounce, nhả, hysteresis, KPI |
| ISO 17361:2017 | *Intelligent transport systems — Lane departure warning systems — Performance requirements and test procedures*, ISO, 2017. [Bản xem trước chính thức](https://cdn.standards.iteh.ai/samples/72349/5bc19fae954f4763a85dfe0eb3ce2a6a/ISO-17361-2017.pdf) | Định nghĩa mục 3.4, 3.6, 3.12, 3.16–3.18 |
| ISO 17361:2007 | Bản thứ nhất của tiêu chuẩn trên. Chưa đối chiếu toàn văn | 0.75 m, 0.3 m, 30 cm (qua TRL) |
| TRL PPR 374 | C. Visvikis, T. L. Smith, M. Pitcher, R. Smith, *Study on lane departure warning and lane change assistant systems — Final report*, Transport Research Laboratory cho European Commission DG Enterprise and Industry, 11/2008. [PDF](https://circabc.europa.eu/sd/a/30d82a3c-bdc9-4034-b197-62286988bd03/report_ldwlca_en.pdf) | Mục 3.1.3.2, Bảng 3-1, phụ lục C mục 5.1–5.3 |
| UN R130 | UNECE, *UN Regulation No. 130 — Uniform provisions concerning the approval of motor vehicles with regard to the Lane Departure Warning System (LDWS)*. [PDF](https://www.mlit.go.jp/jidosha/un/UN_R130.pdf) | Mục 2.6, 6.5.1 |
| FMCSA-MCRR-05-005 | U.S. DOT FMCSA, *Concept of Operations and Voluntary Operational Requirements for LDWS On-board Commercial Motor Vehicles*, 2005. Trích qua TRL | Dung sai ±0.1 m |

File PDF của các bài báo và của ISO không đưa vào repo.

## Cấu trúc thư mục

```
main.py          # thông số đầu vào, nhận frame (UDP / video / ảnh), nối hai phần, ghi CSV
lane_vision.py   # phần 1: xử lý ảnh → thông số hình học của làn, vẽ hiển thị
lane_warning.py  # phần 2: LOR → ngưỡng, debounce, hysteresis → mức cảnh báo
calib/           # script calib ống kính và homography, ảnh và mốc calib mẫu
ket_qua/         # hình kết quả
BaoCao_LDWS.pdf  # báo cáo chi tiết
CLAUDE.md        # ghi chú kỹ thuật cho Claude Code
```
