CHÉP 3 FILE MODEL VÀO THƯ MỤC NÀY (đúng tên):
    models\xe_may.pt
    models\nguoi_di_bo.pt
    models\o_to.pt
rồi chạy:  .venv\Scripts\python main.py
Không phải sửa code. Dòng đầu chương trình in BẬT/TẮT cho từng vật thể; thiếu file nào thì vật đó luôn gửi -1.
(Lỡ chép ra thư mục ngoài D:\detectkhoangcach cũng được nhận.)

Cần kiểm tra trong cau_hinh.py:
  - real_height_m: đang là cỡ THẬT ngoài đường (xe máy 1.10 m, người 1.62 m, ô tô 1.50 m).
    Chạy với vật thể mô hình (đồ chơi, sa bàn) thì đo lại chiều cao của chính vật thể đó.
  - Mỗi file nên train 1 lớp. File có nhiều lớp (vd. yolo11n.pt gốc 80 lớp) -> main.py in [CANH BAO];
    khi đó điền names=("tên lớp",) cho vật thể đó trong cau_hinh.py.

Cách khác: một file train chung 3 lớp (vd. models\vat_can.pt có lớp xe_may, nguoi_di_bo, o_to):
    trong cau_hinh.py điền model="models/vat_can.pt" cho cả 3 vật thể và names=("xe_may",) / ("nguoi_di_bo",) / ("o_to",)
    -> nhanh gấp ~3 lần (chạy 1 model thay vì 3).
