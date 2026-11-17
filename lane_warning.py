"""Phần 2 — cảnh báo: từ thông số hình học của phần 1 (LaneGeometry) tính LOR, so ngưỡng, debounce, hysteresis.

Hai mức, đều chỉ xét vị trí, đều so sánh MỘT đại lượng là LOR (Tài liệu 2):
    cấp 1 LỆCH LÀN (vàng): LOR > 0.45      cấp 2 CHẠM VẠCH (đỏ): LOR > 0.55
Không dùng vận tốc ở bất kỳ đâu, kể cả khi đặt ngưỡng.

Vì sao dùng LOR thay cho khoảng hở (clearance):
    LOR = (W/2 - d_min) / (T_H·W/2) là tỉ số của các độ dài đo trên cùng một mặt cắt, tức đại lượng KHÔNG THỨ NGUYÊN.
    Nếu camera / homography sai tỉ lệ (mọi toạ độ ngang bị nhân cùng một hệ số g, kể cả g đổi theo khoảng cách kiểu
    sai số phối cảnh) thì y_L, y_R, W cùng bị nhân g và LOR KHÔNG ĐỔI: có thể bỏ qua loại sai số này của cảm biến.
    Khoảng hở có thứ nguyên (cm) nên sai đúng (g - 1)·d_min ~ (g - 1)·12 cm khi sát vạch. Trên làn cho trước, đây là
    khác biệt duy nhất giữa hai đại lượng (chia cho W đo được hay W danh định). Đo trên sa bàn tổng hợp: dưới sai số tỉ
    lệ ±5% và sai số phối cảnh, vị trí cảnh báo theo LOR dịch <= 0.25 cm, theo khoảng hở tới 1.21 cm (báo cáo PDF mục
    4.8, 6.4). Giới hạn: LOR KHÔNG bù được sai số dịch chuyển (camera lệch ngang, góc lệch) và nhiễu hơn một chút vì
    dùng cả vạch xa.

Vùng đặt ngưỡng của ISO 17361 (thu nhỏ 1:10, đổi sang LOR) được tính ở đây chỉ để đánh giá, không dùng để quyết định.
"""

from dataclasses import dataclass


@dataclass
class WarningResult:
    lor: float  # LOR tại đầu xe (0 ở giữa làn, 1.25 khi tâm xe trên tâm vạch gần nhất)
    raw_level: int  # mức theo ngưỡng ở frame này, trước debounce
    reason: str  # "LANE DEPARTURE" / "LINE TOUCH" / ""
    level: int  # mức cảnh báo sau debounce: 0 an toàn, 1 lệch làn, 2 chạm vạch
    status: str  # "SAFE" hoặc "LEFT: LANE DEPARTURE"...
    direction: str  # "LEFT" / "RIGHT" / ""


class LaneWarning:
    """LaneGeometry -> WarningResult. params: TrackAndVehicleParams, iso: IsoLdwCriteria (khai báo trong main.py)."""

    # Cảnh báo hai mức, cả hai so sánh cùng một đại lượng LOR tính tại đầu xe (x = rear_axle_to_front). Không dùng
    # vận tốc ở bất kỳ đâu, kể cả khi đặt ngưỡng.
    SAFE_ZONE_TH = 0.80  # T_H = 80% nửa độ rộng làn (Tài liệu 2)
    # Cấp 1 LỆCH LÀN: lựa chọn của dự án, nằm trong vùng ISO quy đổi [0.20; 0.81]. LOR 0.45 ~ góc đầu xe cách mép vạch
    # 1.8 cm; cách cấp 2 0.10 LOR (1.7 cm), lớn hơn dung sai ±1 cm (±0.1 m của FMCSA-MCRR-05-005 thu nhỏ 1:10) nên hai
    # cấp vẫn tách được trong sai số đo cho phép. Cấp 2 CHẠM VẠCH tính hình học trong _make_lor_thresholds()
    LOR_DEPART_TH = 0.45
    DEBOUNCE_TIME_S = 0.20  # Duy trì lỗi > 200ms mới kích hoạt (Tài liệu 3)
    RELEASE_TIME_S = 0.30  # Trở về vùng an toàn > 300ms mới tắt (Tài liệu 3)
    # Hysteresis ngưỡng (Tài liệu 3): đang đếm debounce hoặc đang cảnh báo ở cấp nào thì ngưỡng LOR của cấp đó hạ
    # xuống chừng này, để nhiễu đo quanh ngưỡng không làm cảnh báo chập chờn hay đặt lại bộ đếm 200 ms.
    # 0.05 LOR ~ 0.9 cm trên làn 0.43 m, khoảng 4 lần độ lệch chuẩn của vị trí vạch sau lọc
    HYST_LOR = 0.05

    def __init__(self, params, iso):
        self.params = params
        self.iso = iso
        self._make_lor_thresholds()
        self.warning_level = 0  # 0: An toàn, 1: Lệch làn, 2: Chạm vạch
        self.warning_side = None  # "LEFT" / "RIGHT" khi đang cảnh báo
        self.level_since = {1: None, 2: None}  # thời điểm điều kiện cấp 1 / cấp 2 bắt đầu đúng liên tục
        self.safe_start_time = None

    def lor_at_gap(self, gap_m: float) -> float:
        """LOR khi mép ngoài xe ở đầu xe cách lane boundary (tâm vạch) gap_m trên sa bàn (dương: phía trong làn).

        Khoảng cách tâm xe -> tâm vạch là d = b + gap_m (b = nửa bề rộng xe) nên LOR = (h - b - gap_m) / (T_H·h), h = nửa
        bề rộng làn danh định. Bề rộng xe 0.21 m được dùng thay cho khoảng cách mép ngoài hai bánh trước.
        """
        h = 0.5 * self.params.lane_width
        return (h - 0.5 * self.params.vehicle_width - gap_m) / (self.SAFE_ZONE_TH * h)

    def _make_lor_thresholds(self):
        """Ngưỡng LOR của hai cấp cảnh báo và vùng ISO 17361 quy đổi (vùng ISO chỉ để đánh giá)."""
        s = self.iso.scale
        self.lor_iso_earliest = self.lor_at_gap(s * self.iso.earliest_line)  # 0.20: dưới mức này không được cảnh báo
        self.lor_iso_boundary = self.lor_at_gap(0.0)  # 0.64: mép xe tới tâm vạch (lane departure theo ISO)
        self.lor_iso_latest = self.lor_at_gap(-s * self.iso.latest_line)  # 0.81
        # Cấp 2 CHẠM VẠCH: mép xe tới mép trong vạch (khoảng hở = 0), thuần hình học
        self.lor_touch = round(self.lor_at_gap(0.5 * self.params.line_width), 2)  # 0.552 -> 0.55
        self.lor_depart = self.LOR_DEPART_TH  # 0.45

    def _lor_level(self, lor: float, h1: bool = False, h2: bool = False):
        """Mức cảnh báo theo LOR. h1 / h2: hạ ngưỡng cấp 1 / cấp 2 đi HYST_LOR (hysteresis)."""
        if lor > self.lor_touch - (self.HYST_LOR if h2 else 0.0):
            return 2, "LINE TOUCH"
        if lor > self.lor_depart - (self.HYST_LOR if h1 else 0.0):
            return 1, "LANE DEPARTURE"
        return 0, ""

    def update(self, geom) -> WarningResult:
        """LOR (Tài liệu 2) -> mức cảnh báo + debounce (Tài liệu 3). geom: LaneGeometry của phần 1."""
        t = geom.t
        if not geom.valid:
            self._debounce(0, None, t)
            return self._result(0.0, 0, "")

        y_l, y_r = geom.y_left, geom.y_right
        # LOR (Tài liệu 2) tính tại đầu xe với bề rộng làn đo được: 0 ở giữa làn, 1/SAFE_ZONE_TH khi tâm xe tới tâm
        # vạch gần nhất. Đây là đại lượng duy nhất quyết định cảnh báo; hướng là phía vạch gần hơn
        half_lane = max(0.5 * (y_l - y_r), 1e-3)
        left_is_near = y_l <= -y_r
        min_dist = min(y_l, -y_r)
        lor = (half_lane - min_dist) / (self.SAFE_ZONE_TH * half_lane)

        # Hysteresis: hạ ngưỡng cấp 1 khi cấp 1 đang bật hoặc đang đếm debounce (để nhiễu không đặt lại bộ đếm);
        # chỉ hạ ngưỡng cấp 2 khi cấp 2 đã bật (để một đỉnh nhiễu không tự kéo dài thành cảnh báo chạm vạch giả)
        relax1 = self.warning_level >= 1 or self.level_since[1] is not None
        relax2 = self.warning_level >= 2
        raw, reason = self._lor_level(lor, relax1, relax2)
        self._debounce(raw, "LEFT" if left_is_near else "RIGHT", t)
        return self._result(float(lor), raw, reason)

    def _debounce(self, raw: int, side, t: float):
        """Cấp k bật khi điều kiện cấp k kéo dài >= 200 ms (mỗi cấp một bộ đếm), xuống cấp khi an toàn >= 300 ms
        (Tài liệu 3)."""
        for k in (1, 2):
            if raw < k:
                self.level_since[k] = None
            elif self.level_since[k] is None:
                self.level_since[k] = t
        ready = [k for k in (1, 2)
                 if self.level_since[k] is not None and t - self.level_since[k] >= self.DEBOUNCE_TIME_S]
        target = max(ready, default=0)
        if target > self.warning_level:
            # Hướng cảnh báo chốt lúc bắt đầu cảnh báo và giữ tới khi tắt: lúc xe cắt qua vạch, vạch đổi nhãn
            # trái <-> phải nhưng xe vẫn đang lệch về hướng cũ
            if self.warning_level == 0:
                self.warning_side = side
            self.warning_level = target
            self.safe_start_time = None
            return
        if raw < self.warning_level:
            if self.safe_start_time is None:
                self.safe_start_time = t
            if t - self.safe_start_time >= self.RELEASE_TIME_S:
                self.warning_level, self.safe_start_time = raw, None
        else:
            self.safe_start_time = None
        if self.warning_level == 0:
            self.warning_side = None

    def _result(self, lor: float, raw: int, reason: str) -> WarningResult:
        if self.warning_level == 0:
            status = "SAFE"
        else:
            kind = "LINE TOUCH" if self.warning_level == 2 else "LANE DEPARTURE"
            status = f"{self.warning_side}: {kind}"
        return WarningResult(lor=lor, raw_level=raw, reason=reason, level=self.warning_level, status=status,
                             direction=self.warning_side or "")

    def hud_rows(self, res: WarningResult, valid: bool):
        """Hai dòng đầu bảng HUD (chữ không dấu: putText không vẽ được tiếng Việt)."""
        state = res.status + ("" if valid else " | NO LANE")
        return [state, f"LOR: {res.lor:.2f} (L1 > {self.lor_depart:.2f}, L2 > {self.lor_touch:.2f})"]
