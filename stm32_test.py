#!/usr/bin/env python3
"""
stm32_test.py - thử Nucleo (firmware anh An) trực tiếp, KHÔNG cần PC/main_master: cắm Nucleo vào Pi 5 (hoặc PC) bằng
cáp mini-USB. Gửi đúng lệnh như pi_bridge: "#speed:..;;", "#steer:..;;", "#dir:..;;".

    python3 stm32_test.py steer          # lái trái 20 -> giữa -> phải 20: kiểm tra chiều (+ phải là PHẢI), tâm
    python3 stm32_test.py speed 0.3      # KÊ BÁNH LÊN: chạy 0.3 m/s trong 4 s (số âm = lùi), xem tốc độ @hall
    python3 stm32_test.py monitor        # chỉ in @hall / @imu Nucleo gửi lên (đẩy xe tay để xem tốc độ, yaw)
Cổng mặc định lấy từ NUCLEO_SERIAL trong pi_bridge.py (/dev/ttyACM0). Trên Windows: --serial COM5.
Ctrl+C bất kỳ lúc nào: gửi lệnh dừng.
"""

import argparse
import time

from pi_bridge import NUCLEO_BAUD, NUCLEO_SERIAL, NucleoLink

PERIOD_S = 0.02   # 50 lần/s như main_master


def show(n, prefix=""):
    speed = "chua nhan @hall" if n.speed is None else f"v {n.speed:+.3f} m/s"
    imu = "" if n.imu is None else " | roll {:.1f} pitch {:.1f} yaw {:.1f}".format(*n.imu[:3])
    other = "".join(f" | @{k}:{v}" for k, v in n.other.items())
    print(f"{prefix}{speed}{imu}{other}", flush=True)


def hold(n, seconds, speed=0.0, steer=0.0, direction=0, send=True, every=0.25, prefix=""):
    end, next_show = time.monotonic() + seconds, time.monotonic()
    while time.monotonic() < end:
        if send:
            n.send(speed, steer, direction)
        if time.monotonic() >= next_show:
            next_show += every
            show(n, prefix)
        time.sleep(PERIOD_S)


def main():
    ap = argparse.ArgumentParser(description="Thu Nucleo (firmware anh An) qua USB")
    ap.add_argument("--serial", default=NUCLEO_SERIAL)
    ap.add_argument("test", choices=["steer", "speed", "monitor"])
    ap.add_argument("value", nargs="?", type=float, default=0.3, help="speed: m/s (am = lui)")
    args = ap.parse_args()

    n = NucleoLink(args.serial, NUCLEO_BAUD)
    deadline = time.monotonic() + 5
    while n.ser is None and time.monotonic() < deadline:
        n.maintain()
        time.sleep(0.1)
    if n.ser is None:
        return
    try:
        if args.test == "monitor":
            print("Chi doc, khong gui lenh. Ctrl+C de thoat.", flush=True)
            hold(n, 3600, send=False, every=0.5)
        elif args.test == "steer":
            for deg in (-20, -10, 0, 10, 20, 0):
                print(f"--- lai {deg:+d} do (+ phai lam banh quay sang PHAI)", flush=True)
                hold(n, 1.5, steer=deg, every=1.5)
        else:
            d = 1 if args.value >= 0 else -1
            print(f"--- toc do {abs(args.value):.2f} m/s chieu {d:+d} trong 4 s. BANH PHAI KE KHOI SAN.", flush=True)
            hold(n, 4.0, speed=abs(args.value), direction=d)
    except KeyboardInterrupt:
        pass
    finally:
        print("--- dung", flush=True)
        for _ in range(10):
            n.send_stop()
            time.sleep(PERIOD_S)
        n.close()


if __name__ == "__main__":
    main()
