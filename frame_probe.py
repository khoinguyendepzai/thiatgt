"""Receive-only proof of decoded Jetson frames. Never opens serial/TCP or models."""
import argparse
import json
import math
from pathlib import Path
import socket
import time

import cv2
from frame_transport import FrameReceiver


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--lane-port', type=int, default=7000)
    parser.add_argument('--road-port', type=int, default=7001)
    parser.add_argument('--listen-ip', default='0.0.0.0')
    parser.add_argument('--source-ip', help='Expected vehicle IPv4 address; reject other senders')
    parser.add_argument('--duration', type=float, default=10)
    parser.add_argument('--output-dir', default='frame_check')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('--duration must be finite and positive')
    if args.lane_port == args.road_port or not all(0 < p < 65536 for p in (args.lane_port,args.road_port)):
        parser.error('Require two distinct UDP ports in 1..65535')
    try:
        socket.inet_pton(socket.AF_INET,args.listen_ip)
        if args.source_ip:
            socket.inet_pton(socket.AF_INET,args.source_ip)
    except OSError:
        parser.error('Expected an IPv4 address')
    output = Path(args.output_dir)
    output.mkdir(parents=True,exist_ok=True)
    receivers, images = {}, {}
    started = time.perf_counter()
    interrupted = False
    report = {}
    try:
        receivers['lane'] = FrameReceiver(args.lane_port,(640,480),source_ip=args.source_ip,listen_ip=args.listen_ip)
        receivers['road'] = FrameReceiver(args.road_port,(640,480),source_ip=args.source_ip,listen_ip=args.listen_ip)
        last_print = started-1
        print('Receive-only: no serial, no TCP commands. Waiting for decoded frames...',flush=True)
        while time.perf_counter()-started < args.duration:
            for name,rx in receivers.items():
                image,_ = rx.latest(timeout=.01)
                if image is not None:
                    images[name] = image
            now = time.perf_counter()
            if now-last_print >= 1:
                for name,rx in receivers.items():
                    s = rx.stats()
                    print(f"[{name}] packets={s['packets']} assembled={s['assembled']} decoded={s['decoded']} "
                          f"delivered={s['delivered']} id={s['last_frame_id']} source={s['last_source']} "
                          f"size_errors={s['size_errors']}",flush=True)
                last_print = now
    except KeyboardInterrupt:
        interrupted = True
    finally:
        for rx in receivers.values():
            rx.close()
        # A final decode can complete between the last latest() and close().
        # Drain after joining so saved PNGs and report IDs describe the same frame.
        for name, rx in receivers.items():
            image, _ = rx.latest(timeout=0)
            if image is not None:
                images[name] = image
        report = {name:rx.stats() for name,rx in receivers.items()}
        verified = not interrupted and len(report)==2 and all(
            r['decoded']>0 and r['delivered']>0 and r['error'] is None
            and r['last_frame_age_s'] is not None and r['last_frame_age_s'] <= .5
            for r in report.values())
        report.update(verified=verified,interrupted=interrupted,
                      timestamp_basis='server first-chunk receive time; no camera capture timestamp',
                      expected_source_ip=args.source_ip,
                      note='UDP transport proof only; inspect PNGs to verify camera identity/scene. Not YOLO or vehicle-control validation.')
        for name,image in images.items():
            if not cv2.imwrite(str(output/(name+'.png')),image):
                raise OSError('Cannot write received image')
        (output/'report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print('PASS: two streams decoded and fresh.' if report['verified'] else
          'FAIL: no verified fresh frames from one or both streams. Inspect report.json.',flush=True)
    return 0 if report['verified'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
