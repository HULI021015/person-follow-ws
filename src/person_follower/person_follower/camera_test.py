#!/usr/bin/env python3
"""Measure Dahua NetSDK capture freshness without publishing robot commands."""

from __future__ import annotations

import argparse
import os
import statistics
import time
from pathlib import Path

import cv2

from .dahua_camera import DahuaCamera


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Test NetSDK/PlaySDK capture and report latest-frame latency."
    )
    parser.add_argument("--ip", default=os.environ.get("DAHUA_IP", "192.168.1.21"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("DAHUA_PORT", "37777")))
    parser.add_argument("--user", default=os.environ.get("DAHUA_USER", "admin"))
    parser.add_argument(
        "--password", default=os.environ.get("DAHUA_PASSWORD", "123456baoye!!")
    )
    parser.add_argument("--channel", type=int, default=0)
    parser.add_argument("--stream", choices=("main", "extra"), default="extra")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--snapshot", help="Optional path for the newest decoded frame")
    return parser.parse_args()


def main() -> int:
    args = _arguments()
    camera = DahuaCamera(
        args.ip,
        args.port,
        args.user,
        args.password,
        args.channel,
        args.stream,
        event_callback=lambda message, is_error: print(
            f"{'ERROR' if is_error else 'INFO'}: {message}", flush=True
        ),
    )
    ages_ms = []
    source_fps = 0
    last_version = -1
    latest_image = None
    started = time.monotonic()
    deadline = started + max(args.duration, 1.0)
    try:
        camera.login()
        camera.start()
        while time.monotonic() < deadline:
            frame = camera.read_latest(last_version)
            if frame is None:
                time.sleep(0.002)
                continue
            last_version = frame.version
            latest_image = frame.image
            source_fps = frame.source_fps
            ages_ms.append((time.monotonic() - frame.received_monotonic) * 1000.0)
    finally:
        camera.close()

    elapsed = time.monotonic() - started
    if not ages_ms:
        raise RuntimeError("测试期间没有收到解码帧")
    ages_sorted = sorted(ages_ms)
    p95_index = min(len(ages_sorted) - 1, int(len(ages_sorted) * 0.95))
    _version, _received, drops = camera.status()
    print(
        "RESULT "
        f"frames={len(ages_ms)} observed_fps={len(ages_ms) / elapsed:.2f} "
        f"source_fps={source_fps} age_mean_ms={statistics.fmean(ages_ms):.2f} "
        f"age_p95_ms={ages_sorted[p95_index]:.2f} age_max_ms={max(ages_ms):.2f} "
        f"playsdk_input_drops={drops}"
    )
    if args.snapshot and latest_image is not None:
        output = Path(args.snapshot).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(output), latest_image):
            raise RuntimeError(f"无法保存快照：{output}")
        print(f"SNAPSHOT {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
