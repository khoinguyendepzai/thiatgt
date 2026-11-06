"""
Offline lane-change demo.

Without --input, generate a synthetic BEV.
"""

import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path

import cv2
import numpy as np

from lane_change_planner import (
    LaneChangePlanner,
    PlannerConfig,
)


def synthetic_bev():
    image = np.full(
        (800, 800, 3),
        35,
        dtype=np.uint8,
    )

    for x in (300, 500):
        for y in range(0, 800, 100):
            cv2.line(
                image,
                (x, y),
                (x, y + 48),
                (255, 255, 255),
                5,
            )

    return image


def _json_safe(value):
    if isinstance(value, np.ndarray):
        return _json_safe(
            value.tolist()
        )

    if isinstance(value, dict):
        return {
            key: _json_safe(item)
            for key, item in value.items()
        }

    if isinstance(value, (list, tuple)):
        return [
            _json_safe(item)
            for item in value
        ]

    if (
        isinstance(value, float)
        and not math.isfinite(value)
    ):
        return None

    return value


def main():
    parser = argparse.ArgumentParser(
        description=__doc__
    )

    parser.add_argument(
        "--input",
        help="Clean calibrated BEV image",
    )

    parser.add_argument(
        "--distance",
        type=float,
        default=0.9,
        help=(
            "Ego-center to obstacle-center distance "
            "along lane, meters"
        ),
    )

    parser.add_argument(
        "--distance-age",
        type=float,
        default=0,
        help="Age of distance sample, seconds",
    )

    parser.add_argument(
        "--speed",
        type=float,
        default=0.3,
        help="Ego speed, m/s",
    )

    parser.add_argument(
        "--scale-x",
        type=float,
        default=0.002,
        help="Horizontal meters per pixel",
    )

    parser.add_argument(
        "--scale-y",
        type=float,
        default=0.002,
        help="Vertical meters per pixel",
    )

    parser.add_argument(
        "--ego-x",
        type=float,
    )

    parser.add_argument(
        "--ego-y",
        type=float,
    )

    parser.add_argument(
        "--lane-width",
        type=float,
        default=0.4,
        help="Expected lane width, meters",
    )

    parser.add_argument(
        "--config",
        help="JSON file containing PlannerConfig overrides",
    )

    parser.add_argument(
        "--output",
        default="demo_output/proposal.png",
    )

    args = parser.parse_args()

    overrides = (
        json.loads(
            Path(args.config).read_text(
                encoding="utf-8"
            )
        )
        if args.config
        else {}
    )

    options = {
        "speed_mps": args.speed,
        "meters_per_pixel_x": args.scale_x,
        "meters_per_pixel_y": args.scale_y,
        "ego_x_px": args.ego_x,
        "ego_y_px": args.ego_y,
        "lane_width_m": args.lane_width,
    }

    options.update(overrides)

    config = PlannerConfig(**options)

    image = (
        cv2.imread(args.input)
        if args.input
        else synthetic_bev()
    )

    if image is None:
        parser.error(
            "Cannot read input BEV"
        )

    planner = LaneChangePlanner(config)

    result = planner.propose(
        image,
        args.distance,
        distance_age_s=args.distance_age,
    )

    output = Path(args.output)

    if output.suffix.lower() not in (
        ".png",
        ".jpg",
        ".jpeg",
    ):
        parser.error(
            "--output must use .png, .jpg or .jpeg"
        )

    output.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    success = cv2.imwrite(
        str(output),
        result.annotated_bev,
    )

    if not success:
        raise OSError(
            f"Cannot write {output}"
        )

    report = {
        "status": result.status,
        "reason": result.reason,
        "assumptions": result.assumptions,
        "config": asdict(config),
        "selected": (
            asdict(result.selected)
            if result.selected is not None
            else None
        ),
        "candidates": [
            asdict(candidate)
            for candidate in result.candidates
        ],
    }

    json_output = output.with_suffix(
        ".json"
    )

    json_output.write_text(
        json.dumps(
            _json_safe(report),
            indent=2,
            allow_nan=False,
        ),
        encoding="utf-8",
    )

    print(
        f"{result.status}: {result.reason}"
    )
    print(f"BEV: {output}")
    print(f"Diagnostics: {json_output}")

    if result.selected is not None:
        print(
            "Selected direction:",
            result.selected.direction,
        )

        print(
            "Total cost:",
            result.selected.cost,
        )

        print(
            "Cost components:",
            result.selected.cost_terms,
        )


if __name__ == "__main__":
    main()