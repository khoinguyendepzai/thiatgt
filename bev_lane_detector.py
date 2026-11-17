"""
Baseline detection of two white lane boundaries on BEV.

Does not detect vehicles or verify adjacent-lane occupancy.
"""

import cv2
import numpy as np

from lane_change_planner import LaneObservation


def _marking(
    mask,
    coefficients,
    y_min,
    y_max,
):
    occupancy = []

    for y in range(y_min, y_max):
        x = round(
            float(
                np.polyval(
                    coefficients,
                    y,
                )
            )
        )

        left = max(0, x - 5)
        right = min(
            mask.shape[1],
            x + 6,
        )

        occupied = (
            right > left
            and np.count_nonzero(
                mask[y, left:right]
            ) >= 2
        )

        occupancy.append(occupied)

    occupied_rows = np.asarray(
        occupancy,
        dtype=np.uint8,
    )

    if len(occupied_rows) < 30:
        return "unknown"

    occupied_rows = cv2.morphologyEx(
        occupied_rows[:, None],
        cv2.MORPH_CLOSE,
        np.ones(
            (3, 1),
            dtype=np.uint8,
        ),
    ).ravel()

    coverage = float(
        np.mean(occupied_rows)
    )

    if coverage > 0.85:
        return "solid"

    padded = np.r_[
        0,
        occupied_rows,
        0,
    ].astype(int)

    starts = np.flatnonzero(
        np.diff(padded) == 1
    )

    ends = np.flatnonzero(
        np.diff(padded) == -1
    )

    runs = ends - starts
    gaps = starts[1:] - ends[:-1]

    if (
        0.15 < coverage < 0.85
        and np.count_nonzero(runs >= 5) >= 2
        and np.any(gaps >= 5)
    ):
        return "dashed"

    return "unknown"


def detect_lanes(
    bev,
    config,
):
    height, width = bev.shape[:2]

    ego_x, ego_y = config.origin(
        bev.shape
    )

    hsv = cv2.cvtColor(
        bev,
        cv2.COLOR_BGR2HSV,
    )

    mask = cv2.inRange(
        hsv,
        (0, 0, 180),
        (180, 65, 255),
    )

    histogram = np.count_nonzero(
        mask[height // 3:],
        axis=0,
    )

    middle = int(ego_x)

    if middle < 1 or middle >= width - 1:
        return None

    seeds = [
        int(
            np.argmax(
                histogram[:middle]
            )
        ),
        middle + int(
            np.argmax(
                histogram[middle:]
            )
        ),
    ]

    if any(
        histogram[x] < 10
        for x in seeds
    ):
        return None

    ys, xs = np.nonzero(mask)

    fits = []
    observed_ranges = []

    margin = max(
        12,
        min(
            60,
            int(
                0.12
                / config.meters_per_pixel_x
            ),
        ),
    )

    window_height = max(
        15,
        height // 16,
    )

    for seed in seeds:
        current = seed
        indices = []

        for bottom in range(
            height,
            0,
            -window_height,
        ):
            top = max(
                0,
                bottom - window_height,
            )

            found = np.flatnonzero(
                (ys >= top)
                & (ys < bottom)
                & (
                    abs(xs - current)
                    <= margin
                )
            )

            if len(found) >= 5:
                current = int(
                    np.median(
                        xs[found]
                    )
                )

                indices.extend(
                    found.tolist()
                )

        if len(indices) < 50:
            return None

        iy = ys[indices]
        ix = xs[indices]

        if (
            np.ptp(iy) < 0.65 * height
            or np.max(iy)
            < ego_y
            - 0.1 / config.meters_per_pixel_y
        ):
            return None

        observed_rows = np.unique(iy)

        largest_gap_m = (
            np.max(
                np.diff(observed_rows)
            )
            * config.meters_per_pixel_y
        )

        if largest_gap_m > config.max_lane_gap_m:
            return None

        coefficients = np.polyfit(
            iy,
            ix,
            2,
        )

        residual = np.sqrt(
            np.mean(
                (
                    np.polyval(
                        coefficients,
                        iy,
                    ) - ix
                )**2
            )
        )

        if residual > 10:
            return None

        fits.append(coefficients)

        observed_ranges.append(
            (
                int(iy.min()),
                int(iy.max()),
            )
        )

    top = max(
        observed_range[0]
        for observed_range in observed_ranges
    )

    yy = np.linspace(
        ego_y,
        top,
        max(
            10,
            int(
                (ego_y - top) / 5
            ) + 1,
        ),
    )

    left, right = [
        np.polyval(
            coefficients,
            yy,
        )
        for coefficients in fits
    ]

    widths = (
        right - left
    ) * config.meters_per_pixel_x

    if (
        np.any(widths <= 0)
        or np.any(
            abs(
                widths - config.lane_width_m
            )
            > 0.3 * config.lane_width_m
        )
    ):
        return None

    if not (
        left[0] < ego_x < right[0]
    ):
        return None

    centerline = np.column_stack(
        (
            (left + right) / 2,
            yy,
        )
    )

    return LaneObservation(
        centerline_px=centerline,
        lane_width_m=float(
            np.median(widths)
        ),
        left_marking=_marking(
            mask,
            fits[0],
            top,
            int(ego_y) + 1,
        ),
        right_marking=_marking(
            mask,
            fits[1],
            top,
            int(ego_y) + 1,
        ),
    )