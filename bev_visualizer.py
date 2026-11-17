"""Draw the proposal on a copy of the BEV."""

import cv2
import numpy as np


def draw_plan(
    bev,
    result,
    config,
):
    output = bev.copy()

    # Other feasible trajectories.
    for candidate in result.candidates:
        if candidate.feasible:
            points = np.rint(
                candidate.path_px
            ).astype(np.int32)

            cv2.polylines(
                output,
                [points],
                False,
                (100, 100, 100),
                1,
            )

    # Selected trajectory.
    if result.selected is not None:
        points = np.rint(
            result.selected.path_px
        ).astype(np.int32)

        cv2.polylines(
            output,
            [points],
            False,
            (0, 255, 0),
            3,
            cv2.LINE_AA,
        )

        cv2.circle(
            output,
            tuple(points[-1]),
            5,
            (0, 255, 255),
            -1,
        )

    ego_x, ego_y = config.origin(
        bev.shape
    )

    cv2.circle(
        output,
        (
            round(ego_x),
            round(ego_y),
        ),
        5,
        (255, 150, 0),
        -1,
    )

    lines = [result.status]

    if result.selected is not None:
        candidate = result.selected

        lines.extend(
            [
                (
                    f"{candidate.direction} "
                    f"cost={candidate.cost:.3f} "
                    f"T={candidate.duration_s:.2f}s"
                ),
                (
                    "clearance="
                    f"{candidate.min_clearance_m:.3f}m "
                    "TTC(static)="
                    f"{candidate.ttc_s:.2f}s"
                ),
            ]
        )
    else:
        lines.append(result.reason)

    lines.append(
        "ASSUMPTION: static front obstacle; "
        "adjacent traffic NOT checked"
    )

    for index, line in enumerate(lines):
        position = (
            8,
            22 + 22 * index,
        )

        cv2.putText(
            output,
            line,
            position,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (0, 0, 0),
            3,
            cv2.LINE_AA,
        )

        cv2.putText(
            output,
            line,
            position,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

    return output