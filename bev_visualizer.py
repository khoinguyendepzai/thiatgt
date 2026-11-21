"""Draw the proposal on a copy of the BEV."""

import cv2
import numpy as np
from collections import Counter


def draw_plan(
    bev,
    result,
    config,
    lanes=None,
):
    output = bev.copy()
    center_color = (255, 200, 0)
    plan_color = (0, 255, 0)
    legend = []

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

    if lanes is not None:
        center = np.asarray(lanes.centerline_px, dtype=float)
        if (center.ndim == 2 and center.shape[1] == 2
                and len(center) >= 2 and np.isfinite(center).all()):
            points = np.rint(center[np.argsort(-center[:, 1])]).astype(np.int32)
            # Keep the center visible around the narrower path when they coincide.
            cv2.polylines(output, [points], False, center_color, 7, cv2.LINE_AA)
            legend.append(("Centerline", center_color))

    # Selected trajectory.
    if result.selected is not None:
        points = np.rint(
            result.selected.path_px
        ).astype(np.int32)

        cv2.polylines(
            output,
            [points],
            False,
            plan_color,
            3,
            cv2.LINE_AA,
        )
        legend.append((f"Plan {result.selected.direction}", plan_color))

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

    if result.candidates:
        scores = []
        for direction in ("KEEP", "LEFT", "RIGHT"):
            group = [c for c in result.candidates if c.direction == direction]
            costs = [c.cost for c in group if c.feasible]
            if group:
                score = f"{min(costs):.3f}" if costs else "rejected"
                scores.append(f"{direction} {score} ({len(costs)}/{len(group)})")
        lines.append("Best cost: " + " | ".join(scores))
        if result.selected is None:
            reasons = Counter(reason for c in result.candidates for reason in c.rejection_reasons)
            lines.extend(f"{reason}: {count}" for reason, count in reasons.most_common())
    if lanes is not None:
        lines.append(f"Markings: L={lanes.left_marking}  R={lanes.right_marking}")

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

    for index, (label, color) in enumerate(legend):
        y = output.shape[0] - 12 - 20 * index
        cv2.line(output, (8, y), (28, y), color, 3, cv2.LINE_AA)
        for ink, thickness in [((0, 0, 0), 3), (color, 1)]:
            cv2.putText(output, label, (34, y + 4), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, ink, thickness, cv2.LINE_AA)

    return output
