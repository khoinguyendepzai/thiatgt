"""
Lane-change proposal only: no vehicle commands.

Algorithm reference:
CyberTruck/Henry, commit ed227077.

path_m[:, 0]: forward distance from ego, meters.
path_m[:, 1]: lateral distance to the right, meters.
"""

from dataclasses import dataclass, field
import math

import numpy as np


@dataclass(frozen=True)
class PlannerConfig:
    speed_mps: float = 0.3

    meters_per_pixel_x: float = 0.002
    meters_per_pixel_y: float = 0.002

    # Default origin: middle column, bottom row.
    ego_x_px: float | None = None
    ego_y_px: float | None = None

    lane_width_m: float = 0.4

    vehicle_width_m: float = 0.21
    vehicle_length_m: float = 0.432

    obstacle_width_m: float = 0.20
    obstacle_length_m: float = 0.22

    safe_margin_m: float = 0.1
    trigger_distance_m: float = 1.1
    distance_timeout_s: float = 0.5

    maneuver_times_s: tuple[float, ...] = (
        1.0, 1.5, 2.0, 2.5
    )

    max_lane_gap_m: float = 0.2
    sample_step_m: float = 0.005

    max_curvature_inv_m: float = 15.0
    max_lateral_accel_mps2: float = 1.5
    max_jerk_mps3: float = 4.0

    def __post_init__(self):
        nonnegative = (
            "speed_mps",
            "safe_margin_m",
        )

        positive = (
            "meters_per_pixel_x",
            "meters_per_pixel_y",
            "lane_width_m",
            "vehicle_width_m",
            "vehicle_length_m",
            "obstacle_width_m",
            "obstacle_length_m",
            "trigger_distance_m",
            "distance_timeout_s",
            "sample_step_m",
            "max_lane_gap_m",
            "max_curvature_inv_m",
            "max_lateral_accel_mps2",
            "max_jerk_mps3",
        )

        for name in nonnegative + positive:
            value = getattr(self, name)

            if (
                not math.isfinite(value)
                or value < 0
                or (name in positive and value == 0)
            ):
                raise ValueError(
                    f"{name} has an invalid value"
                )

        times = self.maneuver_times_s

        if (
            not times
            or any(
                not math.isfinite(t) or t <= 0
                for t in times
            )
            or len(set(times)) != len(times)
        ):
            raise ValueError(
                "maneuver_times_s must contain "
                "distinct positive finite times"
            )

        for origin in (
            self.ego_x_px,
            self.ego_y_px,
        ):
            if (
                origin is not None
                and not math.isfinite(origin)
            ):
                raise ValueError(
                    "ego origin must be finite"
                )

    def origin(self, shape):
        height, width = shape[:2]

        x = (
            float(width // 2)
            if self.ego_x_px is None
            else self.ego_x_px
        )

        y = (
            height - 1
            if self.ego_y_px is None
            else self.ego_y_px
        )

        if not (
            0 <= x < width
            and 0 <= y < height
        ):
            raise ValueError(
                "ego origin must be inside BEV"
            )

        return x, y


@dataclass
class LaneObservation:
    centerline_px: np.ndarray
    lane_width_m: float
    left_marking: str = "unknown"
    right_marking: str = "unknown"


@dataclass
class Candidate:
    direction: str
    duration_s: float
    transition_length_m: float

    path_m: np.ndarray
    path_px: np.ndarray

    cost: float = math.inf

    cost_terms: dict[str, float] = field(
        default_factory=dict
    )

    rejection_reasons: list[str] = field(
        default_factory=list
    )

    min_clearance_m: float = math.inf
    max_curvature_inv_m: float = 0.0
    max_jerk_mps3: float = 0.0

    ttc_s: float | None = None

    @property
    def feasible(self):
        return (
            not self.rejection_reasons
            and math.isfinite(self.cost)
        )


@dataclass
class PlanResult:
    status: str
    reason: str

    selected: Candidate | None = None

    candidates: list[Candidate] = field(
        default_factory=list
    )

    annotated_bev: np.ndarray | None = None

    assumptions: tuple[str, ...] = (
        "static_obstacle_on_reference_centerline",
        "adjacent_lane_occupancy_unverified",
        "calibrated_ground_plane_required",
    )


def _segment_hits_box(
    a,
    b,
    half_forward,
    half_right,
):
    """
    Continuous intersection between a segment
    and an axis-aligned rectangle in Frenet.
    """
    lower = 0.0
    upper = 1.0

    for axis, half in enumerate(
        (half_forward, half_right)
    ):
        delta = b[axis] - a[axis]

        if abs(delta) < 1e-12:
            if abs(a[axis]) > half:
                return False
        else:
            t1 = (-half - a[axis]) / delta
            t2 = (half - a[axis]) / delta

            lower = max(
                lower,
                min(t1, t2),
            )
            upper = min(
                upper,
                max(t1, t2),
            )

            if lower > upper:
                return False

    return True


class LaneChangePlanner:
    def __init__(
        self,
        config: PlannerConfig | None = None,
    ):
        self.config = config or PlannerConfig()

    def propose(
        self,
        bev,
        front_distance_m,
        *,
        lanes=None,
        distance_age_s=0.0,
    ):
        """
        bev:
            uint8 BGR ground-plane image.

        front_distance_m:
            Ego-center to obstacle-center distance
            along the reference lane, in meters.

        lanes:
            Optional LaneObservation in the exact
            coordinate system of the input BEV.

        Returns:
            PlanResult, including an annotated copy
            of the input image.
        """
        valid_image = (
            isinstance(bev, np.ndarray)
            and bev.ndim == 3
            and bev.shape[2] == 3
            and bev.dtype == np.uint8
            and min(bev.shape[:2]) >= 3
        )

        if not valid_image:
            raise ValueError(
                "BEV must be a nonempty uint8 BGR image"
            )

        cfg = self.config
        origin = cfg.origin(bev.shape)

        def finish(
            status,
            reason,
            selected=None,
            candidates=None,
        ):
            from bev_visualizer import draw_plan

            result = PlanResult(
                status=status,
                reason=reason,
                selected=selected,
                candidates=candidates or [],
            )

            result.annotated_bev = draw_plan(
                bev,
                result,
                cfg,
            )

            return result

        if (
            front_distance_m is None
            or not np.isfinite(front_distance_m)
            or front_distance_m <= 0
        ):
            return finish(
                "NO_VALID_DISTANCE",
                "No valid front-obstacle distance; "
                "no maneuver proposed.",
            )

        if (
            not np.isfinite(distance_age_s)
            or distance_age_s < 0
            or distance_age_s > cfg.distance_timeout_s
        ):
            return finish(
                "STALE_DISTANCE",
                "Distance timestamp is stale or invalid.",
            )

        if cfg.speed_mps <= 1e-6:
            return finish(
                "EGO_STOPPED",
                "A positive ego speed is required "
                "to evaluate a maneuver.",
            )

        if lanes is None:
            from bev_lane_detector import detect_lanes

            lanes = detect_lanes(bev, cfg)

        reference = self._reference(
            lanes,
            origin,
        )

        if reference is None:
            return finish(
                "INVALID_LANES",
                "Missing, incomplete or invalid "
                "lane geometry.",
            )

        s, _, _ = reference

        if (
            front_distance_m <= cfg.trigger_distance_m
            and front_distance_m > s[-1]
        ):
            return finish(
                "INSUFFICIENT_HORIZON",
                "Front obstacle lies beyond "
                "observed lane geometry.",
            )

        candidates = [
            self._candidate(
                sign=0,
                duration=0.0,
                ref=reference,
                lanes=lanes,
                shape=bev.shape,
                origin=origin,
                distance=front_distance_m,
            )
        ]

        if front_distance_m <= cfg.trigger_distance_m:
            directions = (
                (-1, lanes.left_marking),
                (1, lanes.right_marking),
            )

            for sign, marking in directions:
                if marking.lower() == "dashed":
                    for duration in cfg.maneuver_times_s:
                        candidates.append(
                            self._candidate(
                                sign=sign,
                                duration=duration,
                                ref=reference,
                                lanes=lanes,
                                shape=bev.shape,
                                origin=origin,
                                distance=front_distance_m,
                            )
                        )

        valid_candidates = [
            candidate
            for candidate in candidates
            if candidate.feasible
        ]

        if not valid_candidates:
            return finish(
                "NO_FEASIBLE_TRAJECTORY",
                "No safe candidate under the stated "
                "assumptions; no path selected.",
                candidates=candidates,
            )

        best = min(
            valid_candidates,
            key=lambda candidate: candidate.cost,
        )

        status = (
            "KEEP_LANE"
            if best.direction == "KEEP"
            else "CHANGE_PROPOSED"
        )

        return finish(
            status,
            "Lowest-cost feasible trajectory under "
            "static-obstacle assumptions.",
            selected=best,
            candidates=candidates,
        )

    def _reference(
        self,
        lanes,
        origin,
    ):
        if (
            lanes is None
            or not np.isfinite(lanes.lane_width_m)
            or lanes.lane_width_m <= 0
        ):
            return None

        cfg = self.config

        points = np.asarray(
            lanes.centerline_px,
            dtype=float,
        )

        if (
            points.ndim != 2
            or points.shape[1] != 2
            or len(points) < 5
            or not np.isfinite(points).all()
        ):
            return None

        # Sort from near to far.
        points = points[
            np.argsort(-points[:, 1])
        ]

        xy = np.column_stack(
            (
                (
                    origin[1] - points[:, 1]
                ) * cfg.meters_per_pixel_y,
                (
                    points[:, 0] - origin[0]
                ) * cfg.meters_per_pixel_x,
            )
        )

        xy = xy[xy[:, 0] >= -1e-8]

        if len(xy) < 5:
            return None

        forward_steps = np.diff(xy[:, 0])

        metric_steps = np.linalg.norm(
            np.diff(xy, axis=0),
            axis=1,
        )

        if (
            np.any(forward_steps <= 0)
            or xy[0, 0] > 0.05
            or np.any(
                metric_steps > cfg.max_lane_gap_m
            )
        ):
            return None

        initial_slope = (
            (xy[1, 1] - xy[0, 1])
            / (xy[1, 0] - xy[0, 0])
        )

        # This proposal model expects ego near lane center.
        if (
            abs(xy[0, 1]) > 0.05
            or abs(initial_slope) > 0.5
        ):
            return None

        xy[0] = 0.0

        arc = np.concatenate(
            (
                [0.0],
                np.cumsum(
                    np.linalg.norm(
                        np.diff(xy, axis=0),
                        axis=1,
                    )
                ),
            )
        )

        if arc[-1] < 0.1:
            return None

        count = min(
            5000,
            max(
                20,
                int(
                    np.ceil(
                        arc[-1] / cfg.sample_step_m
                    )
                ) + 1,
            ),
        )

        # Reserve room for the front vehicle footprint.
        reserve = 0.5 * math.hypot(
            cfg.vehicle_length_m,
            cfg.vehicle_width_m,
        )

        usable_horizon = arc[-1] - reserve

        if usable_horizon < 0.1:
            return None

        s = np.linspace(
            0,
            usable_horizon,
            count,
        )

        xy = np.column_stack(
            [
                np.interp(
                    s,
                    arc,
                    xy[:, axis],
                )
                for axis in (0, 1)
            ]
        )

        tangent = np.gradient(
            xy,
            s,
            axis=0,
        )

        tangent /= np.maximum(
            np.linalg.norm(
                tangent,
                axis=1,
                keepdims=True,
            ),
            1e-9,
        )

        normals = np.column_stack(
            (
                -tangent[:, 1],
                tangent[:, 0],
            )
        )

        return s, xy, normals

    def _candidate(
        self,
        sign,
        duration,
        ref,
        lanes,
        shape,
        origin,
        distance,
    ):
        cfg = self.config
        s, xy, normals = ref

        length = cfg.speed_mps * duration
        target = sign * lanes.lane_width_m

        if sign:
            u = np.clip(
                s / length,
                0,
                1,
            )

            blend = (
                10 * u**3
                - 15 * u**4
                + 6 * u**5
            )

            d = target * blend

            # Lateral profile metrics on straight reference.
            jerk = (
                abs(target)
                * 60
                / duration**3
            )

            lateral_accel = (
                abs(target)
                * (10 / np.sqrt(3))
                / duration**2
            )
        else:
            d = np.zeros_like(s)
            jerk = 0.0
            lateral_accel = 0.0

        path = xy + d[:, None] * normals

        pixels = np.column_stack(
            (
                origin[0]
                + path[:, 1] / cfg.meters_per_pixel_x,
                origin[1]
                - path[:, 0] / cfg.meters_per_pixel_y,
            )
        )

        direction = {
            -1: "LEFT",
            0: "KEEP",
            1: "RIGHT",
        }[sign]

        candidate = Candidate(
            direction=direction,
            duration_s=duration,
            transition_length_m=length,
            path_m=path,
            path_px=pixels,
        )

        if length > s[-1]:
            candidate.rejection_reasons.append(
                "insufficient_horizon"
            )

        height, width = shape[:2]

        if (
            np.any(pixels < -1e-6)
            or np.any(
                pixels[:, 0] > width - 1 + 1e-6
            )
            or np.any(
                pixels[:, 1] > height - 1 + 1e-6
            )
        ):
            candidate.rejection_reasons.append(
                "outside_bev"
            )

        half_width_px = (
            cfg.vehicle_width_m
            / 2
            / cfg.meters_per_pixel_x
        )

        if (
            np.any(
                pixels[:, 0] - half_width_px < 0
            )
            or np.any(
                pixels[:, 0] + half_width_px >= width
            )
        ):
            candidate.rejection_reasons.append(
                "vehicle_outside_bev_width"
            )

        first = np.gradient(
            path,
            s,
            axis=0,
        )

        second = np.gradient(
            first,
            s,
            axis=0,
        )

        curvature = (
            np.abs(
                first[:, 0] * second[:, 1]
                - first[:, 1] * second[:, 0]
            )
            / np.maximum(
                np.linalg.norm(
                    first,
                    axis=1,
                )**3,
                1e-9,
            )
        )

        candidate.max_curvature_inv_m = float(
            np.max(curvature)
        )

        candidate.max_jerk_mps3 = jerk

        if (
            candidate.max_curvature_inv_m
            > cfg.max_curvature_inv_m
        ):
            candidate.rejection_reasons.append(
                "curvature_limit"
            )

        curvature_accel = (
            candidate.max_curvature_inv_m
            * cfg.speed_mps**2
        )

        if (
            max(
                lateral_accel,
                curvature_accel,
            )
            > cfg.max_lateral_accel_mps2
        ):
            candidate.rejection_reasons.append(
                "lateral_acceleration_limit"
            )

        if jerk > cfg.max_jerk_mps3:
            candidate.rejection_reasons.append(
                "jerk_limit"
            )

        long_clear = (
            (
                cfg.vehicle_length_m
                + cfg.obstacle_length_m
            ) / 2
            + cfg.safe_margin_m
        )

        lat_clear = (
            (
                cfg.vehicle_width_m
                + cfg.obstacle_width_m
            ) / 2
            + cfg.safe_margin_m
        )

        relative = np.column_stack(
            (
                s - distance,
                d,
            )
        )

        separation = np.maximum(
            np.abs(relative)
            - (long_clear, lat_clear),
            0,
        )

        candidate.min_clearance_m = float(
            np.min(
                np.linalg.norm(
                    separation,
                    axis=1,
                )
            )
        )

        collision = any(
            _segment_hits_box(
                a,
                b,
                long_clear,
                lat_clear,
            )
            for a, b in zip(
                relative[:-1],
                relative[1:],
            )
        )

        if collision:
            candidate.rejection_reasons.append(
                "collision"
            )

        # Current center closing time to a static obstacle.
        candidate.ttc_s = (
            distance / cfg.speed_mps
        )

        candidate.cost_terms = {
            "clearance": (
                2.0
                * np.exp(
                    -candidate.min_clearance_m / 0.1
                )
            ),
            "curvature": (
                candidate.max_curvature_inv_m
                / cfg.max_curvature_inv_m
            ),
            "jerk": (
                0.3
                * jerk
                / cfg.max_jerk_mps3
            ),
            "duration": (
                0.25
                * duration
                / max(cfg.maneuver_times_s)
            ),
            "lane_change": (
                0.2 * abs(sign)
            ),
        }

        candidate.cost_terms = {
            name: float(value)
            for name, value
            in candidate.cost_terms.items()
        }

        if not candidate.rejection_reasons:
            candidate.cost = sum(
                candidate.cost_terms.values()
            )

        return candidate