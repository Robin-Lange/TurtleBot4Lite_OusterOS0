"""Pure systems for passive range-edge observations and their freshness.

Inspired by the visibility/occlusion distinction in OcclusionCBF (Kim et al.,
2026); this is an independent scan heuristic, not its predictor or controller.
Finite range discontinuities are candidate occlusion boundaries only. Missing
returns remain unknown and are never converted into free space or geometry.
"""

import math

from .components import (
    VisibilityConfig, VisibilityEdge, VisibilityHealth, VisibilityObservation,
    VisibilityScan,
)
from .systems import sample_fresh


def scan_visibility_system(scan: VisibilityScan, config: VisibilityConfig) -> VisibilityObservation:
    """Locate nearby finite-return jumps without connecting across missing beams.

    All bearings stay in the scan frame. No map, obstacle identity, velocity,
    traversability, or conservative hidden-region coverage is inferred.
    """
    count = len(scan.ranges)
    metadata = (scan.angle_min, scan.angle_max, scan.angle_increment, scan.range_min, scan.range_max)
    if count < 2 or not all(math.isfinite(value) for value in metadata):
        raise ValueError("Scan needs at least two beams and finite metadata")
    step = abs(scan.angle_increment)
    if step == 0 or not 0 <= scan.range_min < scan.range_max:
        raise ValueError("Invalid scan increment or range limits")
    tolerance = max(1e-6, step * 0.05)
    expected_max = scan.angle_min + (count - 1) * scan.angle_increment
    if abs(scan.angle_max - expected_max) > tolerance:
        raise ValueError("Scan angles do not match beam count")
    span = (count - 1) * step
    if span > math.tau + tolerance:
        raise ValueError("Scan spans more than one revolution")

    # Drivers may include the duplicate endpoint or stop one beam short of it.
    wrap_neighbors = abs(count * step - math.tau) <= tolerance
    full_circle = wrap_neighbors or abs(span - math.tau) <= tolerance
    valid = tuple(
        math.isfinite(value) and value > 0 and scan.range_min <= value <= scan.range_max
        for value in scan.ranges
    )
    edges = {}
    if step <= config.max_neighbor_angle_rad:
        pairs = [(index, index + 1) for index in range(count - 1)]
        if wrap_neighbors and count > 2:
            pairs.append((count - 1, 0))
        for left, right in pairs:
            if not (valid[left] and valid[right]):
                continue
            near, far = sorted((left, right), key=lambda index: scan.ranges[index])
            near_range, far_range = scan.ranges[near], scan.ranges[far]
            if near_range <= config.max_edge_range_m and far_range - near_range >= config.range_jump_m:
                edge = VisibilityEdge(near, scan.angle_min + near * scan.angle_increment, near_range, far_range)
                # A single near beam can border two gaps; retain its largest jump.
                if near not in edges or edges[near].far_range < far_range:
                    edges[near] = edge
    return VisibilityObservation(
        count, sum(valid), full_circle, step <= config.max_neighbor_angle_rad,
        tuple(edges[key] for key in sorted(edges)),
    )


def visibility_health_system(observation, sample, error, config, now, ros_now) -> VisibilityHealth:
    """Report observation quality; an OK result is never a motion permission."""
    if not sample_fresh(sample, now, ros_now, config.timeout_s, config.future_tolerance_s):
        return VisibilityHealth("STALE", "Missing, stale, or future-dated scan")
    if error or observation is None:
        return VisibilityHealth("INVALID", error or "No scan observation")
    fraction = observation.finite_count / observation.beam_count
    if fraction < config.min_finite_fraction:
        return VisibilityHealth("LIMITED", "Insufficient finite returns; visibility remains unknown", True)
    if not observation.resolution_supported:
        return VisibilityHealth("LIMITED", "Angular spacing too coarse for edge detection", True)
    if not observation.full_circle:
        return VisibilityHealth("LIMITED", "Partial angular coverage; unobserved bearings remain unknown", True)
    if observation.edges:
        return VisibilityHealth("EDGES", "Possible occlusion boundaries; passive observation only", True)
    return VisibilityHealth("OBSERVED", "Scan processed; no detected edges does not establish safety", True)
