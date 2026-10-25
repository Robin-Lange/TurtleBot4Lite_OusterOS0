"""Pure systems over robot, sensor, and command-source components."""

import math

import numpy as np

from .components import Decision, SafetyConfig, Sample, Velocity, World


def sample_fresh(sample: Sample, now: float, ros_now: float, timeout: float, future: float) -> bool:
    """Check receipt and acquisition time; replaying an old stamp is not health."""
    return (
        sample is not None
        and all(math.isfinite(v) for v in (sample.received, sample.stamp, now, ros_now))
        and sample.stamp > 0
        and 0 <= now - sample.received <= timeout
        and -future <= ros_now - sample.stamp <= timeout
    )


def telemetry_problem(world: World, config: SafetyConfig, now: float, ros_now: float) -> str:
    for entity in ("dock", "hazard", "odom", "tf", "scan", "cloud"):
        if not sample_fresh(
            world.samples.get(entity), now, ros_now,
            getattr(config, f"{entity}_timeout_s"), config.future_tolerance_s,
        ):
            return f"Missing, stale, or future-dated {entity}"
    return ""


def safety_system(world: World, config: SafetyConfig, now: float, ros_now: float) -> Decision:
    """Resolve authority before passing a bounded proposal to Collision Monitor.

    Contact is a latched stop, never an implicit reverse/rotate maneuver.
    Create 3's independent hardware reflexes remain enabled.
    """
    authority = world.authority
    if authority.fault:
        return Decision("LATCHED_FAULT", authority.fault)
    if authority.docked:
        return Decision("INHIBITED_DOCKED", "Robot is docked or dock state is unknown")
    problem = telemetry_problem(world, config, now, ros_now)
    if problem:
        return Decision("INHIBITED_STALE_TELEMETRY", problem)
    for source in ("TELEOP", "EXPLORER", "NAV2"):
        if source != "TELEOP" and (not config.allow_nav2 or authority.autonomy_paused):
            continue
        command = world.commands.get(source)
        if command is None or not 0 <= now - command.received <= config.cmd_timeout_s:
            continue
        if source == "TELEOP" and config.require_deadman_topic:
            if (
                not authority.deadman or authority.deadman_received is None
                or not 0 <= now - authority.deadman_received <= config.deadman_timeout_s
            ):
                return Decision("INHIBITED_DEADMAN", "Teleop deadman released or expired", source)
        velocity = command.velocity
        if not all(math.isfinite(v) for v in (velocity.linear, velocity.angular)):
            return Decision("INVALID_COMMAND", "Non-finite velocity", source)
        if 0 in authority.hazards and velocity.linear < 0:
            return Decision("INHIBITED_BACKUP_LIMIT", "Create 3 backup limit", source)
        bounded = Velocity(
            max(-config.max_linear_speed, min(config.max_linear_speed, velocity.linear)),
            max(-config.max_angular_speed, min(config.max_angular_speed, velocity.angular)),
        )
        return Decision("ACTIVE", f"Forwarding {source}", source, bounded)
    return Decision("IDLE", "No fresh authorized proposal")


def transform_points(points, translation, quaternion):
    """Apply target<-source rigid transform to N x 3 row vectors (xyzw quaternion)."""
    q = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(q).all() or norm < 1e-12:
        raise ValueError("Invalid transform quaternion")
    x, y, z, w = q / norm
    rotation = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])
    translation = np.asarray(translation, dtype=float)
    if not np.isfinite(translation).all():
        raise ValueError("Non-finite translation")
    return np.asarray(points, dtype=np.float32).reshape(-1, 3) @ rotation.T + translation


def obstacle_filter_system(points, min_range, max_range, min_height, max_height, box_min, box_max):
    """Crop in base_link (REP-103), where Z is height; preserve empty observations."""
    points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    finite = np.isfinite(points).all(axis=1)
    ranges = np.linalg.norm(points[:, :2], axis=1)
    inside = ((points >= box_min) & (points <= box_max)).all(axis=1)
    keep = (
        finite & ~inside & (ranges >= min_range) & (ranges <= max_range)
        & (points[:, 2] >= min_height) & (points[:, 2] <= max_height)
    )
    return points[keep]


def contact_points_system(angles, radius, pose):
    """Anchor contact points to odom at acquisition, so they do not follow the robot."""
    x, y, yaw = pose
    return tuple(
        (x + radius * math.cos(a + yaw), y + radius * math.sin(a + yaw), z)
        for a in angles for z in (0.04, 0.12)
    )
