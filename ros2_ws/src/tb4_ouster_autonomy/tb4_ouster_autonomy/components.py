"""Data-only ECS components with no ROS dependencies."""

from dataclasses import dataclass, field
import math
from typing import Optional


@dataclass(frozen=True)
class Velocity:
    linear: float = 0.0
    angular: float = 0.0


@dataclass(frozen=True)
class Command:
    velocity: Velocity
    received: float


@dataclass(frozen=True)
class Sample:
    received: float
    stamp: float


@dataclass
class Authority:
    docked: bool = True
    fault: str = ""
    autonomy_paused: bool = False
    hazards: tuple = ()
    deadman: bool = False
    deadman_received: Optional[float] = None


@dataclass
class World:
    """Robot entity plus keyed sensor and command-source entities.

    Callbacks replace components; pure systems read them and return decisions.
    There is no transport or platform SDK in the world.
    """

    authority: Authority = field(default_factory=Authority)
    samples: dict[str, Sample] = field(default_factory=dict)
    commands: dict[str, Command] = field(default_factory=dict)


@dataclass(frozen=True)
class SafetyConfig:
    publish_rate_hz: float = 20.0
    cmd_timeout_s: float = 0.5
    odom_timeout_s: float = 1.0
    tf_timeout_s: float = 1.0
    scan_timeout_s: float = 1.0
    cloud_timeout_s: float = 1.0
    dock_timeout_s: float = 4.0
    hazard_timeout_s: float = 1.5
    future_tolerance_s: float = 0.25
    deadman_timeout_s: float = 0.5
    max_linear_speed: float = 0.15
    max_angular_speed: float = 0.50
    allow_nav2: bool = False
    require_deadman_topic: bool = True

    def __post_init__(self):
        for name, value in vars(self).items():
            if isinstance(value, bool):
                continue
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")


@dataclass(frozen=True)
class Decision:
    state: str
    reason: str
    source: str = "NONE"
    velocity: Velocity = Velocity()


@dataclass(frozen=True)
class VisibilityConfig:
    """Heuristic thresholds for passive scan diagnostics, not control limits."""

    range_jump_m: float = 0.75
    max_edge_range_m: float = 6.0
    max_neighbor_angle_rad: float = 0.05
    min_finite_fraction: float = 0.5
    timeout_s: float = 0.5
    future_tolerance_s: float = 0.1

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if self.min_finite_fraction > 1:
            raise ValueError("min_finite_fraction must be <= 1")


@dataclass(frozen=True)
class VisibilityScan:
    """One planar scan entity, expressed in its acquisition frame."""

    ranges: tuple[float, ...]
    angle_min: float
    angle_max: float
    angle_increment: float
    range_min: float
    range_max: float


@dataclass(frozen=True)
class VisibilityEdge:
    beam_index: int
    angle: float
    near_range: float
    far_range: float


@dataclass(frozen=True)
class VisibilityObservation:
    beam_count: int
    finite_count: int
    full_circle: bool
    resolution_supported: bool
    edges: tuple[VisibilityEdge, ...]


@dataclass(frozen=True)
class VisibilityHealth:
    state: str
    reason: str
    show_edges: bool = False
