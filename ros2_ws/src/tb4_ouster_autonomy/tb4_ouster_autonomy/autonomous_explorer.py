#!/usr/bin/env python3
"""Autonomous Frontier Explorer and Automatic Map Saver for Create 3 + RPLIDAR S2 + RealSense D455.

Features:
- Automatically undocks Create 3 if starting on the charging dock (configurable).
- Performs an initial 360-degree scan spin to seed SLAM Toolbox and 3D Voxel costmaps.
- Extracts frontier clusters (boundaries between known free space and unknown space) from /map.
- Scores frontiers by cluster size and Euclidean distance, filtering out obstacles and blacklisted goals.
- Sends NavigateToPose action goals to Nav2 (with reactive LiDAR/bumper safety fallback).
- Continuously checkpoints the map every N seconds and deterministically saves `<map_prefix>.pgm`
  and `<map_prefix>.yaml` on exploration completion OR when the user aborts via Ctrl+C / /explore/stop.
"""

from collections import deque
import math
import os
import signal
import time
from typing import List, Optional, Tuple

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from irobot_create_msgs.action import Undock
from irobot_create_msgs.msg import DockStatus, HazardDetectionVector
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger


def quaternion_to_yaw(q) -> float:
    """Convert geometry_msgs/Quaternion to yaw (radians)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """Convert yaw (radians) to (x, y, z, w) quaternion tuple."""
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def save_occupancy_grid_to_files(grid: OccupancyGrid, map_prefix: str) -> Tuple[str, str]:
    """Save OccupancyGrid directly to standard Nav2 trinary .pgm and .yaml files.

    Deterministic, zero-dependency, and instantaneous (<5ms) so Ctrl+C signal
    handlers always persist the latest SLAM map even during node teardown.
    """
    width = int(grid.info.width)
    height = int(grid.info.height)
    resolution = float(grid.info.resolution)
    origin_x = float(grid.info.origin.position.x)
    origin_y = float(grid.info.origin.position.y)
    origin_yaw = quaternion_to_yaw(grid.info.origin.orientation)

    parent_dir = os.path.dirname(os.path.abspath(map_prefix))
    if parent_dir:
        os.makedirs(parent_dir, exist_ok=True)

    pgm_path = f"{map_prefix}.pgm"
    yaml_path = f"{map_prefix}.yaml"
    pgm_basename = os.path.basename(pgm_path)

    data = grid.data
    pixel_bytes = bytearray(width * height)

    # OccupancyGrid row 0 is bottom (origin_y); PGM row 0 is top
    for row in range(height):
        src_row = height - 1 - row
        src_offset = src_row * width
        dst_offset = row * width
        for col in range(width):
            val = data[src_offset + col]
            if val < 0:
                pixel_bytes[dst_offset + col] = 205  # Unknown (gray)
            elif val >= 65:
                pixel_bytes[dst_offset + col] = 0    # Occupied (black)
            elif val <= 25:
                pixel_bytes[dst_offset + col] = 254  # Free (white)
            else:
                pixel_bytes[dst_offset + col] = 205

    with open(pgm_path, "wb") as f:
        header = f"P5\n{width} {height}\n255\n".encode("ascii")
        f.write(header)
        f.write(pixel_bytes)

    yaml_content = (
        f"image: {pgm_basename}\n"
        f"mode: trinary\n"
        f"resolution: {resolution:.4f}\n"
        f"origin: [{origin_x:.3f}, {origin_y:.3f}, {origin_yaw:.3f}]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.25\n"
    )
    with open(yaml_path, "w", encoding="utf-8") as f:
        f.write(yaml_content)

    # Also sync to install share directory if saving to the repo root arena_map
    install_maps_dir = os.path.join(
        parent_dir,
        "ros2_ws",
        "install",
        "tb4_ouster_autonomy",
        "share",
        "tb4_ouster_autonomy",
        "maps",
    )
    if os.path.isdir(install_maps_dir) and os.path.basename(map_prefix) == "arena_map":
        try:
            with open(os.path.join(install_maps_dir, pgm_basename), "wb") as f_inst:
                f_inst.write(header)
                f_inst.write(pixel_bytes)
            with open(
                os.path.join(install_maps_dir, "arena_map.yaml"), "w", encoding="utf-8"
            ) as f_yaml_inst:
                f_yaml_inst.write(yaml_content)
        except OSError:
            pass

    return pgm_path, yaml_path


def find_frontier_clusters(
    grid: OccupancyGrid,
    min_cluster_size: int = 6,
    clearance_cells: int = 5,
) -> List[Tuple[float, float, int]]:
    """Extract frontier cluster centroids (x_m, y_m, cell_count) from an OccupancyGrid.

    A frontier cell is a known free cell (0 <= v <= 25) that has at least one
    4-connected unknown neighbor (v == -1) and no occupied cell (v >= 50)
    within `clearance_cells` radius.
    """
    width = int(grid.info.width)
    height = int(grid.info.height)
    res = float(grid.info.resolution)
    ox = float(grid.info.origin.position.x)
    oy = float(grid.info.origin.position.y)
    data = grid.data

    if width <= 2 or height <= 2 or len(data) < width * height:
        return []

    # 1. Identify candidate frontier cells
    is_frontier = bytearray(width * height)
    for r in range(1, height - 1):
        row_off = r * width
        for c in range(1, width - 1):
            idx = row_off + c
            val = data[idx]
            if 0 <= val <= 25:
                # Check 4-neighbors for unknown (-1)
                if (
                    data[idx - 1] == -1
                    or data[idx + 1] == -1
                    or data[idx - width] == -1
                    or data[idx + width] == -1
                ):
                    is_frontier[idx] = 1

    # 2. Filter out frontier cells too close to obstacles
    if clearance_cells > 0:
        for r in range(height):
            row_off = r * width
            for c in range(width):
                if data[row_off + c] >= 50:
                    r_min = max(0, r - clearance_cells)
                    r_max = min(height - 1, r + clearance_cells)
                    c_min = max(0, c - clearance_cells)
                    c_max = min(width - 1, c + clearance_cells)
                    for nr in range(r_min, r_max + 1):
                        noff = nr * width
                        for nc in range(c_min, c_max + 1):
                            is_frontier[noff + nc] = 0

    # 3. 8-connected BFS clustering
    visited = bytearray(width * height)
    clusters: List[Tuple[float, float, int]] = []

    for r in range(1, height - 1):
        row_off = r * width
        for c in range(1, width - 1):
            start_idx = row_off + c
            if is_frontier[start_idx] and not visited[start_idx]:
                queue = deque([(r, c)])
                visited[start_idx] = 1
                cells = []
                while queue:
                    cr, cc = queue.popleft()
                    cells.append((cr, cc))
                    for dr in (-1, 0, 1):
                        for dc in (-1, 0, 1):
                            if dr == 0 and dc == 0:
                                continue
                            nr, nc = cr + dr, cc + dc
                            if 0 <= nr < height and 0 <= nc < width:
                                nidx = nr * width + nc
                                if is_frontier[nidx] and not visited[nidx]:
                                    visited[nidx] = 1
                                    queue.append((nr, nc))

                if len(cells) >= min_cluster_size:
                    # Pick the cluster cell closest to the cluster median (ensures goal lies on a valid free cell)
                    mean_r = sum(p[0] for p in cells) / len(cells)
                    mean_c = sum(p[1] for p in cells) / len(cells)
                    best_r, best_c = min(
                        cells,
                        key=lambda p: (p[0] - mean_r) ** 2 + (p[1] - mean_c) ** 2,
                    )
                    wx = ox + (best_c + 0.5) * res
                    wy = oy + (best_r + 0.5) * res
                    clusters.append((wx, wy, len(cells)))

    return clusters


class AutonomousExplorer(Node):
    """Autonomous SLAM frontier exploration and automatic map persistence node."""

    def __init__(self):
        super().__init__("autonomous_explorer")

        default_map_prefix = os.path.expanduser(
            "~/TurtleBot4Lite_OusterOS0/arena_map"
        )
        self.declare_parameter("map_output_prefix", default_map_prefix)
        self.declare_parameter("auto_undock", True)
        self.declare_parameter("auto_save_interval_s", 20.0)
        self.declare_parameter("goal_timeout_s", 35.0)
        self.declare_parameter("min_frontier_size", 6)
        self.declare_parameter("obstacle_clearance_m", 0.25)
        self.declare_parameter("blacklist_radius_m", 0.45)
        self.declare_parameter("max_explore_Duration_s", 900.0)
        self.declare_parameter("initial_spin_duration_s", 8.0)
        self.declare_parameter("reactive_fallback_enabled", True)

        self.map_output_prefix = str(self.get_parameter("map_output_prefix").value)
        self.auto_undock = bool(self.get_parameter("auto_undock").value)
        self.auto_save_interval_s = float(self.get_parameter("auto_save_interval_s").value)
        self.goal_timeout_s = float(self.get_parameter("goal_timeout_s").value)
        self.min_frontier_size = int(self.get_parameter("min_frontier_size").value)
        self.obstacle_clearance_m = float(self.get_parameter("obstacle_clearance_m").value)
        self.blacklist_radius_m = float(self.get_parameter("blacklist_radius_m").value)
        self.max_explore_duration_s = float(self.get_parameter("max_explore_Duration_s").value)
        self.initial_spin_duration_s = float(self.get_parameter("initial_spin_duration_s").value)
        self.reactive_fallback_enabled = bool(
            self.get_parameter("reactive_fallback_enabled").value
        )

        # State
        self.latest_map: Optional[OccupancyGrid] = None
        self.robot_x: float = 0.0
        self.robot_y: float = 0.0
        self.robot_yaw: float = 0.0
        self.odom_received: bool = False
        self.scan_received: bool = False
        self.min_front_scan_m: float = 10.0
        self.min_left_scan_m: float = 10.0
        self.min_right_scan_m: float = 10.0
        self.is_docked: Optional[bool] = None
        self.bumper_hit: bool = False
        self.last_bumper_time: float = 0.0

        self.phase: str = "WAIT_SENSORS"
        self.start_time: float = time.monotonic()
        self.phase_start_time: float = time.monotonic()
        self.last_save_time: float = 0.0
        self.exploration_finished: bool = False
        self.map_saved_on_exit: bool = False

        self.blacklisted_goals: List[Tuple[float, float]] = []
        self.current_goal: Optional[Tuple[float, float]] = None
        self.goal_sent_time: float = 0.0
        self.last_nav_reject_time: float = 0.0
        self.blacklist_reset_count: int = 0
        self.nav_goal_active: bool = False
        self.nav_goal_handle = None
        self.undock_requested: bool = False
        self.no_frontier_count: int = 0

        # QoS for /map (transient local or volatile)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.sub_map = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, map_qos
        )
        self.sub_map_volatile = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10
        )
        self.sub_odom = self.create_subscription(
            Odometry, "/odom", self.odom_callback, qos_profile_sensor_data
        )
        self.sub_scan = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data
        )
        self.sub_dock = self.create_subscription(
            DockStatus, "/dock_status", self.dock_callback, qos_profile_sensor_data
        )
        self.sub_hazard = self.create_subscription(
            HazardDetectionVector,
            "/hazard_detection",
            self.hazard_callback,
            qos_profile_sensor_data,
        )

        self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.undock_client = ActionClient(self, Undock, "/undock")

        self.srv_save = self.create_service(
            Trigger, "/explore/save_map", self.handle_save_map_service
        )
        self.srv_stop = self.create_service(
            Trigger, "/explore/stop", self.handle_stop_service
        )

        self.timer = self.create_timer(0.25, self.control_step)
        self.get_logger().info(
            f"AutonomousExplorer initialized. Maps will be saved to: {self.map_output_prefix}.[pgm|yaml]"
        )

    def map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg

    def odom_callback(self, msg: Odometry):
        self.robot_x = float(msg.pose.pose.position.x)
        self.robot_y = float(msg.pose.pose.position.y)
        self.robot_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.odom_received = True

    def dock_callback(self, msg: DockStatus):
        self.is_docked = bool(msg.is_docked)

    def hazard_callback(self, msg: HazardDetectionVector):
        for d in msg.detections:
            if d.type in (1, 2, 3, 4):  # BUMP, CLIFF, STALL, WHEEL_DROP
                self.bumper_hit = True
                self.last_bumper_time = time.monotonic()

    def scan_callback(self, msg: LaserScan):
        self.scan_received = True
        n = len(msg.ranges)
        if n == 0:
            return
        front_dists = []
        left_dists = []
        right_dists = []
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max and math.isfinite(r):
                # Normalize angle to [-pi, pi]
                a = math.atan2(math.sin(angle), math.cos(angle))
                if -0.55 <= a <= 0.55:
                    front_dists.append(r)
                elif 0.55 < a <= 1.6:
                    left_dists.append(r)
                elif -1.6 <= a < -0.55:
                    right_dists.append(r)
            angle += msg.angle_increment

        self.min_front_scan_m = min(front_dists) if front_dists else 10.0
        self.min_left_scan_m = min(left_dists) if left_dists else 10.0
        self.min_right_scan_m = min(right_dists) if right_dists else 10.0

    def stop_robot(self):
        """Publish zero velocity and cancel any active Nav2 goal."""
        if self.nav_goal_handle is not None:
            try:
                self.nav_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.nav_goal_handle = None
        self.nav_goal_active = False
        zero = Twist()
        self.pub_cmd_vel.publish(zero)

    def save_current_map(self, reason: str = "checkpoint") -> bool:
        """Save latest OccupancyGrid to disk."""
        if self.latest_map is None:
            self.get_logger().warn(f"Cannot save map ({reason}): no /map received yet.")
            return False
        try:
            pgm_path, yaml_path = save_occupancy_grid_to_files(
                self.latest_map, self.map_output_prefix
            )
            self.last_save_time = time.monotonic()
            self.get_logger().info(
                f"[MAP SAVED - {reason}] -> {yaml_path} & {pgm_path} "
                f"({self.latest_map.info.width}x{self.latest_map.info.height} @ "
                f"{self.latest_map.info.resolution:.2f}m)"
            )
            return True
        except Exception as exc:
            self.get_logger().error(f"Failed to save map ({reason}): {exc}")
            return False

    def handle_save_map_service(self, request, response):
        ok = self.save_current_map(reason="manual_service")
        response.success = ok
        response.message = (
            f"Map saved to {self.map_output_prefix}.yaml"
            if ok
            else "No map available yet"
        )
        return response

    def handle_stop_service(self, request, response):
        self.get_logger().info("Stop requested via /explore/stop service.")
        self.finish_exploration("user_stop_service")
        response.success = True
        response.message = f"Exploration stopped and map saved to {self.map_output_prefix}.yaml"
        return response

    def finish_exploration(self, reason: str):
        if self.exploration_finished:
            return
        self.exploration_finished = True
        self.phase = "FINISHED"
        self.stop_robot()
        self.save_current_map(reason=reason)
        self.map_saved_on_exit = True
        self.get_logger().info(
            f"Autonomous mapping complete ({reason}). Final map saved at {self.map_output_prefix}.yaml"
        )

    def is_blacklisted(self, x: float, y: float) -> bool:
        for bx, by in self.blacklisted_goals:
            if math.hypot(x - bx, y - by) < self.blacklist_radius_m:
                return True
        return False

    def select_next_frontier(self) -> Optional[Tuple[float, float]]:
        if self.latest_map is None:
            return None

        res = float(self.latest_map.info.resolution)
        clearance_cells = max(1, int(round(self.obstacle_clearance_m / res)))
        clusters = find_frontier_clusters(
            self.latest_map,
            min_cluster_size=self.min_frontier_size,
            clearance_cells=clearance_cells,
        )

        candidates = []
        for fx, fy, size in clusters:
            dist = math.hypot(fx - self.robot_x, fy - self.robot_y)
            if dist < 0.35:
                continue
            if self.is_blacklisted(fx, fy):
                continue
            # Score balances proximity and frontier size
            score = dist - 0.04 * min(size, 50)
            candidates.append((score, fx, fy, size, dist))

        if not candidates:
            return None

        candidates.sort(key=lambda item: item[0])
        _, best_x, best_y, best_size, best_dist = candidates[0]
        self.get_logger().info(
            f"Selected frontier at ({best_x:.2f}, {best_y:.2f}), "
            f"dist={best_dist:.2f}m, size={best_size} cells (from {len(candidates)} candidates)"
        )
        return (best_x, best_y)

    def send_nav2_goal(self, gx: float, gy: float) -> bool:
        if not self.nav_client.wait_for_server(timeout_sec=0.5):
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = gx
        goal_msg.pose.pose.position.y = gy
        goal_msg.pose.pose.position.z = 0.0

        yaw = math.atan2(gy - self.robot_y, gx - self.robot_x)
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.current_goal = (gx, gy)
        self.goal_sent_time = time.monotonic()
        self.nav_goal_active = True

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)
        return True

    def _goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn(
                    "Nav2 bt_navigator not ready yet (goal rejected); using reactive LiDAR exploration for 3s."
                )
                self.last_nav_reject_time = time.monotonic()
                self.nav_goal_active = False
                self.nav_goal_handle = None
                if self.reactive_fallback_enabled:
                    self.reactive_explore_step()
                return
            self.nav_goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._goal_result_callback)
        except Exception as exc:
            self.get_logger().warn(f"Nav2 goal response error: {exc}")
            self.last_nav_reject_time = time.monotonic()
            self.nav_goal_active = False

    def _goal_result_callback(self, future):
        try:
            status = future.result().status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("Reached frontier goal!")
            else:
                self.get_logger().info(
                    f"Frontier goal ended with status {status}; blacklisting region."
                )
            if self.current_goal is not None:
                self.blacklisted_goals.append(self.current_goal)
        except Exception:
            pass
        finally:
            self.nav_goal_active = False
            self.nav_goal_handle = None

    def reactive_explore_step(self):
        """Safe LiDAR-guided reactive exploration step when Nav2 is warming up or between goals."""
        twist = Twist()
        now = time.monotonic()

        # If bumper recently triggered, back up gently and turn
        if self.bumper_hit and (now - self.last_bumper_time) < 2.0:
            if (now - self.last_bumper_time) < 0.8:
                twist.linear.x = -0.06
                twist.angular.z = 0.0
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.40 if self.min_left_scan_m >= self.min_right_scan_m else -0.40
            self.pub_cmd_vel.publish(twist)
            return
        self.bumper_hit = False

        if self.min_front_scan_m < 0.45:
            # Obstacle ahead: rotate toward more open side
            twist.linear.x = 0.0
            twist.angular.z = 0.40 if self.min_left_scan_m >= self.min_right_scan_m else -0.40
        elif self.min_front_scan_m < 0.75:
            twist.linear.x = 0.07
            twist.angular.z = 0.25 if self.min_left_scan_m >= self.min_right_scan_m else -0.25
        else:
            twist.linear.x = 0.12
            twist.angular.z = 0.0

        self.pub_cmd_vel.publish(twist)

    def control_step(self):
        if self.exploration_finished:
            return

        now = time.monotonic()

        # Periodic map checkpoint save
        if (
            self.latest_map is not None
            and (now - self.last_save_time) >= self.auto_save_interval_s
        ):
            self.save_current_map(reason="periodic_checkpoint")

        # Global exploration timeout check
        if (now - self.start_time) >= self.max_explore_duration_s:
            self.finish_exploration("max_duration_reached")
            return

        # Phase 1: Wait for sensors (/scan and /odom, plus /dock_status if auto_undock enabled)
        if self.phase == "WAIT_SENSORS":
            dock_ready = (not self.auto_undock) or (self.is_docked is not None) or ((now - self.phase_start_time) > 3.5)
            if self.scan_received and self.odom_received and dock_ready:
                if self.auto_undock and (self.is_docked is True or self.is_docked is None):
                    self.phase = "UNDOCKING"
                    self.phase_start_time = now
                    self.get_logger().info(
                        f"Sensors online (is_docked={self.is_docked}). Initiating automatic undock..."
                    )
                else:
                    self.phase = "INITIAL_SPIN"
                    self.phase_start_time = now
                    self.get_logger().info(
                        "Sensors online (already undocked). Performing initial 360° scan spin to seed SLAM map..."
                    )
            elif int((now - self.phase_start_time) * 4) % 12 == 0:
                self.get_logger().info(
                    f"Waiting for sensors: scan={self.scan_received}, odom={self.odom_received}, is_docked={self.is_docked}"
                )
            return

        # Phase 2: Auto-Undock if starting on dock
        if self.phase == "UNDOCKING":
            if self.is_docked is False:
                self.get_logger().info("Undock complete! Transitioning to initial SLAM spin.")
                self.phase = "INITIAL_SPIN"
                self.phase_start_time = now
                return

            if not self.undock_requested:
                if self.undock_client.wait_for_server(timeout_sec=1.0):
                    self.undock_client.send_goal_async(Undock.Goal())
                    self.undock_requested = True
                    self.get_logger().info("Sent /undock action goal to Create 3.")
                elif (now - self.phase_start_time) > 5.0:
                    self.get_logger().warn(
                        "/undock action server unavailable; proceeding to INITIAL_SPIN."
                    )
                    self.phase = "INITIAL_SPIN"
                    self.phase_start_time = now
            elif (now - self.phase_start_time) > 20.0:
                self.phase = "INITIAL_SPIN"
                self.phase_start_time = now
            return

        # Phase 3: Initial spin to populate SLAM map around starting pose
        if self.phase == "INITIAL_SPIN":
            if (now - self.phase_start_time) < self.initial_spin_duration_s:
                spin_cmd = Twist()
                spin_cmd.angular.z = 0.40
                self.pub_cmd_vel.publish(spin_cmd)
                return
            else:
                self.stop_robot()
                self.save_current_map(reason="initial_scan")
                self.phase = "EXPLORING"
                self.phase_start_time = now
                self.get_logger().info("Initial SLAM spin complete. Starting frontier exploration!")
                return

        # Phase 4: Active Frontier Exploration
        if self.phase == "EXPLORING":
            if self.latest_map is None:
                return

            # Check active Nav2 goal timeout or proximity completion
            if self.nav_goal_active and self.current_goal is not None:
                dist_to_goal = math.hypot(
                    self.current_goal[0] - self.robot_x,
                    self.current_goal[1] - self.robot_y,
                )
                if dist_to_goal < 0.30:
                    self.get_logger().info("Within 0.30m of frontier goal; advancing to next frontier.")
                    self.blacklisted_goals.append(self.current_goal)
                    self.stop_robot()
                elif (now - self.goal_sent_time) > self.goal_timeout_s:
                    self.get_logger().info("Frontier goal timed out; blacklisting and replanning.")
                    self.blacklisted_goals.append(self.current_goal)
                    self.stop_robot()
                else:
                    return

            # If Nav2 rejected a goal recently (bt_navigator still activating), drive reactively
            if (now - self.last_nav_reject_time) < 3.0:
                if self.reactive_fallback_enabled:
                    self.reactive_explore_step()
                return

            # Pick next frontier
            next_goal = self.select_next_frontier()
            if next_goal is not None:
                self.no_frontier_count = 0
                sent = self.send_nav2_goal(next_goal[0], next_goal[1])
                if not sent and self.reactive_fallback_enabled:
                    self.reactive_explore_step()
            else:
                if self.blacklisted_goals and self.blacklist_reset_count < 2:
                    self.blacklist_reset_count += 1
                    self.get_logger().info(
                        f"Clearing frontier blacklist (pass {self.blacklist_reset_count}/2) to re-check remaining frontiers."
                    )
                    self.blacklisted_goals.clear()
                    if self.reactive_fallback_enabled:
                        self.reactive_explore_step()
                    return
                self.no_frontier_count += 1
                if self.no_frontier_count >= 12:  # 3 seconds of zero remaining frontiers
                    self.finish_exploration("all_frontiers_explored")
                elif self.reactive_fallback_enabled:
                    # Gentle rotation to verify no missed frontiers
                    spin = Twist()
                    spin.angular.z = 0.35
                    self.pub_cmd_vel.publish(spin)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousExplorer()

    def _sig_term_handler(signum, frame):
        raise KeyboardInterrupt()

    signal.signal(signal.SIGTERM, _sig_term_handler)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if not node.map_saved_on_exit:
            if node.latest_map is not None:
                try:
                    save_occupancy_grid_to_files(node.latest_map, node.map_output_prefix)
                    node.map_saved_on_exit = True
                except Exception:
                    pass
        try:
            if rclpy.ok():
                node.stop_robot()
                node.destroy_node()
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
