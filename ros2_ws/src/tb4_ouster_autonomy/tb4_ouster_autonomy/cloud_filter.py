#!/usr/bin/env python3
"""Point cloud filter node for iRobot Create 3 + Ouster OS0.

Implements Enhanced RANSAC ground plane segmentation based on:
- PMC9862692: A Survey on Ground Segmentation Methods for Automotive LiDAR Sensors
- MengWoods/enhanced-RANSAC-ground-segmentation

Vectorized pipeline:
1. Filters robot self-returns inside the Create 3 + payload physical bounding envelope.
2. Filters out-of-range returns (min_range / max_range).
3. Selects ground candidate seeds near the expected floor level.
4. Generates plane hypotheses with:
   - Normal angle gating (n_z >= cos(theta_max)) to reject vertical walls and boxes.
   - Floor height intercept gating (|d - d_nominal| <= tol) to reject non-floor planes.
5. Inlier counting and SVD/least-squares plane refinement.
6. Temporal exponential moving average (EMA) smoothing to prevent frame-to-frame plane jitter.
7. Partitions point cloud into obstacles (/ouster/cloud_filtered) and ground (/ouster/ground_points).
"""

import math
import time
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


class CloudFilter(Node):
    """Filters robot self-returns, extracts ground via Enhanced RANSAC, and outputs obstacles."""

    def __init__(self):
        super().__init__("cloud_filter")

        # Topics and rates
        self.declare_parameter("input_topic", "/ouster/points")
        self.declare_parameter("output_topic", "/ouster/cloud_filtered")
        self.declare_parameter("ground_topic", "/ouster/ground_points")
        self.declare_parameter("publish_ground_cloud", True)
        self.declare_parameter("target_rate_hz", 10.0)

        # Bounding box of robot envelope to filter self-returns
        # Approx +/-0.22m in X/Y, -0.25m to +0.06m in Z relative to os_lidar/laser_frame
        self.declare_parameter("self_box_min_x", -0.22)
        self.declare_parameter("self_box_max_x", 0.22)
        self.declare_parameter("self_box_min_y", -0.22)
        self.declare_parameter("self_box_max_y", 0.22)
        self.declare_parameter("self_box_min_z", -0.25)
        self.declare_parameter("self_box_max_z", 0.06)

        # Range and height limits relative to laser_frame
        self.declare_parameter("min_range", 0.30)
        self.declare_parameter("max_range", 25.0)
        self.declare_parameter("min_height", -0.12)
        self.declare_parameter("max_height", 1.50)

        # Enhanced RANSAC ground segmentation parameters
        self.declare_parameter("enable_ransac", True)
        self.declare_parameter("ransac_iterations", 60)
        self.declare_parameter("ransac_inlier_thresh", 0.025)
        self.declare_parameter("ransac_max_tilt_deg", 15.0)
        self.declare_parameter("nominal_floor_z", -0.1712)
        self.declare_parameter("floor_z_tolerance", 0.08)
        self.declare_parameter("ground_seed_min_z", -0.25)
        self.declare_parameter("ground_seed_max_z", -0.05)
        self.declare_parameter("ground_obstacle_margin", 0.03)
        self.declare_parameter("underground_noise_margin", 0.05)
        self.declare_parameter("max_obstacle_height_above_ground", 1.80)
        self.declare_parameter("temporal_ema_alpha", 0.80)
        self.declare_parameter("min_ground_inliers", 30)

        # Read parameters
        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.ground_topic = self.get_parameter("ground_topic").value
        self.publish_ground_cloud = self.get_parameter("publish_ground_cloud").value
        self.target_rate_hz = self.get_parameter("target_rate_hz").value
        self.min_period_s = 1.0 / self.target_rate_hz if self.target_rate_hz > 0 else 0.0
        self.last_published_time_s = 0.0

        self.self_box_min_x = self.get_parameter("self_box_min_x").value
        self.self_box_max_x = self.get_parameter("self_box_max_x").value
        self.self_box_min_y = self.get_parameter("self_box_min_y").value
        self.self_box_max_y = self.get_parameter("self_box_max_y").value
        self.self_box_min_z = self.get_parameter("self_box_min_z").value
        self.self_box_max_z = self.get_parameter("self_box_max_z").value

        self.min_range = self.get_parameter("min_range").value
        self.max_range = self.get_parameter("max_range").value
        self.min_height = self.get_parameter("min_height").value
        self.max_height = self.get_parameter("max_height").value

        self.enable_ransac = self.get_parameter("enable_ransac").value
        self.ransac_iterations = int(self.get_parameter("ransac_iterations").value)
        self.ransac_inlier_thresh = self.get_parameter("ransac_inlier_thresh").value
        self.ransac_max_tilt_deg = self.get_parameter("ransac_max_tilt_deg").value
        self.min_cos_tilt = math.cos(math.radians(self.ransac_max_tilt_deg))
        self.nominal_floor_z = self.get_parameter("nominal_floor_z").value
        self.nominal_d = -float(self.nominal_floor_z)
        self.floor_z_tolerance = self.get_parameter("floor_z_tolerance").value
        self.ground_seed_min_z = self.get_parameter("ground_seed_min_z").value
        self.ground_seed_max_z = self.get_parameter("ground_seed_max_z").value
        self.ground_obstacle_margin = self.get_parameter("ground_obstacle_margin").value
        self.underground_noise_margin = self.get_parameter("underground_noise_margin").value
        self.max_obstacle_height_above_ground = self.get_parameter("max_obstacle_height_above_ground").value
        self.temporal_ema_alpha = self.get_parameter("temporal_ema_alpha").value
        self.min_ground_inliers = int(self.get_parameter("min_ground_inliers").value)

        # Smoothed plane state: plane equation n_x*x + n_y*y + n_z*z + d = 0
        # Initialized to perfectly calibrated level floor
        self.prev_normal = np.array([0.0, 0.0, 1.0], dtype=np.float32)
        self.prev_d = float(self.nominal_d)

        # Publishers & Subscriptions
        self.sub_cloud = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cloud_callback,
            qos_profile_sensor_data,
        )

        self.pub_cloud = self.create_publisher(
            PointCloud2,
            self.output_topic,
            10,
        )

        self.pub_ground = (
            self.create_publisher(PointCloud2, self.ground_topic, 10)
            if self.publish_ground_cloud
            else None
        )

        self.get_logger().info(
            f"CloudFilter initialized. Filtering {self.input_topic} -> {self.output_topic} "
            f"at rate <= {self.target_rate_hz} Hz. Enhanced RANSAC: {self.enable_ransac}."
        )

    def fit_ground_plane_ransac(self, seeds: np.ndarray) -> Tuple[np.ndarray, float, bool]:
        """Fit ground plane using Enhanced RANSAC with normal & height gating and SVD refinement.

        Returns:
            Tuple of (normal_vector (3,), plane_d, success_bool)
        """
        n_seeds = len(seeds)
        if n_seeds < self.min_ground_inliers:
            return self.prev_normal.copy(), self.prev_d, False

        # Subsample seeds if pool is large to ensure deterministic <3ms execution
        if n_seeds > 600:
            sub_idx = np.random.choice(n_seeds, 600, replace=False)
            seeds_sample = seeds[sub_idx]
        else:
            seeds_sample = seeds

        sample_len = len(seeds_sample)
        k_iter = min(self.ransac_iterations, sample_len * 2)

        # Vectorized hypothesis generation
        sample_indices = np.random.randint(0, sample_len, size=(k_iter, 3))
        samples = seeds_sample[sample_indices]  # shape: (K, 3, 3)

        v1 = samples[:, 1] - samples[:, 0]
        v2 = samples[:, 2] - samples[:, 0]
        n_hypo = np.cross(v1, v2)
        norms = np.linalg.norm(n_hypo, axis=1, keepdims=True)

        valid_norm = (norms[:, 0] > 1e-6)
        norms[~valid_norm] = 1.0
        n_hypo = n_hypo / norms

        # Ensure normals point upward
        flip = n_hypo[:, 2] < 0
        n_hypo[flip] = -n_hypo[flip]

        # Normal Angle Gating: normal must be roughly upright (reject vertical walls/boxes)
        valid_n = (n_hypo[:, 2] >= self.min_cos_tilt)

        # Intercept calculation: n · p + d = 0  =>  d = -n · p
        d_hypo = -np.sum(n_hypo * samples[:, 0], axis=1)

        # Height Intercept Gating: floor intercept must match sensor optical height within tolerance
        valid_d = (np.abs(d_hypo - self.nominal_d) <= self.floor_z_tolerance)

        valid_mask = valid_norm & valid_n & valid_d
        valid_indices = np.where(valid_mask)[0]

        best_count = 0
        best_n = self.prev_normal.copy()
        best_d = self.prev_d
        best_inliers_mask: Optional[np.ndarray] = None

        if len(valid_indices) > 0:
            valid_n_mat = n_hypo[valid_indices]  # (V, 3)
            valid_d_arr = d_hypo[valid_indices]  # (V,)

            # Vectorized distance computation from all valid hypotheses to seeds
            # dists shape: (V, sample_len)
            dists = np.abs(np.dot(valid_n_mat, seeds_sample.T) + valid_d_arr[:, None])
            inliers_mat = dists <= self.ransac_inlier_thresh
            counts = np.sum(inliers_mat, axis=1)

            best_v_idx = int(np.argmax(counts))
            best_count = int(counts[best_v_idx])
            best_n = valid_n_mat[best_v_idx]
            best_d = float(valid_d_arr[best_v_idx])
            best_inliers_mask = inliers_mat[best_v_idx]

        # SVD / Covariance refinement on inliers
        if best_count >= self.min_ground_inliers and best_inliers_mask is not None:
            inlier_pts = seeds_sample[best_inliers_mask]
            centroid = np.mean(inlier_pts, axis=0)
            centered = inlier_pts - centroid
            cov = np.dot(centered.T, centered)
            _, eigenvectors = np.linalg.eigh(cov)
            n_opt = eigenvectors[:, 0]
            if n_opt[2] < 0:
                n_opt = -n_opt
            d_opt = -float(np.dot(n_opt, centroid))
            return n_opt.astype(np.float32), d_opt, True

        return best_n, best_d, False

    def cloud_callback(self, in_msg: PointCloud2):
        now_s = time.monotonic()
        if (now_s - self.last_published_time_s) < (self.min_period_s * 0.9):
            # Downsample to target rate to protect Jetson CPU
            return

        try:
            field_names = [f.name for f in in_msg.fields]
            if "x" not in field_names or "y" not in field_names or "z" not in field_names:
                return

            points_gen = pc2.read_points_numpy(in_msg, field_names=["x", "y", "z"])
            if points_gen.size == 0:
                return

            x = points_gen[:, 0]
            y = points_gen[:, 1]
            z = points_gen[:, 2]

            # 1. Range filter
            dist_sq = x * x + y * y + z * z
            valid_mask = (dist_sq >= (self.min_range * self.min_range)) & (
                dist_sq <= (self.max_range * self.max_range)
            )

            # 2. Self-filter: remove points inside the robot bounding envelope
            in_self_box = (
                (x >= self.self_box_min_x)
                & (x <= self.self_box_max_x)
                & (y >= self.self_box_min_y)
                & (y <= self.self_box_max_y)
                & (z >= self.self_box_min_z)
                & (z <= self.self_box_max_z)
            )
            valid_mask &= ~in_self_box

            valid_points = points_gen[valid_mask]
            if valid_points.size == 0:
                return

            if self.enable_ransac:
                # 3. Enhanced RANSAC ground segmentation
                cand_z = valid_points[:, 2]
                seed_mask = (cand_z >= self.ground_seed_min_z) & (cand_z <= self.ground_seed_max_z)
                seeds = valid_points[seed_mask]

                fitted_normal, fitted_d, success = self.fit_ground_plane_ransac(seeds)

                # 4. Temporal Exponential Moving Average (EMA) smoothing
                if success:
                    alpha = self.temporal_ema_alpha
                    smoothed_normal = alpha * fitted_normal + (1.0 - alpha) * self.prev_normal
                    normal_norm = np.linalg.norm(smoothed_normal)
                    if normal_norm > 1e-6:
                        smoothed_normal = smoothed_normal / normal_norm
                    smoothed_d = alpha * fitted_d + (1.0 - alpha) * self.prev_d
                    self.prev_normal = smoothed_normal.astype(np.float32)
                    self.prev_d = float(smoothed_d)
                # If unsuccessful, preserve self.prev_normal and self.prev_d

                # 5. Partition points by height relative to estimated ground plane
                # Signed height h = n · p + d
                h = np.dot(valid_points, self.prev_normal) + self.prev_d

                # Obstacles: points strictly above ground margin and below ceiling
                obs_mask = (h > self.ground_obstacle_margin) & (
                    h <= self.max_obstacle_height_above_ground
                )
                filtered_obstacles = valid_points[obs_mask]

                # Ground: points within ground tolerance band
                ground_mask = (h >= -self.underground_noise_margin) & (
                    h <= self.ground_obstacle_margin
                )
                ground_points = valid_points[ground_mask]

                # Publish obstacle point cloud
                out_msg = pc2.create_cloud_xyz32(in_msg.header, filtered_obstacles)
                self.pub_cloud.publish(out_msg)

                # Publish ground point cloud for inspection/visualization
                if self.pub_ground is not None:
                    ground_msg = pc2.create_cloud_xyz32(in_msg.header, ground_points)
                    self.pub_ground.publish(ground_msg)

            else:
                # Fallback: legacy fixed-height filtering
                z_valid = valid_points[:, 2]
                legacy_mask = (z_valid >= self.min_height) & (z_valid <= self.max_height)
                filtered_points = valid_points[legacy_mask]

                out_msg = pc2.create_cloud_xyz32(in_msg.header, filtered_points)
                self.pub_cloud.publish(out_msg)

            self.last_published_time_s = now_s

        except Exception as exc:
            self.get_logger().error(f"Error in cloud_callback: {exc}", throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = CloudFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
