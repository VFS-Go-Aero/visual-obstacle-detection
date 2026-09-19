#!/usr/bin/env python3

import time
import numpy as np

from .detection_support import (
    required_points, coherent_groups, CandidateConfirmation,
)

import rclpy
from rclpy.logging import LoggingSeverity
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header


# ── sector-map config ─────────────────────────────────────────────────────────
N_AZ = 32    # azimuth bins (360 / 32 = 11.25° each)
N_EL = 8     # elevation bins (180 / 8 = 22.5° each)
DIST_BIN_W = 0.1    # distance shell width (metres)
MIN_POINTS = 100      # required support at distances <= 2 m
MIN_POINTS_FLOOR = 20
SUPPORT_BINS = 3      # combine up to three adjacent 0.1 m bins
# ─────────────────────────────────────────────────────────────────────────────


def build_sector_map(points: np.ndarray,
                     n_az: int = N_AZ,
                     n_el: int = N_EL,
                     dist_bin_w: float = DIST_BIN_W,
                     min_pts: int = MIN_POINTS,
                     nearest_ranges: dict | None = None):
    """
    Build a sector map finding the nearest obstacle in each angular sector.

    Divide the point cloud into angular sectors and, for each sector, find
    the nearest spatially coherent group with distance-adaptive support
    within a sliding window of three adjacent distance bins.

    Returns
    -------
    winner_mask : bool array, shape (N,)
        True for every point that is the reported obstacle representative of
        its sector (a measured point near the center of the first dense bin).

    """
    if points.shape[0] == 0:
        return np.zeros(0, dtype=bool), np.zeros(0, dtype=np.uint32)

    # drop NaN and zero-distance points
    finite_mask = np.isfinite(points).all(axis=1)
    dists = np.linalg.norm(points, axis=1)
    valid = finite_mask & (dists > 0) & (dists <= 5.0)

    if not np.any(valid):
        return np.zeros(len(points), dtype=bool), np.zeros(len(points), dtype=np.uint32)

    dirs = np.zeros_like(points)
    dirs[valid] = points[valid] / dists[valid, np.newaxis]

    az = np.arctan2(dirs[:, 1], dirs[:, 0])            # −π … π
    el = np.arcsin(np.clip(dirs[:, 2], -1.0, 1.0))     # −π/2 … π/2

    az_idx = ((az + np.pi) / (2 * np.pi) * n_az).astype(int) % n_az
    el_idx = ((el + np.pi / 2) / np.pi * n_el).astype(int).clip(0, n_el - 1)

    winner_mask = np.zeros(len(points), dtype=bool)
    winner_sector = np.zeros(len(points), dtype=np.uint32)

    for a in range(n_az):
        for e in range(n_el):
            mask = valid & (az_idx == a) & (el_idx == e)
            if not np.any(mask):
                continue

            sector_dists = dists[mask]
            sector_indices = np.where(mask)[0]

            max_d = sector_dists.max()
            if not np.isfinite(max_d) or max_d <= 0:
                continue
            bin_ids = np.floor(sector_dists / dist_bin_w).astype(np.int64)
            bin_counts = np.bincount(bin_ids)

            # Sliding windows prevent a surface being split at shell boundaries.
            padded = np.pad(bin_counts, (0, SUPPORT_BINS - 1))
            counts = sum(padded[k:k + len(bin_counts)] for k in range(SUPPORT_BINS))
            thresholds = required_points(
                np.arange(len(bin_counts)) * dist_bin_w,
                base=min_pts, floor=min(MIN_POINTS_FLOOR, min_pts),
            )
            for b in np.flatnonzero((counts >= thresholds) & (bin_counts > 0)):
                in_window = (bin_ids >= b) & (bin_ids < b + SUPPORT_BINS)
                window_indices = sector_indices[in_window]
                candidates = []
                for group in coherent_groups(points[window_indices]):
                    indices = window_indices[group]
                    near = float(dists[indices].min())
                    needed = required_points(
                        near, base=min_pts, floor=min(MIN_POINTS_FLOOR, min_pts),
                    )
                    if len(indices) >= needed:
                        candidates.append((near, indices))
                if not candidates:
                    continue
                near, pts_in_bin = min(candidates, key=lambda candidate: candidate[0])
                center = np.median(points[pts_in_bin], axis=0)
                closest = pts_in_bin[np.argmin(
                    np.linalg.norm(points[pts_in_bin] - center, axis=1)
                )]
                winner_mask[closest] = True
                sector_id = a * n_el + e
                winner_sector[closest] = sector_id
                if nearest_ranges is not None:
                    nearest_ranges[sector_id] = near
                break
            # No supported group means unknown, not evidence of free space.

    return winner_mask, winner_sector


class ObstacleDetection(Node):

    def __init__(self) -> None:
        super().__init__("obstacle_detection_segment")
        self._verbose = self.declare_parameter("verbose", True).value

        level = LoggingSeverity.DEBUG if self._verbose else LoggingSeverity.INFO
        self.get_logger().set_level(level)
        self._frame_id = "base_link"
        self.cloud = np.empty((0, 3), dtype=np.float32)
        self._rx_count = 0
        self._empty_parse_count = 0
        self._empty_detect_count = 0
        self._zero_obs_streak = 0
        self._frame_mismatch_count = 0
        self._last_n_obs = None
        self._confirmation = CandidateConfirmation()
        self._health_timer = self.create_timer(5.0, self._health_check)

        # publish only the obstacle-representative points (red)
        self.pub = self.create_publisher(
            PointCloud2,
            "/merged_cloud/obstacles",
            1,
        )

        self._sub_merged = self.create_subscription(
            PointCloud2,
            "/merged_cloud",
            self._cb_merged,
            1,
        )

        self.get_logger().info(
            f"Obstacle detection started  "
            f"[{N_AZ}×{N_EL} sectors, bin_w={DIST_BIN_W}m, "
            f"support_bins={SUPPORT_BINS}, adaptive_points={MIN_POINTS_FLOOR}–{MIN_POINTS}, "
            "confirmation=2 of 3 frames, hold=0.6s]"
        )
        if self._verbose:
            self.get_logger().debug("Logger level forced to DEBUG in code")

    # ── parsing ───────────────────────────────────────────────────────────────

    def _parse(self, msg: PointCloud2) -> np.ndarray:
        try:
            return pc2.read_points_numpy(
                msg, field_names=("x", "y", "z"), skip_nans=True,
            ).reshape(-1, 3).astype(np.float32, copy=False)
        except Exception as exc:
            self.get_logger().error(f"PointCloud parse failed: {exc}")
            return np.empty((0, 3), dtype=np.float32)

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _health_check(self) -> None:
        self.get_logger().debug(
            "DIAG heartbeat "
            f"rx={self._rx_count}, parsed_empty={self._empty_parse_count}, "
            f"detect_empty={self._empty_detect_count}, zero_obs_streak={self._zero_obs_streak}"
        )
        if self._rx_count == 0:
            self.get_logger().warning(
                "No /merged_cloud messages received yet. "
                "Check topic name, publisher state, and QoS compatibility."
            )

    def _cb_merged(self, msg: PointCloud2) -> None:
        self._rx_count += 1
        self.get_logger().debug(
            "DIAG callback "
            f"rx={self._rx_count}, frame={msg.header.frame_id}, size={msg.width}x{msg.height}"
        )
        if self._verbose and (self._rx_count <= 5 or self._rx_count % 30 == 0):
            self.get_logger().debug(
                "RX /merged_cloud "
                f"count={self._rx_count}, frame={msg.header.frame_id}, "
                f"size={msg.width}x{msg.height}, point_step={msg.point_step}, "
                f"row_step={msg.row_step}, is_dense={msg.is_dense}"
            )

        if msg.header.frame_id and msg.header.frame_id != self._frame_id:
            self._frame_mismatch_count += 1
            if self._frame_mismatch_count <= 10 or self._frame_mismatch_count % 20 == 0:
                self.get_logger().warning(
                    "Incoming /merged_cloud frame differs from publish frame "
                    f"(incoming={msg.header.frame_id}, publish={self._frame_id}, "
                    f"count={self._frame_mismatch_count})"
                )

        self.cloud = self._parse(msg)
        if self.cloud.shape[0] == 0:
            self._empty_parse_count += 1
            if self._empty_parse_count <= 10 or self._empty_parse_count % 20 == 0:
                self.get_logger().warning(
                    "Parsed empty cloud from /merged_cloud "
                    f"(count={self._empty_parse_count}, rx_count={self._rx_count})"
                )

        self._detect_and_publish()

    # ── detection + publish ───────────────────────────────────────────────────

    def _detect_and_publish(self) -> None:
        if self.cloud.shape[0] == 0:
            self._empty_detect_count += 1
            if self._empty_detect_count <= 10 or self._empty_detect_count % 20 == 0:
                self.get_logger().warning(
                    "Publishing empty obstacle observation because cloud is empty "
                    f"(count={self._empty_detect_count})"
                )

        nearest_ranges = {}
        winner_mask, winner_sector = build_sector_map(
            self.cloud, nearest_ranges=nearest_ranges,
        )

        obstacle_points = self.cloud[winner_mask]
        obstacle_sectors = winner_sector[winner_mask]

        # Preserve the nearest measured range in each winning bin while using
        # its central representative direction, so stabilization never moves
        # a reported obstacle farther away.
        for i, point in enumerate(obstacle_points):
            point_distance = np.linalg.norm(point)
            obstacle_points[i] *= nearest_ranges[int(obstacle_sectors[i])] / point_distance

        obstacle_points, obstacle_sectors = self._confirmation.stabilize(
            obstacle_points, obstacle_sectors, time.monotonic(),
        )
        n_obs = obstacle_points.shape[0]

        if n_obs == 0:
            self._zero_obs_streak += 1
            if self._zero_obs_streak <= 10 or self._zero_obs_streak % 20 == 0:
                d = np.linalg.norm(self.cloud, axis=1)
                self.get_logger().warning(
                    "No obstacle representatives this frame "
                    f"(streak={self._zero_obs_streak}, pts={self.cloud.shape[0]}, "
                    f"dist_min={np.min(d) if d.size else float('nan'):.3f}, dist_max={np.max(d) if d.size else float('nan'):.3f}, "
                    f"bin_w={DIST_BIN_W}, min_pts={MIN_POINTS}, sectors={N_AZ}x{N_EL})"
                )
        else:
            if self._zero_obs_streak > 0:
                if self._verbose:
                    self.get_logger().info(
                        f"Obstacle detection recovered after {self._zero_obs_streak} empty frames"
                    )
                else:
                    self.get_logger().debug(
                        f"Obstacle detection recovered after {self._zero_obs_streak} empty frames"
                    )
            self._zero_obs_streak = 0

            if self._verbose:
                self.get_logger().info(
                    f"Sector map: {n_obs} obstacle representatives "
                    f"from {self.cloud.shape[0]} total points"
                )
            elif self._last_n_obs is None or self._last_n_obs != n_obs:
                self.get_logger().info(
                    f"Detected {n_obs} obstacle representatives from {self.cloud.shape[0]} points"
                )

        self._last_n_obs = n_obs

        obstacle_points_with_ids = [
            [p[0], p[1], p[2], int(s)] for p, s in zip(obstacle_points, obstacle_sectors)
            ]

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._frame_id

        fields = [
            PointField(
                name='x',
                offset=0,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name='y',
                offset=4,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name='z',
                offset=8,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name='obstacle_id',
                offset=12,
                datatype=PointField.UINT32,
                count=1,
            ),
        ]

        cloud_msg = pc2.create_cloud(header, fields, obstacle_points_with_ids)
        self.pub.publish(cloud_msg)


def main():
    rclpy.init()
    node = ObstacleDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
