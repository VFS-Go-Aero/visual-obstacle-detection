#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.logging import LoggingSeverity
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header


# ── sector-map config ─────────────────────────────────────────────────────────
N_AZ = 8     # azimuth bins   (360 / 8 = 45° each)
N_EL = 8     # elevation bins (180 / 8 = 45° each)
DIST_BIN_W = 0.1    # distance shell width (metres)
MIN_POINTS = 100      # min points in a shell to count as a real obstacle
# ─────────────────────────────────────────────────────────────────────────────


def build_sector_map(points: np.ndarray,
                     n_az: int = N_AZ,
                     n_el: int = N_EL,
                     dist_bin_w: float = DIST_BIN_W,
                     min_pts: int = MIN_POINTS):
    """
    Build a sector map finding the nearest obstacle in each angular sector.

    Divide the point cloud into angular sectors and, for each sector, find
    the near edge of the first distance-bin that contains >= min_pts points.

    Returns
    -------
    winner_mask : bool array, shape (N,)
        True for every point that is the reported obstacle representative of
        its sector (i.e. the closest point inside the first dense bin).

    """
    if points.shape[0] == 0:
        return np.zeros(0, dtype=bool), np.zeros(0, dtype=np.uint32)

    # drop NaN and zero-distance points
    finite_mask = np.isfinite(points).all(axis=1)
    dists = np.linalg.norm(points, axis=1)
    valid = finite_mask & (dists > 0)

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
            bin_edges = np.arange(0, max_d + dist_bin_w, dist_bin_w)
            bin_ids = np.digitize(sector_dists, bin_edges) - 1   # 0-indexed

            # walk near → far; first bin with enough points wins
            for b in range(len(bin_edges) - 1):
                in_bin = bin_ids == b
                if in_bin.sum() >= min_pts:
                    pts_in_bin = sector_indices[in_bin]
                    closest = pts_in_bin[np.argmin(sector_dists[in_bin])]
                    winner_mask[closest] = True
                    sector_id = a * n_el + e
                    winner_sector[closest] = sector_id
                    break
            # no bin reached threshold → sector is clear, no winner

    return winner_mask, winner_sector


class ObstacleDetection(Node):

    def __init__(self) -> None:
        super().__init__("obstacle_detection_segment")
        self.get_logger().set_level(LoggingSeverity.DEBUG)
        self._frame_id = "base_link"
        self.cloud = np.empty((0, 3), dtype=np.float32)
        self._rx_count = 0
        self._empty_parse_count = 0
        self._empty_detect_count = 0
        self._zero_obs_streak = 0
        self._frame_mismatch_count = 0
        self._health_timer = self.create_timer(5.0, self._health_check)

        # publish only the obstacle-representative points (red)
        self.pub = self.create_publisher(
            PointCloud2,
            "/merged_cloud/obstacles",
            10,
        )

        self._sub_merged = self.create_subscription(
            PointCloud2,
            "/merged_cloud",
            self._cb_merged,
            10,
        )

        self.get_logger().info(
            f"Obstacle detection started  "
            f"[{N_AZ}×{N_EL} sectors, bin_w={DIST_BIN_W}m, min_pts={MIN_POINTS}]"
        )
        self.get_logger().info("Logger level forced to DEBUG in code")

    # ── parsing ───────────────────────────────────────────────────────────────

    def _parse(self, msg: PointCloud2) -> np.ndarray:
        try:
            structured = np.array(
                list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            )
        except Exception as exc:
            self.get_logger().error(f"PointCloud parse failed: {exc}")
            return np.empty((0, 3), dtype=np.float32)

        if structured.size == 0:
            return np.empty((0, 3), dtype=np.float32)

        if structured.dtype.names is None:
            # Some sensor_msgs_py versions return plain tuples instead of named fields.
            arr = np.asarray(structured, dtype=np.float32)
            if arr.ndim == 1:
                arr = arr.reshape(1, -1)
            if arr.shape[1] < 3:
                self.get_logger().error(
                    f"Unexpected parsed point shape={arr.shape}; expected (?, >=3)"
                )
                return np.empty((0, 3), dtype=np.float32)
            return arr[:, :3]

        return np.column_stack(
            [structured["x"], structured["y"], structured["z"]]
        ).astype(np.float32)

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _health_check(self) -> None:
        self.get_logger().warning(
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
        self.get_logger().warning(
            "DIAG callback "
            f"rx={self._rx_count}, frame={msg.header.frame_id}, size={msg.width}x{msg.height}"
        )
        if self._rx_count <= 5 or self._rx_count % 30 == 0:
            self.get_logger().info(
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
                    "Skipping detect/publish because cloud is empty "
                    f"(count={self._empty_detect_count})"
                )
            return

        winner_mask, winner_sector = build_sector_map(self.cloud)

        obstacle_points = self.cloud[winner_mask]
        obstacle_sectors = winner_sector[winner_mask]

        n_obs = obstacle_points.shape[0]

        if n_obs == 0:
            self._zero_obs_streak += 1
            if self._zero_obs_streak <= 10 or self._zero_obs_streak % 20 == 0:
                d = np.linalg.norm(self.cloud, axis=1)
                self.get_logger().warning(
                    "No obstacle representatives this frame "
                    f"(streak={self._zero_obs_streak}, pts={self.cloud.shape[0]}, "
                    f"dist_min={np.min(d):.3f}, dist_max={np.max(d):.3f}, "
                    f"bin_w={DIST_BIN_W}, min_pts={MIN_POINTS}, sectors={N_AZ}x{N_EL})"
                )
        else:
            if self._zero_obs_streak > 0:
                self.get_logger().info(
                    f"Obstacle detection recovered after {self._zero_obs_streak} empty frames"
                )
            self._zero_obs_streak = 0

        self.get_logger().info(
            f"Sector map: {n_obs} obstacle representatives "
            f"from {self.cloud.shape[0]} total points"
        )

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
