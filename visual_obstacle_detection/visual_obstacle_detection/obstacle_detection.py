#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header


# ── sector-map config ─────────────────────────────────────────────────────────
N_AZ = 8     # azimuth bins   (360 / 8 = 45° each)
N_EL = 8     # elevation bins (180 / 8 = 45° each)
DIST_BIN_W = 0.5    # distance shell width (metres)
MIN_POINTS = 3      # min points in a shell to count as a real obstacle
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
        self._frame_id = "zed1_left_camera_frame"

        self._cloud1 = np.empty((0, 3), dtype=np.float32)
        self._cloud2 = np.empty((0, 3), dtype=np.float32)
        self.cloud = np.empty((0, 3), dtype=np.float32)

        # publish only the obstacle-representative points (red)
        self.pub = self.create_publisher(
            PointCloud2,
            "/merged_cloud/obstacles",
            10,
        )

        self._sub_zed1 = self.create_subscription(
            PointCloud2,
            "/zed1/zed_node/point_cloud/cloud_registered",
            self._cb_zed1,
            10,
        )

        self._sub_zed2 = self.create_subscription(
            PointCloud2,
            "/zed2/zed_node/point_cloud/cloud_registered",
            self._cb_zed2,
            10,
        )

        self.get_logger().info(
            f"Obstacle detection started  "
            f"[{N_AZ}×{N_EL} sectors, bin_w={DIST_BIN_W}m, min_pts={MIN_POINTS}]"
        )

    # ── parsing ───────────────────────────────────────────────────────────────

    def _parse(self, msg: PointCloud2) -> np.ndarray:
        structured = np.array(
            list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        )
        if structured.size == 0:
            return np.empty((0, 3), dtype=np.float32)
        return np.column_stack(
            [structured["x"], structured["y"], structured["z"]]
        ).astype(np.float32)

    # ── merging ───────────────────────────────────────────────────────────────

    def _merge(self) -> None:
        self.cloud = np.concatenate((self._cloud1, self._cloud2), axis=0)

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _cb_zed1(self, msg: PointCloud2) -> None:
        self._cloud1 = self._parse(msg)
        self._merge()
        self._detect_and_publish()

    def _cb_zed2(self, msg: PointCloud2) -> None:
        self._cloud2 = self._parse(msg)
        self._merge()
        self._detect_and_publish()

    # ── detection + publish ───────────────────────────────────────────────────

    def _detect_and_publish(self) -> None:
        if self.cloud.shape[0] == 0:
            return

        winner_mask, winner_sector = build_sector_map(self.cloud)

        obstacle_points  = self.cloud[winner_mask]
        obstacle_sectors = winner_sector[winner_mask]

        n_obs = obstacle_points.shape[0]

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
