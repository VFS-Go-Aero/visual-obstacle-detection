#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.logging import LoggingSeverity
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header


# ── sector-map config ─────────────────────────────────────────────────────────
N_AZ = 32     # azimuth bins   (360 / 32 = 11.25° each)
N_EL = 8     # elevation bins (180 / 8 = 22.5° each)
DIST_BIN_W = 0.1    # distance shell width (metres)
MIN_POINTS = 100      # min points in a shell to count as a real obstacle
DIST_EMA_ALPHA = 0.15    # smoothing factor for per-sector reported distance (0=frozen, 1=no smoothing)
SECTOR_MAX_AGE_SEC = 0.3    # expire a sector if it has not been refreshed for this long, bridges brief input dropouts
SECTOR_DISTANCE_TOL = 0.25    # hysteresis tolerance before a sector distance is treated as a real change
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
    The reported obstacle point is placed at the sector's angular center,
    at the distance of the closest point inside that dense bin.

    Returns
    -------
    obstacle_points : float array, shape (M, 3)
        One representative xyz point per winning sector, centered on the
        sector's azimuth/elevation midpoint at the detected obstacle distance.
    obstacle_sectors : uint32 array, shape (M,)
        Sector id (a * n_el + e) for each entry in obstacle_points.

    """
    if points.shape[0] == 0:
        return np.zeros((0, 3), dtype=np.float32), np.zeros(0, dtype=np.uint32)

    # drop NaN and zero-distance points
    finite_mask = np.isfinite(points).all(axis=1)
    dists = np.linalg.norm(points, axis=1)
    valid = finite_mask & (dists > 0) & (dists <= 6.0)

    if not np.any(valid):
        return np.zeros((0, 3), dtype=np.float32), np.zeros(0, dtype=np.uint32)

    dirs = np.zeros_like(points)
    dirs[valid] = points[valid] / dists[valid, np.newaxis]

    az = np.arctan2(dirs[:, 1], dirs[:, 0])            # −π … π
    el = np.arcsin(np.clip(dirs[:, 2], -1.0, 1.0))     # −π/2 … π/2

    az_idx = ((az + np.pi) / (2 * np.pi) * n_az).astype(int) % n_az
    el_idx = ((el + np.pi / 2) / np.pi * n_el).astype(int).clip(0, n_el - 1)

    az_bin_w = 2 * np.pi / n_az
    el_bin_w = np.pi / n_el

    obstacle_points = []
    obstacle_sectors = []

    for a in range(n_az):
        for e in range(n_el):
            mask = valid & (az_idx == a) & (el_idx == e)
            if not np.any(mask):
                continue

            sector_dists = dists[mask]

            max_d = sector_dists.max()
            if not np.isfinite(max_d) or max_d <= 0:
                continue
            bin_edges = np.arange(0, max_d + dist_bin_w, dist_bin_w)
            bin_ids = np.digitize(sector_dists, bin_edges) - 1   # 0-indexed

            # walk near → far; first bin with enough points wins
            for b in range(len(bin_edges) - 1):
                in_bin = bin_ids == b
                if in_bin.sum() >= min_pts:
                    closest_dist = sector_dists[in_bin].min()

                    az_center = -np.pi + (a + 0.5) * az_bin_w
                    el_center = -np.pi / 2 + (e + 0.5) * el_bin_w
                    center_dir = np.array([
                        np.cos(el_center) * np.cos(az_center),
                        np.cos(el_center) * np.sin(az_center),
                        np.sin(el_center),
                    ])

                    obstacle_points.append(center_dir * closest_dist)
                    obstacle_sectors.append(a * n_el + e)
                    break
            # no bin reached threshold → sector is clear, no winner

    if not obstacle_points:
        return np.zeros((0, 3), dtype=np.float32), np.zeros(0, dtype=np.uint32)

    return (np.asarray(obstacle_points, dtype=np.float32),
            np.asarray(obstacle_sectors, dtype=np.uint32))


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
        self._sector_state = {}
        self._last_stamp_sec = None
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
            f"[{N_AZ}×{N_EL} sectors, bin_w={DIST_BIN_W}m, min_pts={MIN_POINTS}]"
        )
        if self._verbose:
            self.get_logger().debug("Logger level forced to DEBUG in code")

    # ── parsing ───────────────────────────────────────────────────────────────

    def _parse(self, msg: PointCloud2) -> np.ndarray:
        try:
            points = pc2.read_points_numpy(
                msg,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )
        except Exception as exc:
            self.get_logger().error(f"PointCloud parse failed: {exc}")
            return np.empty((0, 3), dtype=np.float32)

        if points.size == 0:
            return np.empty((0, 3), dtype=np.float32)
        return np.asarray(points, dtype=np.float32).reshape(-1, 3)

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

        # use the ZED-stamped capture time, not local wall-clock, so sector aging tracks sensor time
        self._last_stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._detect_and_publish()

    # ── detection + publish ───────────────────────────────────────────────────

    def _smooth_distances(self, points: np.ndarray, sectors: np.ndarray):
        """Hold each sector by sensor capture time so brief input dropouts don't blank the whole map at once."""
        state = self._sector_state
        now = self._last_stamp_sec
        if not now:
            # no valid header stamp on this message (e.g. stamp never set upstream) → fall back to frame order
            state.clear()
            return points, sectors

        current_ids = set()
        filtered_points = []
        filtered_sectors = []

        for p, sector_id in zip(points, sectors):
            sid = int(sector_id)
            current_ids.add(sid)
            dist = float(np.linalg.norm(p))
            if dist <= 0:
                continue

            direction = p / dist
            prev_state = state.get(sid)
            prev_dist = float(prev_state["dist"]) if prev_state is not None else dist

            if prev_state is None or dist < prev_dist:
                # obstacle got closer (or first sighting) → trust it immediately, no lag
                new_dist = dist
            elif abs(dist - prev_dist) <= SECTOR_DISTANCE_TOL:
                new_dist = prev_dist + DIST_EMA_ALPHA * (dist - prev_dist)
            else:
                new_dist = prev_dist + 0.35 * (dist - prev_dist)

            state[sid] = {
                "dist": new_dist,
                "dir": direction,
                "last_seen": now,
            }
            filtered_points.append(direction * new_dist)
            filtered_sectors.append(sid)

        # keep stale sectors alive briefly, then expire them in bulk once their hold time elapses
        to_drop = []
        for sid, s in list(state.items()):
            if sid in current_ids:
                continue
            if now - s["last_seen"] > SECTOR_MAX_AGE_SEC:
                to_drop.append(sid)
            else:
                filtered_points.append(s["dir"] * s["dist"])
                filtered_sectors.append(sid)

        for sid in to_drop:
            del state[sid]

        if not filtered_points:
            return np.zeros((0, 3), dtype=np.float32), np.zeros(0, dtype=np.uint32)

        return (np.asarray(filtered_points, dtype=np.float32),
                np.asarray(filtered_sectors, dtype=np.uint32))

    def _detect_and_publish(self) -> None:
        if self.cloud.shape[0] == 0:
            self._empty_detect_count += 1
            if self._empty_detect_count <= 10 or self._empty_detect_count % 20 == 0:
                self.get_logger().warning(
                    "Cloud is empty this frame, publishing held sector state instead "
                    f"(count={self._empty_detect_count})"
                )
            obstacle_points = np.zeros((0, 3), dtype=np.float32)
            obstacle_sectors = np.zeros(0, dtype=np.uint32)
        else:
            obstacle_points, obstacle_sectors = build_sector_map(self.cloud)

        obstacle_points, obstacle_sectors = self._smooth_distances(obstacle_points, obstacle_sectors)

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
