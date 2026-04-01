#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Occupancy Grid Mapping with Known Poses

Subscriptions:
  /scan   sensor_msgs/LaserScan   - laser range measurements
  /odom   nav_msgs/Odometry       - robot pose in odom frame

Publications:
  /grid_map   nav_msgs/OccupancyGrid  - built occupancy map (latched)
"""

import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry

import tf_transformations


class GridMappingNode(Node):

    def __init__(self):
        super().__init__('grid_mapping_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('map_size',     20.0)   # world extent [m]
        self.declare_parameter('map_res',       0.05)  # metres per cell
        self.declare_parameter('prior',         0.5)   # prior occupancy prob
        self.declare_parameter('prob_occ',      0.7)   # occupied cell update
        self.declare_parameter('prob_free',     0.3)   # free cell update
        self.declare_parameter('max_range',    10.0)   # max usable range [m]
        self.declare_parameter('publish_rate',  2.0)   # map publish rate [Hz]
        self.declare_parameter('frame_id',    'odom')  # map frame
        self.declare_parameter('save_map_path',
                               '~/robotics/ros2_ws/maps/grid_map')
        self.declare_parameter('auto_save_interval', 0.0)  # 0 = on shutdown only

        map_size          = self.get_parameter('map_size').value
        self.map_res      = self.get_parameter('map_res').value
        self.prior        = self.get_parameter('prior').value
        self.prob_occ     = self.get_parameter('prob_occ').value
        self.prob_free    = self.get_parameter('prob_free').value
        self.max_range    = self.get_parameter('max_range').value
        self.frame_id     = self.get_parameter('frame_id').value

        # ── Log-odds map (rows = Y axis, cols = X axis) ───────────────────────
        grid_n = int(np.ceil(map_size / self.map_res))
        self.l_prior = self.prob2logodds(self.prior)
        self.l_occ   = self.prob2logodds(self.prob_occ)
        self.l_free  = self.prob2logodds(self.prob_free)
        self.log_odds_map = np.full((grid_n, grid_n), self.l_prior, dtype=np.float64)

        # Pre-compute update deltas (avoids repeated subtraction inside loop)
        self.delta_occ  = self.l_occ  - self.l_prior
        self.delta_free = self.l_free - self.l_prior

        # Map half-extent in cells (for clipping bounds checks)
        self.n_rows, self.n_cols = self.log_odds_map.shape

        # ── Robot state ───────────────────────────────────────────────────────
        self.current_pose = None   # [x, y, theta] in odom frame

        # ── Latched publisher for OccupancyGrid ───────────────────────────────
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, '/grid_map', map_qos)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(Odometry,  '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # ── Publish timer ─────────────────────────────────────────────────────
        publish_rate = self.get_parameter('publish_rate').value
        self.create_timer(1.0 / publish_rate, self.publish_map)

        # ── Optional periodic map-save timer ──────────────────────────────────
        auto_save_interval = self.get_parameter('auto_save_interval').value
        if auto_save_interval > 0.0:
            self.create_timer(auto_save_interval, self.save_map)

        self.get_logger().info(
            f'Grid Mapping Node started — '
            f'{grid_n}×{grid_n} cells @ {self.map_res} m/cell '
            f'({map_size} m × {map_size} m)'
        )
      

    @staticmethod
    def prob2logodds(p: float) -> float:
        """Convert probability p ∈ (0, 1) to log-odds: log(p / (1 − p))."""
        p = np.clip(p, 1e-9, 1.0 - 1e-9)
        return float(np.log(p / (1.0 - p)))

    @staticmethod
    def logodds2prob(l: np.ndarray) -> np.ndarray:
        """Convert log-odds back to probability: 1 − 1 / (1 + exp(l))."""
        return 1.0 - 1.0 / (1.0 + np.exp(l))

    def inv_sensor_model(self, is_endpoint: bool) -> float:
        """Return the log-odds value for an occupied or free cell update."""
        return self.l_occ if is_endpoint else self.l_free

    def world2map(self, x: float, y: float):
        """Convert world (x, y) to (row, col) grid indices.

        Grid origin is at the centre.
          col = round(x / res) + n_cols / 2
          row = round(y / res) + n_rows / 2
        Returns (row, col) as ints, or None if outside grid bounds.
        """
        col = int(np.round(x / self.map_res)) + self.n_cols // 2
        row = int(np.round(y / self.map_res)) + self.n_rows // 2
        return row, col

    def in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.n_rows and 0 <= col < self.n_cols

    @staticmethod
    def v2t(pose: np.ndarray) -> np.ndarray:
        """[x, y, theta] → 3×3 homogeneous transform matrix."""
        c, s = np.cos(pose[2]), np.sin(pose[2])
        return np.array([[c, -s, pose[0]],
                         [s,  c, pose[1]],
                         [0,  0,       1]])

    @staticmethod
    def bresenham(x0: int, y0: int, x1: int, y1: int):
        """Yield (x, y) integer coordinates along the line from (x0,y0) to (x1,y1).

        Thin wrapper around the classic Bresenham algorithm.  Returns a list
        so the caller can slice off the endpoint cheaply.
        """
        pts = []
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0
        while True:
            pts.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        return pts

    def grid_mapping_with_known_poses(
        self,
        pose: np.ndarray,
        ranges: np.ndarray,
        angle_min: float,
        angle_increment: float,
    ) -> None:
        """Update log-odds map for one pose+scan pair (core assignment function).

        This is the real-time streaming equivalent of the batch assignment
        function grid_mapping_with_known_poses().  It processes one laser scan
        at the given robot pose and updates self.log_odds_map in-place.

        Steps (per valid beam):
          1. Compute endpoint in world frame.
          2. Convert robot and endpoint positions to grid cells.
          3. Trace ray with Bresenham; update free cells along ray.
          4. Update endpoint cell as occupied.
        """
        n_beams = len(ranges)
        angles  = angle_min + np.arange(n_beams) * angle_increment

        # Valid beam mask: positive, finite, within usable range
        valid = (
            np.isfinite(ranges) &
            (ranges > 0.05) &
            (ranges < self.max_range)
        )

        # Robot cell (col = x direction, row = y direction)
        r_row, r_col = self.world2map(pose[0], pose[1])

        for i in np.where(valid)[0]:
            r = ranges[i]
            a = angles[i]

            # Endpoint in world frame  (angle a is in robot/laser frame)
            ep_x = pose[0] + r * np.cos(pose[2] + a)
            ep_y = pose[1] + r * np.sin(pose[2] + a)

            ep_row, ep_col = self.world2map(ep_x, ep_y)

            # Bresenham ray: col is "x", row is "y" for the line algorithm
            ray = self.bresenham(r_col, r_row, ep_col, ep_row)

            # Free cells along ray (all except the last point)
            for col, row in ray[:-1]:
                if self.in_bounds(row, col):
                    self.log_odds_map[row, col] += self.delta_free

            # Occupied endpoint
            if self.in_bounds(ep_row, ep_col):
                self.log_odds_map[ep_row, ep_col] += self.delta_occ


    def odom_callback(self, msg: Odometry) -> None:
        """Extract (x, y, yaw) from Odometry and store as current_pose."""
        pos = msg.pose.pose.position
        q   = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )
        self.current_pose = np.array([pos.x, pos.y, yaw])

    def scan_callback(self, msg: LaserScan) -> None:
        """Receive a laser scan and update the occupancy grid."""
        if self.current_pose is None:
            return

        pose   = self.current_pose.copy()
        ranges = np.array(msg.ranges, dtype=np.float64)

        self.grid_mapping_with_known_poses(
            pose,
            ranges,
            msg.angle_min,
            msg.angle_increment,
        )


    def publish_map(self) -> None:
        msg = OccupancyGrid()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.info.resolution = self.map_res
        msg.info.width      = self.n_cols   # X direction
        msg.info.height     = self.n_rows   # Y direction
        # Origin = world coordinate of cell (row=0, col=0) = bottom-left corner
        msg.info.origin.position.x = -(self.n_cols * self.map_res) / 2.0
        msg.info.origin.position.y = -(self.n_rows * self.map_res) / 2.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        # Convert to int8 occupancy [0–100], -1 = unknown (near prior)
        prob_map  = self.logodds2prob(self.log_odds_map)
        ros_map   = np.full_like(prob_map, -1, dtype=np.int8)
        known     = np.abs(self.log_odds_map - self.l_prior) > 0.2
        ros_map[known] = np.clip(prob_map[known] * 100, 0, 100).astype(np.int8)

        # Row 0 in OccupancyGrid = minimum Y — our array is already arranged
        # with row 0 = minimum Y index, so a simple flatten is correct.
        msg.data = ros_map.flatten().tolist()
        self.map_pub.publish(msg)


    def save_map(self) -> None:
        
        from PIL import Image
        import yaml

        path = os.path.expanduser(
            self.get_parameter('save_map_path').value)
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)

        # Build uint8 image
        prob_map = self.logodds2prob(self.log_odds_map)   # (H, W) ∈ [0, 1]
        img      = np.full(prob_map.shape, 205, dtype=np.uint8)  # default: unknown
        known    = np.abs(self.log_odds_map - self.l_prior) > 0.2
        # (1 - p): high occupancy → 0 (black), high free → 254 (white)
        img[known] = np.clip(
            (1.0 - prob_map[known]) * 254, 0, 254
        ).astype(np.uint8)

        # Our array has row-0 = min-Y; PGM row-0 = top (max-Y) → flip vertically
        Image.fromarray(np.flipud(img), mode='L').save(path + '.pgm')

        # YAML metadata
        origin_x = -(self.n_cols * self.map_res) / 2.0
        origin_y = -(self.n_rows * self.map_res) / 2.0
        meta = {
            'image':           os.path.basename(path) + '.pgm',
            'mode':            'trinary',
            'resolution':      float(self.map_res),
            'origin':          [round(origin_x, 4), round(origin_y, 4), 0.0],
            'negate':          0,
            'occupied_thresh': 0.65,
            'free_thresh':     0.25,
        }
        with open(path + '.yaml', 'w') as f:
            yaml.dump(meta, f, default_flow_style=False)

        self.get_logger().info(f'Map saved → {path}.pgm / .yaml')


def main(args=None):
    rclpy.init(args=args)
    node = GridMappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_map()        # always flush the latest map on exit
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
