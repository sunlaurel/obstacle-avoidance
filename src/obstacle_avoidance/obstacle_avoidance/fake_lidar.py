"""Geometric lidar published as ``sensor_msgs/LaserScan``.

CUSTOMIZATION POINT (sensing)
-----------------------------
This node raycasts against the known wall rectangles. To use a real sensor:
  1. Add a ``gpu_lidar`` (or GPU/CPU lidar) plugin to the vehicle in the SDF.
  2. Bridge that Gazebo topic to ``/scan``.
  3. Do not launch this node.

The navigator already consumes ``/scan``, so a real lidar is a drop-in.
"""

from __future__ import annotations

import math
from typing import Sequence

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from obstacle_avoidance.obstacle_map import raycast
from obstacle_avoidance.planner import yaw_from_quat


class FakeLidar(Node):
    def __init__(self) -> None:
        super().__init__("fake_lidar")
        self.declare_parameter("range_min", 0.05)
        self.declare_parameter("range_max", 8.0)
        self.declare_parameter("angle_min", -math.pi)
        self.declare_parameter("angle_max", math.pi)
        self.declare_parameter("angle_increment", math.radians(2.0))
        self.declare_parameter("frame_id", "base_link")

        self._range_min = float(self.get_parameter("range_min").value)
        self._range_max = float(self.get_parameter("range_max").value)
        self._angle_min = float(self.get_parameter("angle_min").value)
        self._angle_max = float(self.get_parameter("angle_max").value)
        self._angle_inc = float(self.get_parameter("angle_increment").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._n = int(round((self._angle_max - self._angle_min) / self._angle_inc))

        self._pose: tuple[float, float, float] | None = None
        self.create_subscription(Odometry, "/odom", self._on_odom, qos_profile_sensor_data)
        self._pub = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.create_timer(0.05, self._tick)  # 20 Hz

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._pose = (p.x, p.y, yaw_from_quat(q.z, q.w))

    def _tick(self) -> None:
        if self._pose is None:
            return
        x, y, yaw = self._pose
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self._frame_id
        scan.angle_min = self._angle_min
        scan.angle_max = self._angle_max
        scan.angle_increment = self._angle_inc
        scan.range_min = self._range_min
        scan.range_max = self._range_max
        scan.time_increment = 0.0
        scan.scan_time = 0.05
        ranges: list[float] = []
        ang = self._angle_min
        for _ in range(self._n):
            # Ray is in the vehicle frame, so add yaw.
            d = raycast(x, y, yaw + ang, self._range_max, inflation=0.0)
            ranges.append(float(max(self._range_min, min(self._range_max, d))))
            ang += self._angle_inc
        scan.ranges = ranges
        self._pub.publish(scan)


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FakeLidar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
