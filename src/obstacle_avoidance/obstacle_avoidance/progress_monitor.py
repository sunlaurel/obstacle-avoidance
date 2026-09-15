"""Exit 0 once the vehicle has visited N waypoints (or 1 on timeout).

Used by the headless smoke test; not required to run the simulation.
"""

from __future__ import annotations

import math
import sys
from typing import Sequence

import rclpy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String

from obstacle_avoidance.obstacle_map import is_occupied


class ProgressMonitor(Node):
    def __init__(self) -> None:
        super().__init__("progress_monitor")
        self.declare_parameter("required_waypoints", 1)
        self.declare_parameter("timeout_sec", 90.0)
        self.declare_parameter("goal_tolerance", 0.7)

        self._need = int(self.get_parameter("required_waypoints").value)
        self._timeout = float(self.get_parameter("timeout_sec").value)
        self._tol = float(self.get_parameter("goal_tolerance").value)

        self.waypoints: list[tuple[float, float]] = []
        self._index = 0
        self.success = False
        self.failed = False
        self._reason = ""
        self._start = self.get_clock().now()
        self._last_pose: tuple[float, float] | None = None
        self.collision_hits = 0

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(PoseArray, "/waypoints", self._on_wps, latched)
        self.create_subscription(Odometry, "/odom", self._on_odom, qos_profile_sensor_data)
        self.create_subscription(String, "/nav_status", self._on_status, 10)
        self.create_timer(0.2, self._tick)

    def _on_wps(self, msg: PoseArray) -> None:
        self.waypoints = [(p.position.x, p.position.y) for p in msg.poses]
        self.get_logger().info(f"Monitoring {len(self.waypoints)} waypoints")

    def _on_status(self, msg: String) -> None:
        if msg.data == "mission_complete":
            self.success = True
            self._reason = "mission_complete"

    def _on_odom(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self._last_pose = (x, y)
        if is_occupied(x, y, inflation=0.02):
            self.collision_hits += 1
        if self.waypoints and self._index < len(self.waypoints):
            gx, gy = self.waypoints[self._index]
            if math.hypot(gx - x, gy - y) < self._tol:
                self._index += 1
                self.get_logger().info(f"Monitor: reached waypoint {self._index}/{len(self.waypoints)}")
                if self._index >= self._need:
                    self.success = True
                    self._reason = f"reached_{self._index}_waypoints"

    def _tick(self) -> None:
        elapsed = (self.get_clock().now() - self._start).nanoseconds * 1e-9
        if self.success:
            rclpy.shutdown()
            return
        if elapsed > self._timeout:
            pose = self._last_pose
            self._reason = f"timeout after {elapsed:.1f}s pose={pose} index={self._index}"
            self.failed = True
            rclpy.shutdown()


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ProgressMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    reason = node._reason
    hits = node.collision_hits
    ok = node.success and hits < 8
    node.get_logger().info(f"progress_monitor done ok={ok} reason={reason} wall_hits={hits}")
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0 if ok else 1)
