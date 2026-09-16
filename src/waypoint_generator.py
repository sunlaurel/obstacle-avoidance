"""Publish a randomized sequence of free-space XY waypoints.

CUSTOMIZATION POINT (waypoint generation)
-----------------------------------------
The function to replace is ``mission.sample_waypoints``. This node only
turns that list into latched ROS messages. Ideas for a custom generator:
  * load a mission file (YAML / GPS list)
  * coverage / lawnmower pattern
  * a ROS 2 service that accepts goals from a UI
"""

from __future__ import annotations

import random
import time
from typing import Sequence
import subprocess

import rclpy
import math
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from src.obstacle_map import START_XY, ROBOT_RADIUS, is_free
from src.planner import OccupancyGrid, plan_path


def _pose(x: float, y: float) -> Pose:
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = 0.15
    pose.orientation.w = 1.0
    return pose


def sample_waypoints(
    count: int,
    rng: random.Random,
    start: tuple[float, float] = START_XY,
    xy_min: float = -8.5,
    xy_max: float = 8.5,
    min_spacing: float = 3.0,
    min_start_dist: float = 2.5,
    inflation: float = ROBOT_RADIUS,
    max_tries: int = 400,
) -> list[tuple[float, float]]:
    """Rejection-sample free XY points that are A*-reachable in sequence."""
    grid = OccupancyGrid(inflation=inflation)
    waypoints: list[tuple[float, float]] = []
    prev = start
    attempts = 0
    while len(waypoints) < count and attempts < max_tries:
        attempts += 1
        x = rng.uniform(xy_min, xy_max)
        y = rng.uniform(xy_min, xy_max)
        if not is_free(x, y, inflation):
            continue
        if math.hypot(x - start[0], y - start[1]) < min_start_dist:
            continue
        if any(math.hypot(x - px, y - py) < min_spacing for px, py in waypoints):
            continue
        if not plan_path(prev, (x, y), grid):
            continue
        waypoints.append((x, y))
        prev = (x, y)
    return waypoints


class WaypointGenerator(Node):
    def __init__(self) -> None:
        super().__init__("waypoint_generator")
        self.declare_parameter("num_waypoints", 4)
        self.declare_parameter("seed", 0)  # 0 = time-based
        self.declare_parameter("frame_id", "odom")

        seed = int(self.get_parameter("seed").value)
        if seed == 0:
            seed = int(time.time()) & 0xFFFFFFFF
        count = int(self.get_parameter("num_waypoints").value)
        frame_id = str(self.get_parameter("frame_id").value)

        rng = random.Random(seed)
        self.waypoints = sample_waypoints(count, rng)
        if len(self.waypoints) < count:
            self.get_logger().warning(
                f"Only sampled {len(self.waypoints)}/{count} reachable waypoints (seed={seed})"
            )
        else:
            self.get_logger().info(f"Sampled {count} waypoints with seed={seed}: {self.waypoints}")

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._wp_pub = self.create_publisher(PoseArray, "/waypoints", latched)
        self._marker_pub = self.create_publisher(MarkerArray, "/waypoint_markers", latched)
        self._goal_pub = self.create_publisher(PoseStamped, "/first_goal", latched)

        self._msg = PoseArray()
        self._msg.header = Header(frame_id=frame_id)
        self._msg.poses = [_pose(x, y) for x, y in self.waypoints]
        self._markers = self._make_markers(frame_id)

        # Latched topics: republish a few times in case late subscribers miss the first one.
        self._ticks = 0
        self.create_timer(0.5, self._publish)

    def _make_markers(self, frame_id: str) -> MarkerArray:
        arr = MarkerArray()
        for i, (x, y) in enumerate(self.waypoints):
            m = Marker()
            m.header.frame_id = frame_id
            m.ns = "waypoints"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.15
            m.pose.orientation.w = 1.0
            m.scale.x = 0.35
            m.scale.y = 0.35
            m.scale.z = 0.35
            # Green-ish, later waypoints slightly bluer.
            t = i / max(1, len(self.waypoints) - 1)
            m.color.r = 0.1
            m.color.g = 0.85 - 0.4 * t
            m.color.b = 0.2 + 0.6 * t
            m.color.a = 0.9
            arr.markers.append(m)

            label = Marker()
            label.header.frame_id = frame_id
            label.ns = "waypoint_labels"
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 0.6
            label.pose.orientation.w = 1.0
            label.scale.z = 0.4
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = f"WP{i + 1}"
            arr.markers.append(label)
        return arr

    def _publish(self) -> None:
        now = self.get_clock().now().to_msg()
        self._msg.header.stamp = now
        self._wp_pub.publish(self._msg)
        for m in self._markers.markers:
            m.header.stamp = now
        self._marker_pub.publish(self._markers)
        if self.waypoints:
            goal = PoseStamped()
            goal.header = self._msg.header
            goal.pose = self._msg.poses[0]
            self._goal_pub.publish(goal)
        self._ticks += 1
        if self._ticks > 6:
            # Keep the latched messages alive but stop spamming.
            pass


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = WaypointGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
