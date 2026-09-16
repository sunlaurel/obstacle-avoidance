"""Autonomous waypoint follower.

Takes:
  * current pose from ``/odom``
  * the next goal from the latched ``/waypoints`` list

Publishes:
  * ``/cmd_vel`` — differential-drive Twist, no teleop
  * ``/planned_path``, ``/current_goal`` — visualization

CUSTOMIZATION POINT (navigation stack)
--------------------------------------
The method to replace is ``Navigator.compute_cmd_vel``. Everything else is
book-keeping (waypoint index, latched goals, logging). A typical upgrade path:

  1. Keep waypoint indexing as-is.
  2. Replace ``plan_path`` (in planner.py) with your global planner.
  3. Replace ``follow_path_cmd`` with your local controller.
  4. Optionally drop the occupancy grid and plan from a live costmap / lidar.
"""

from __future__ import annotations

import math
from typing import Sequence

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Header, Int32, String

from src.obstacle_map import ROBOT_RADIUS, is_occupied
from src.planner import (
    OccupancyGrid,
    follow_path_cmd,
    plan_path,
    skip_unreachable_waypoint,
    yaw_from_quat,
)


class Navigator(Node):
    def __init__(self) -> None:
        super().__init__("navigator")
        self.declare_parameter("goal_tolerance", 0.55)
        self.declare_parameter("replan_period", 1.0)
        self.declare_parameter("obstacle_stop_range", 0.40)
        self.declare_parameter("stuck_radius", 0.45)
        self.declare_parameter("skip_stuck_sec", 8.0)
        self.declare_parameter("frame_id", "odom")

        self._goal_tol = float(self.get_parameter("goal_tolerance").value)
        self._stop_range = float(self.get_parameter("obstacle_stop_range").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._stuck_radius = float(self.get_parameter("stuck_radius").value)
        self._skip_stuck_sec = float(self.get_parameter("skip_stuck_sec").value)

        self._grid = OccupancyGrid()
        self._waypoints: list[tuple[float, float]] = []
        self._index = 0
        self._pose: tuple[float, float, float] | None = None
        self._path: list[tuple[float, float]] = []
        self._last_replan = 0.0
        self._stuck_xy: tuple[float, float] | None = None
        self._stuck_since: float | None = None
        self._linger_xy: tuple[float, float] | None = None
        self._linger_since: float | None = None
        self._done = False

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(PoseArray, "/waypoints", self._on_waypoints, latched)
        self.create_subscription(Odometry, "/odom", self._on_odom, qos_profile_sensor_data)

        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._path_pub = self.create_publisher(Path, "/planned_path", 10)
        self._goal_pub = self.create_publisher(PoseStamped, "/current_goal", 10)
        self._idx_pub = self.create_publisher(Int32, "/waypoint_index", 10)
        self._status_pub = self.create_publisher(String, "/nav_status", 10)

        self.create_timer(0.05, self._tick)  # 20 Hz control loop
        self.get_logger().info("Navigator ready — waiting for /odom and /waypoints")

    def _on_waypoints(self, msg: PoseArray) -> None:
        wps = [(p.position.x, p.position.y) for p in msg.poses]
        if wps == self._waypoints:
            return
        self._waypoints = wps
        self._index = 0
        self._path = []
        self._done = False
        self.get_logger().info(f"Received {len(wps)} waypoints")

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._pose = (p.x, p.y, yaw_from_quat(q.z, q.w))

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)

    def _publish_path(self) -> None:
        path = Path()
        path.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self._frame_id)
        for x, y in self._path:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self._path_pub.publish(path)

    def _current_goal(self) -> tuple[float, float] | None:
        if self._index < len(self._waypoints):
            return self._waypoints[self._index]
        return None

    def _reset_progress_timers(self) -> None:
        self._stuck_xy = None
        self._stuck_since = None
        self._linger_xy = None
        self._linger_since = None

    def _advance_waypoint(self, *, reached: bool, gx: float, gy: float) -> None:
        n = len(self._waypoints)
        finished = self._index + 1
        label = "Reached" if reached else "Skipping unreachable"
        self.get_logger().info(f"{label} waypoint {finished}/{n} at ({gx:.2f}, {gy:.2f})")
        self._index += 1
        self._path = []
        self._reset_progress_timers()
        self._publish_index()
        if self._index >= n:
            self._done = True
            self._publish_status("mission_complete")
            self.get_logger().info("All waypoints visited. Holding position.")
        elif not reached:
            self._publish_status(f"skipped_wp{finished} linger>={self._skip_stuck_sec:.1f}s")
    

    def compute_cmd_vel(self, x: float, y: float, yaw: float) -> Twist:
        """Return the next driving command.

        CUSTOMIZATION POINT: replace the body of this method with your own
        navigation logic. You already have:
          * (x, y, yaw)  — current pose from /odom
          * self._current_goal() — next waypoint
          * self._grid / obstacle_map — known wall rectangles
        Publish by returning a Twist; the timer publishes it on /cmd_vel.
        """
        cmd = Twist()
        goal = self._current_goal()
        if goal is None:
            return cmd

        now = self.get_clock().now().nanoseconds * 1e-9
        gx, gy = goal
        dist = math.hypot(gx - x, gy - y)
        if dist < self._goal_tol:
            self.get_logger().info(f"Reached waypoint {self._index + 1}/{len(self._waypoints)} at ({gx:.2f}, {gy:.2f})")
            self._index += 1
            self._path = []
            idx = Int32()
            idx.data = self._index
            self._idx_pub.publish(idx)
            if self._index >= len(self._waypoints):
                self._done = True
                self._publish_status("mission_complete")
                self.get_logger().info("All waypoints visited. Holding position.")
            return cmd

        # Linger window: reset only when the vehicle leaves stuck_radius.
        # Replans must not reset this, or an unreachable goal would spin forever.
        if self._linger_xy is None:
            self._linger_xy = (x, y)
            self._linger_since = now
        elif math.hypot(x - self._linger_xy[0], y - self._linger_xy[1]) > self._stuck_radius:
            self._linger_xy = (x, y)
            self._linger_since = now
        linger = 0.0 if self._linger_since is None else now - self._linger_since
        if skip_unreachable_waypoint(linger, self._skip_stuck_sec, dist, self._goal_tol):
            self._advance_waypoint(reached=False, gx=gx, gy=gy)
            return cmd

        # Replan periodically, or when we have no path, or if we look stuck.
        need_replan = not self._path or (now - self._last_replan) > float(self.get_parameter("replan_period").value)
        if self._stuck_xy is None:
            self._stuck_xy = (x, y)
            self._stuck_since = now
        elif math.hypot(x - self._stuck_xy[0], y - self._stuck_xy[1]) > 0.15:
            self._stuck_xy = (x, y)
            self._stuck_since = now
        elif self._stuck_since is not None and now - self._stuck_since > 4.0:
            need_replan = True
            self.get_logger().warning("Stuck — forcing replan and in-place turn")
            self._stuck_since = now

        if need_replan:
            self._path = plan_path((x, y), goal, self._grid)
            self._last_replan = now
            self._publish_path()
            if not self._path:
                self.get_logger().warning("A* found no path; spinning in place")
                cmd.angular.z = 0.6
                self._publish_status(f"no_path index={self._index}")
                return cmd

        v, w = follow_path_cmd(x, y, yaw, self._path)

        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self._publish_status(f"goto_wp{self._index + 1} dist={dist:.2f}")
        return cmd

    def _tick(self) -> None:
        cmd = Twist()
        if self._pose is None:
            self._cmd_pub.publish(cmd)
            return
        x, y, yaw = self._pose
        if is_occupied(x, y, inflation=0.05):
            # Should not happen; log so map/SDF mismatches are obvious.
            self.get_logger().warning(f"Pose ({x:.2f},{y:.2f}) is inside an occupied cell", throttle_duration_sec=2.0)

        if self._done or not self._waypoints:
            self._cmd_pub.publish(cmd)
            return

        cmd = self.compute_cmd_vel(x, y, yaw)
        self._cmd_pub.publish(cmd)

        goal = self._current_goal()
        if goal is not None:
            g = PoseStamped()
            g.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self._frame_id)
            g.pose.position.x = goal[0]
            g.pose.position.y = goal[1]
            g.pose.orientation.w = 1.0
            self._goal_pub.publish(g)


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
