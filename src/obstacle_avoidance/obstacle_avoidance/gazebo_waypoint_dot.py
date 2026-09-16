"""Show the active waypoint as one Gazebo sphere.

The navigator still consumes the full latched ``/waypoints`` list. This node
only mirrors ``waypoints[i]`` where ``i`` is ``/waypoint_index``:

  * spawn ``current_waypoint_dot`` via ``/world/<world>/create``
  * move it with ``set_pose`` when the index advances
  * ``remove`` it after the last waypoint is reached

ROS MarkerArray topics are ignored here — Gazebo does not render them.
"""

from __future__ import annotations

from typing import Sequence

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Entity, EntityFactory
from ros_gz_interfaces.srv import DeleteEntity, SetEntityPose, SpawnEntity
from std_msgs.msg import Int32

from obstacle_avoidance.world_sdf import current_waypoint_dot_sdf


class GazeboWaypointDot(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_waypoint_dot")
        self.declare_parameter("world", "obstacle_course")
        self.declare_parameter("model_name", "current_waypoint_dot")
        self.declare_parameter("dot_z", 0.35)

        self._world = str(self.get_parameter("world").value)
        self._name = str(self.get_parameter("model_name").value)
        self._z = float(self.get_parameter("dot_z").value)

        self._waypoints: list[tuple[float, float]] = []
        self._index: int | None = None
        self._shown: int | None = None
        self._spawned = False
        self._busy = False

        world = self._world
        self._create = self.create_client(SpawnEntity, f"/world/{world}/create")
        self._set_pose = self.create_client(SetEntityPose, f"/world/{world}/set_pose")
        self._remove = self.create_client(DeleteEntity, f"/world/{world}/remove")

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(PoseArray, "/waypoints", self._on_waypoints, latched)
        self.create_subscription(Int32, "/waypoint_index", self._on_index, latched)
        self.create_timer(0.4, self._sync)

        self.get_logger().info(
            f"Gazebo waypoint dot ready — waiting for /waypoints and /waypoint_index "
            f"(world={self._world})"
        )

    def _on_waypoints(self, msg: PoseArray) -> None:
        self._waypoints = [(p.position.x, p.position.y) for p in msg.poses]
        self._shown = None

    def _on_index(self, msg: Int32) -> None:
        self._index = int(msg.data)
        self._shown = None

    def _pose_at(self, x: float, y: float) -> Pose:
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = self._z
        pose.orientation.w = 1.0
        return pose

    def _entity(self) -> Entity:
        ent = Entity()
        ent.name = self._name
        ent.type = Entity.MODEL
        return ent

    def _sync(self) -> None:
        if self._busy or self._index is None or not self._waypoints:
            return
        if self._shown == self._index:
            return

        if self._index < 0 or self._index >= len(self._waypoints):
            if self._spawned:
                self._busy = True
                self._call_remove()
            else:
                self._shown = self._index
            return

        x, y = self._waypoints[self._index]
        pose = self._pose_at(x, y)
        self._busy = True
        if not self._spawned:
            self._call_spawn(pose)
        else:
            self._call_set_pose(pose)

    def _call_spawn(self, pose: Pose) -> None:
        if not self._create.wait_for_service(timeout_sec=0.1):
            self._busy = False
            self.get_logger().info("Waiting for Gazebo /create service", throttle_duration_sec=5.0)
            return
        req = SpawnEntity.Request()
        req.entity_factory = EntityFactory()
        req.entity_factory.name = self._name
        req.entity_factory.allow_renaming = False
        req.entity_factory.sdf = current_waypoint_dot_sdf(self._name)
        req.entity_factory.pose = pose
        req.entity_factory.relative_to = "world"
        future = self._create.call_async(req)
        future.add_done_callback(lambda f, p=pose: self._after_spawn(f, p))

    def _after_spawn(self, future, pose: Pose) -> None:
        ok = False
        try:
            ok = bool(future.result() and future.result().success)
        except Exception as exc:  # noqa: BLE001 — service may fail if already spawned
            self.get_logger().warning(f"SpawnEntity failed: {exc}")
        if ok:
            self._spawned = True
            self._shown = self._index
            self.get_logger().info(
                f"Spawned {self._name} at waypoint {self._index + 1}/{len(self._waypoints)}"
            )
            self._busy = False
            return
        # Model may already exist from a previous run; move it instead.
        self._spawned = True
        self._call_set_pose(pose)

    def _call_set_pose(self, pose: Pose) -> None:
        if not self._set_pose.wait_for_service(timeout_sec=0.1):
            self._busy = False
            return
        req = SetEntityPose.Request()
        req.entity = self._entity()
        req.pose = pose
        future = self._set_pose.call_async(req)
        future.add_done_callback(self._after_set_pose)

    def _after_set_pose(self, future) -> None:
        try:
            ok = bool(future.result() and future.result().success)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"SetEntityPose failed: {exc}")
            ok = False
        if ok:
            self._shown = self._index
            if self._index is not None and 0 <= self._index < len(self._waypoints):
                x, y = self._waypoints[self._index]
                self.get_logger().info(
                    f"Moved {self._name} to waypoint {self._index + 1}/{len(self._waypoints)} "
                    f"({x:.2f}, {y:.2f})"
                )
        else:
            self._spawned = False
        self._busy = False

    def _call_remove(self) -> None:
        if not self._remove.wait_for_service(timeout_sec=0.1):
            self._busy = False
            return
        req = DeleteEntity.Request()
        req.entity = self._entity()
        future = self._remove.call_async(req)
        future.add_done_callback(self._after_remove)

    def _after_remove(self, future) -> None:
        try:
            future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"DeleteEntity failed: {exc}")
        self._spawned = False
        self._shown = self._index
        self._busy = False
        self.get_logger().info(f"Removed {self._name} (mission complete or no active waypoint)")


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GazeboWaypointDot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
