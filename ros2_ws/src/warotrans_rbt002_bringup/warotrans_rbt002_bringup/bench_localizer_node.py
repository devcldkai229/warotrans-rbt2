"""bench_localizer: stands in for AMCL when BENCH_MODE=true (robot on a stand, wheels in the air).

  /initialpose (operator or the backend's LOCALIZE) -> map->odom fixed so the robot is at that pose now
  map->odom TF at 20 Hz, /amcl_pose at 2 Hz with a small covariance (the gateway's AMCL readiness then accepts it)
  The pose survives a restart (~/.cache/warotrans/bench_pose.json): the robot does not jump back to the stand.

Never run it with AMCL, and never on a robot standing on the floor: there the lidar is the truth, not the wheels.
"""

from __future__ import annotations

import json
import math
import os
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from warotrans_rbt002_bringup.bench_pose import Pose2D, map_pose, map_to_odom

_XY_VAR = 0.02 ** 2
_YAW_VAR = 0.02 ** 2


def _yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class BenchLocalizer(Node):
    def __init__(self) -> None:
        super().__init__("bench_localizer")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_yaw", 0.0)
        self.declare_parameter("state_file", os.path.expanduser("~/.cache/warotrans/bench_pose.json"))
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self._map, self._odom, self._base = p("global_frame"), p("odom_frame"), p("base_frame")
        self._state_file = str(p("state_file"))

        self._map_odom: Optional[Pose2D] = None
        self._pending: Optional[Pose2D] = self._load_state() or (
            float(p("initial_x")), float(p("initial_y")), float(p("initial_yaw")))

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._broadcaster = TransformBroadcaster(self)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pose_pub = self.create_publisher(PoseWithCovarianceStamped, "amcl_pose", qos)
        self.create_subscription(PoseWithCovarianceStamped, "initialpose", self._on_initial_pose, 10)
        self.create_timer(0.05, self._tick_tf)
        self.create_timer(0.5, self._tick_pose)
        self.get_logger().warning(
            "BENCH MODE: no AMCL, pose = wheel odometry from the last /initialpose. Never use on the floor.")

    # --- state -------------------------------------------------------------------------------------------------
    def _load_state(self) -> Optional[Pose2D]:
        try:
            with open(self._state_file, encoding="utf-8") as f:
                d = json.load(f)
            return (float(d["x"]), float(d["y"]), float(d["yaw"]))
        except (OSError, ValueError, KeyError, TypeError):
            return None

    def _save_state(self, pose: Pose2D) -> None:
        try:
            os.makedirs(os.path.dirname(self._state_file), exist_ok=True)
            tmp = self._state_file + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump({"x": pose[0], "y": pose[1], "yaw": pose[2]}, f)
            os.replace(tmp, self._state_file)
        except OSError as e:
            self.get_logger().warning(f"bench pose not saved: {e}")

    # --- TF ----------------------------------------------------------------------------------------------------
    def _odom_base(self) -> Optional[Pose2D]:
        try:
            t = self._tf_buffer.lookup_transform(self._odom, self._base, Time(), timeout=Duration(seconds=0.0))
        except Exception:  # noqa: BLE001 - not available yet
            return None
        tr = t.transform
        return (tr.translation.x, tr.translation.y, _yaw_of(tr.rotation))

    def _on_initial_pose(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        self._pending = (pose.position.x, pose.position.y, _yaw_of(pose.orientation))
        self.get_logger().info(f"bench pose set to x={self._pending[0]:.3f} y={self._pending[1]:.3f} "
                               f"yaw={self._pending[2]:.3f}")

    def _tick_tf(self) -> None:
        odom_base = self._odom_base()
        if self._pending is not None and odom_base is not None:
            self._map_odom = map_to_odom(self._pending, odom_base)
            self._pending = None
        if self._map_odom is None:
            return
        x, y, yaw = self._map_odom
        t = TransformStamped()
        # Slightly future-dated like AMCL (transform_tolerance) so lookups at "now" succeed.
        t.header.stamp = (self.get_clock().now() + Duration(seconds=0.1)).to_msg()
        t.header.frame_id = self._map
        t.child_frame_id = self._odom
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        self._broadcaster.sendTransform(t)

    def _tick_pose(self) -> None:
        odom_base = self._odom_base()
        if self._map_odom is None or odom_base is None:
            return
        x, y, yaw = map_pose(self._map_odom, odom_base)
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        cov = [0.0] * 36
        cov[0] = cov[7] = _XY_VAR
        cov[35] = _YAW_VAR
        msg.pose.covariance = cov
        self._pose_pub.publish(msg)
        self._save_state((x, y, yaw))


def main() -> None:
    rclpy.init()
    node = BenchLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
