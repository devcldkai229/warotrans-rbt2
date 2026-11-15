"""/rrc_lite/encoders -> /odom (+ optional TF odom->base_footprint). PATCHED FIRMWARE ONLY.

With the stock RRC Lite firmware no encoder report ever arrives, so this node publishes nothing: there is
no dead-reckoning from commanded speeds. It also stays idle until geometry, motor map and counts_per_rev
are measured (all 0 = CHƯA XÁC NHẬN in config/rbt002.yaml).
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster
from warotrans_rbt002_msgs.msg import EncoderFeedback

from .params import declare_geometry_and_map, read_geometry, read_motor_map
from .wheel_odom import WheelOdometry


class OdomNode(Node):
    def __init__(self) -> None:
        super().__init__("wheel_odom")
        declare_geometry_and_map(self)
        self.declare_parameter("counts_per_rev", 0.0)
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("pose_variance", [0.01, 0.01, 1e6, 1e6, 1e6, 0.05])
        self.declare_parameter("twist_variance", [0.01, 0.01, 1e6, 1e6, 1e6, 0.05])
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._odom_frame, self._base_frame = str(g("odom_frame")), str(g("base_frame"))
        if self._base_frame != "base_footprint":
            raise ValueError("base_frame must be base_footprint (TF contract, docs/interfaces.md)")
        self._pose_var = [float(v) for v in g("pose_variance")]
        self._twist_var = [float(v) for v in g("twist_variance")]
        geometry, motor_map, cpr = read_geometry(self), read_motor_map(self), float(g("counts_per_rev"))
        self._odo = None
        if geometry is None or motor_map is None or cpr <= 0:
            self.get_logger().warning("odometry disabled: geometry / motor map / counts_per_rev not measured "
                                      "(CHƯA XÁC NHẬN)")
        else:
            self._odo = WheelOdometry(geometry, motor_map, cpr)
        self._tf = TransformBroadcaster(self) if bool(g("publish_tf")) else None
        self._pub = self.create_publisher(Odometry, "/odom", QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(EncoderFeedback, "/rrc_lite/encoders", self._on_feedback, 20)

    def _on_feedback(self, m: EncoderFeedback) -> None:
        if self._odo is None or not self._odo.update(int(m.mcu_ms), list(m.counts)):
            return
        s = self._odo.state
        stamp = m.header.stamp
        qz, qw = math.sin(s.yaw / 2.0), math.cos(s.yaw / 2.0)
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, self._odom_frame, self._base_frame
        o.pose.pose.position.x, o.pose.pose.position.y = s.x, s.y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.angular.z = s.vx, s.vy, s.wz
        for i, v in enumerate(self._pose_var):
            o.pose.covariance[i * 7] = v
        for i, v in enumerate(self._twist_var):
            o.twist.covariance[i * 7] = v
        self._pub.publish(o)
        if self._tf is not None:
            t = TransformStamped()
            t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, self._odom_frame, self._base_frame
            t.transform.translation.x, t.transform.translation.y = s.x, s.y
            t.transform.rotation.z, t.transform.rotation.w = qz, qw
            self._tf.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = OdomNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
