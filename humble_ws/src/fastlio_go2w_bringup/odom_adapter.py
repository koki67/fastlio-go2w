from __future__ import annotations

from math import isfinite
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformListener

from .math_utils import (
    pose_matrix_from_pose_msg,
    pose_msg_from_matrix,
    rebase_transform_fastlio_to_base,
)


class OdomAdapter(Node):
    def __init__(self):
        super().__init__("fastlio_odom_adapter")

        self.declare_parameters(
            "",
            [
                ("odom_input_topic", "/Odometry", ParameterDescriptor()),
                ("odom_output_topic", "/odom", ParameterDescriptor()),
                ("base_frame", "base_link", ParameterDescriptor()),
                ("imu_frame", "livox_imu_frame", ParameterDescriptor()),
                ("camera_init_frame", "camera_init", ParameterDescriptor()),
                ("odom_frame", "odom", ParameterDescriptor()),
                ("sensor_frame_retry_sec", 1.0, ParameterDescriptor()),
            ],
        )

        self._imu_frame = self.get_parameter("imu_frame").get_parameter_value().string_value
        self._base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self._odom_frame = self.get_parameter("odom_frame").get_parameter_value().string_value
        self._camera_init_frame = self.get_parameter("camera_init_frame").get_parameter_value().string_value
        self._output_topic = self.get_parameter("odom_output_topic").get_parameter_value().string_value
        self._input_topic = self.get_parameter("odom_input_topic").get_parameter_value().string_value
        self._retry_sec = float(
            self.get_parameter("sensor_frame_retry_sec").get_parameter_value().double_value
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_br = TransformBroadcaster(self)
        self._static_tf_br = StaticTransformBroadcaster(self)

        self._base_to_imu: Optional[np.ndarray] = None
        self._published_camera_init_static = False
        self._last_stamp: Optional[Time] = None

        self._publisher = self.create_publisher(Odometry, self._output_topic, 10)
        self.create_subscription(Odometry, self._input_topic, self._odom_cb, 10)
        self.create_timer(self._retry_sec, self._refresh_base_to_imu)

    def _refresh_base_to_imu(self):
        if self._base_to_imu is not None:
            return

        try:
            # Lookup base -> imu from static TF in URDF launch tree.
            tf = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._imu_frame,
                Time(),
                Duration(seconds=self._retry_sec),
            )
        except Exception:
            self.get_logger().warn(
                f"Waiting for static TF {self._base_frame}->{self._imu_frame}. "
                "Retrying every second."
            )
            return

        self._base_to_imu = pose_matrix_from_pose_msg(tf.transform)
        self._publish_static_odom_to_camera_init(tf.header.stamp)

    def _publish_static_odom_to_camera_init(self, stamp: Time):
        if self._base_to_imu is None or self._published_camera_init_static:
            return

        static_tf = TransformStamped()
        static_tf.header.frame_id = self._odom_frame
        static_tf.child_frame_id = self._camera_init_frame
        pose_msg_from_matrix(self._base_to_imu, static_tf.transform)
        static_tf.header.stamp = stamp
        self._static_tf_br.sendTransform([static_tf])
        self._published_camera_init_static = True
        self.get_logger().info(
            f"Published static transform {self._odom_frame}->{self._camera_init_frame}"
        )

    def _odom_cb(self, msg: Odometry) -> None:
        if self._base_to_imu is None:
            self._refresh_base_to_imu()
            if self._base_to_imu is None:
                self.get_logger().warn(
                    "Skipping /Odometry: base->imu TF not ready.",
                    throttle_duration_sec=1.0,
                )
                return

        msg_time = Time.from_msg(msg.header.stamp)
        if self._last_stamp is not None and msg_time == self._last_stamp:
            self.get_logger().warn(
                "Skipping duplicate /Odometry stamp.",
                throttle_duration_sec=1.0,
            )
            return
        self._last_stamp = msg_time

        body_matrix = pose_matrix_from_pose_msg(msg.pose.pose)
        base_matrix = rebase_transform_fastlio_to_base(body_matrix, self._base_to_imu)

        out = Odometry()
        out.header = msg.header
        out.header.frame_id = self._odom_frame
        out.child_frame_id = self._base_frame
        pose_msg_from_matrix(base_matrix, out.pose.pose)
        out.pose.covariance = list(msg.pose.covariance)

        out.twist = msg.twist
        out.twist.twist.linear.x = 0.0
        out.twist.twist.linear.y = 0.0
        out.twist.twist.linear.z = 0.0
        out.twist.twist.angular.x = 0.0
        out.twist.twist.angular.y = 0.0
        out.twist.twist.angular.z = 0.0
        if len(out.twist.covariance) != 36:
            out.twist.covariance = [0.0] * 36

        self._publisher.publish(out)

        transform = TransformStamped()
        transform.header.frame_id = out.header.frame_id
        transform.child_frame_id = out.child_frame_id
        if isfinite(msg_time.nanoseconds):
            transform.header.stamp = msg.header.stamp
        else:
            transform.header.stamp = self.get_clock().now().to_msg()
        pose_msg_from_matrix(base_matrix, transform.transform)
        self._tf_br.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = OdomAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
