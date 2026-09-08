#!/usr/bin/env python3

import math

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class CoordinateDebugNode(Node):
    """Print wrist-camera TF and detected grasp/object pose in base_link."""

    def __init__(self):
        super().__init__("coordinate_debug")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("wrist_frame", "camera_link")
        self.declare_parameter("optical_frame", "camera_color_optical_frame")
        self.declare_parameter("grasp_pose_topic", "/grasp_pose")
        self.declare_parameter("print_period", 1.0)

        self.base_frame = self.get_parameter("base_frame").value
        self.wrist_frame = self.get_parameter("wrist_frame").value
        self.optical_frame = self.get_parameter("optical_frame").value
        self.grasp_pose_topic = self.get_parameter("grasp_pose_topic").value
        self.latest_grasp_pose = None

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            PoseStamped,
            self.grasp_pose_topic,
            self._grasp_pose_callback,
            10,
        )
        self.create_timer(
            float(self.get_parameter("print_period").value),
            self._print_coordinates,
        )

    def _grasp_pose_callback(self, msg):
        self.latest_grasp_pose = msg

    def _print_coordinates(self):
        self._print_transform(self.wrist_frame)
        self._print_transform(self.optical_frame)

        if self.latest_grasp_pose is None:
            self.get_logger().info(
                f"No message received yet on {self.grasp_pose_topic}"
            )
            return

        transformed = self._transform_pose_to_base(self.latest_grasp_pose)
        if transformed is None:
            return

        original = self.latest_grasp_pose
        self.get_logger().info(
            "Detected grasp/object pose raw: "
            f"frame={original.header.frame_id}, "
            f"pos=({original.pose.position.x:.4f}, "
            f"{original.pose.position.y:.4f}, "
            f"{original.pose.position.z:.4f}), "
            f"rpy_deg={self._pose_rpy_deg(original)}"
        )
        self.get_logger().info(
            "Detected grasp/object pose in base_link: "
            f"frame={transformed.header.frame_id}, "
            f"pos=({transformed.pose.position.x:.4f}, "
            f"{transformed.pose.position.y:.4f}, "
            f"{transformed.pose.position.z:.4f}), "
            f"rpy_deg={self._pose_rpy_deg(transformed)}"
        )

    def _print_transform(self, child_frame):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                child_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as error:
            self.get_logger().warn(
                f"Cannot lookup {self.base_frame} -> {child_frame}: {error}"
            )
            return

        t = transform.transform.translation
        q = transform.transform.rotation
        rpy = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler(
            "xyz", degrees=True
        )
        self.get_logger().info(
            f"TF {self.base_frame} -> {child_frame}: "
            f"pos=({t.x:.4f}, {t.y:.4f}, {t.z:.4f}), "
            f"rpy_deg=({rpy[0]:.1f}, {rpy[1]:.1f}, {rpy[2]:.1f})"
        )

    def _transform_pose_to_base(self, pose):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                pose.header.frame_id,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as error:
            self.get_logger().warn(
                f"Cannot transform {pose.header.frame_id} to "
                f"{self.base_frame}: {error}"
            )
            return None

        tq = transform.transform.rotation
        transform_rotation = Rotation.from_quat([tq.x, tq.y, tq.z, tq.w])
        position = np.array([
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        ])
        translation = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ])
        output_position = transform_rotation.apply(position) + translation
        pq = pose.pose.orientation
        output_rotation = transform_rotation * Rotation.from_quat(
            [pq.x, pq.y, pq.z, pq.w]
        )
        qx, qy, qz, qw = output_rotation.as_quat()

        output = PoseStamped()
        output.header.frame_id = self.base_frame
        output.header.stamp = self.get_clock().now().to_msg()
        output.pose.position.x = float(output_position[0])
        output.pose.position.y = float(output_position[1])
        output.pose.position.z = float(output_position[2])
        output.pose.orientation.x = float(qx)
        output.pose.orientation.y = float(qy)
        output.pose.orientation.z = float(qz)
        output.pose.orientation.w = float(qw)
        return output

    @staticmethod
    def _pose_rpy_deg(pose):
        q = pose.pose.orientation
        if not all(math.isfinite(v) for v in [q.x, q.y, q.z, q.w]):
            return "(nan, nan, nan)"
        rpy = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler(
            "xyz", degrees=True
        )
        return f"({rpy[0]:.1f}, {rpy[1]:.1f}, {rpy[2]:.1f})"


def main(args=None):
    rclpy.init(args=args)
    node = CoordinateDebugNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

