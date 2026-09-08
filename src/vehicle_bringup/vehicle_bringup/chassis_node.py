#!/usr/bin/env python3
"""ROS 2 Twist-to-CAN115 chassis driver with fail-safe stopping."""

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> Quaternion:
    msg = Quaternion()
    msg.z = math.sin(yaw / 2.0)
    msg.w = math.cos(yaw / 2.0)
    return msg


class ChassisNode(Node):
    """Receive physical velocity commands and drive the YK four-wheel chassis."""

    def __init__(self) -> None:
        super().__init__("chassis_node")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("can_host", "192.168.0.7")
        self.declare_parameter("can_port", 5578)
        self.declare_parameter("max_linear_speed", 1.257)
        self.declare_parameter("max_angular_speed", 1.67)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("reconnect_period", 2.0)
        self.declare_parameter("publish_odom", True)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        if self.max_linear_speed <= 0.0 or self.max_angular_speed <= 0.0:
            raise ValueError("max_linear_speed and max_angular_speed must be positive")

        self.car = None
        self.last_command_stamp = self.get_clock().now()
        self.last_linear = 0.0
        self.last_angular = 0.0
        self.stopped_for_timeout = True
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_odom_stamp = self.get_clock().now()

        topic = str(self.get_parameter("cmd_vel_topic").value)
        self.create_subscription(Twist, topic, self._on_cmd_vel, 10)
        self.create_timer(0.05, self._watchdog)
        self.create_timer(float(self.get_parameter("reconnect_period").value), self._ensure_connection)

        self.publish_odom_enabled = bool(self.get_parameter("publish_odom").value)
        if self.publish_odom_enabled:
            self.odom_publisher = self.create_publisher(Odometry, "/odom", 10)
            self.tf_broadcaster = TransformBroadcaster(self)
            self.create_timer(0.02, self._publish_odom)

        self._ensure_connection()
        self.get_logger().info(f"底盘节点就绪，订阅 {topic}；超时 {self.command_timeout:.2f}s 自动停车")

    def _new_vehicle(self):
        from yk_can_sdk import FourWheelVehicle, VehicleConfig
        from yk_can_sdk.config import NetworkConfig

        host = str(self.get_parameter("can_host").value)
        port = int(self.get_parameter("can_port").value)
        config = VehicleConfig(network=NetworkConfig(host=host, port=port))
        return FourWheelVehicle(config)

    def _ensure_connection(self) -> None:
        if self.car is not None and self.car.is_connected:
            return
        try:
            if self.car is not None:
                self.car.close()
            self.car = self._new_vehicle()
            self.car.connect()
            self.stopped_for_timeout = True
            self.get_logger().info(
                "已连接 CAN115 %s:%d"
                % (self.get_parameter("can_host").value, self.get_parameter("can_port").value)
            )
        except Exception as exc:  # hardware may legitimately be absent during build/test
            self.car = None
            self.get_logger().warning(f"CAN115 暂不可用，将自动重试：{exc}")

    def _on_cmd_vel(self, msg: Twist) -> None:
        linear = max(-self.max_linear_speed, min(self.max_linear_speed, float(msg.linear.x)))
        angular = max(-self.max_angular_speed, min(self.max_angular_speed, float(msg.angular.z)))
        self.last_command_stamp = self.get_clock().now()
        self.stopped_for_timeout = False

        if self.car is None or not self.car.is_connected:
            self.last_linear = 0.0
            self.last_angular = 0.0
            return
        try:
            self.car.set_motion(
                linear=linear / self.max_linear_speed,
                angular=angular / self.max_angular_speed,
            )
            self.last_linear = linear
            self.last_angular = angular
        except Exception as exc:
            self.last_linear = 0.0
            self.last_angular = 0.0
            self.get_logger().error(f"底盘指令发送失败：{exc}")

    def _stop(self) -> None:
        self.last_linear = 0.0
        self.last_angular = 0.0
        if self.car is not None and self.car.is_connected:
            try:
                self.car.stop()
            except Exception as exc:
                self.get_logger().warning(f"停车指令发送失败：{exc}")

    def _watchdog(self) -> None:
        age = (self.get_clock().now() - self.last_command_stamp).nanoseconds / 1e9
        if age > self.command_timeout and not self.stopped_for_timeout:
            self.stopped_for_timeout = True
            self._stop()
            self.get_logger().warning("/cmd_vel 超时，已停车")

    def _publish_odom(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_odom_stamp).nanoseconds / 1e9
        self.last_odom_stamp = now
        if dt <= 0.0:
            return
        self.x += self.last_linear * math.cos(self.yaw) * dt
        self.y += self.last_linear * math.sin(self.yaw) * dt
        self.yaw += self.last_angular * dt

        odom_frame = str(self.get_parameter("odom_frame").value)
        base_frame = str(self.get_parameter("base_frame").value)
        orientation = yaw_to_quaternion(self.yaw)
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = odom_frame
        odom.child_frame_id = base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = orientation
        odom.twist.twist.linear.x = self.last_linear
        odom.twist.twist.angular.z = self.last_angular
        self.odom_publisher.publish(odom)

        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation = orientation
        self.tf_broadcaster.sendTransform(transform)

    def destroy_node(self) -> None:
        self._stop()
        if self.car is not None:
            try:
                self.car.close()
            except Exception:
                pass
        super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ChassisNode()
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
