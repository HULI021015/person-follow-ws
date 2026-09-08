#!/usr/bin/env python3
"""Ground calibration helper for person_follow_ws chassis.

This script publishes temporary /cmd_vel commands through ROS 2, then asks the
operator to enter physically measured distance / rotation angle. It calculates
actual linear/angular velocity and recommends corrected chassis.yaml scale
parameters.

It DOES NOT modify chassis.yaml automatically.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


DEFAULT_MAX_LINEAR = 1.257
DEFAULT_MAX_ANGULAR = 1.67
DEFAULT_LINEAR_COMMAND = 0.20
DEFAULT_LINEAR_DURATION = 5.0
DEFAULT_ANGULAR_COMMAND = 0.40
DEFAULT_ANGULAR_DURATION = 6.0
DEFAULT_RATE = 20.0


@dataclass
class CalibrationResult:
    commanded: float
    duration: float
    measured_motion: float
    actual_speed: float
    old_max: float
    recommended_max: float


class CmdVelCalibrator(Node):
    def __init__(self, topic: str, rate_hz: float) -> None:
        super().__init__("chassis_ground_calibrator")
        self.publisher = self.create_publisher(Twist, topic, 10)
        self.topic = topic
        self.rate_hz = rate_hz

    def wait_for_chassis(self, timeout: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.publisher.get_subscription_count() > 0:
                return True
        return False

    def other_cmd_vel_publishers(self) -> list[str]:
        others: list[str] = []
        for info in self.get_publishers_info_by_topic(self.topic):
            if info.node_name != self.get_name():
                others.append(f"/{info.node_name}")
        return sorted(set(others))

    def stop(self) -> None:
        msg = Twist()
        for _ in range(10):
            self.publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)

    def run_motion(self, linear: float, angular: float, duration: float) -> float:
        """Publish a motion command for duration seconds and return elapsed time."""
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)

        period = 1.0 / self.rate_hz
        start = time.monotonic()
        next_tick = start
        motion_elapsed = 0.0
        try:
            while rclpy.ok():
                now = time.monotonic()
                if now - start >= duration:
                    break
                self.publisher.publish(msg)
                rclpy.spin_once(self, timeout_sec=0.0)
                next_tick += period
                sleep_time = next_tick - time.monotonic()
                if sleep_time > 0:
                    time.sleep(sleep_time)
            motion_elapsed = time.monotonic() - start
        finally:
            self.stop()
        return motion_elapsed


def positive_float(text: str) -> float:
    value = float(text)
    if value <= 0:
        raise argparse.ArgumentTypeError("must be > 0")
    return value


def ask_yes(prompt: str) -> bool:
    answer = input(prompt).strip().lower()
    return answer in {"y", "yes", "是", "1"}


def ask_positive_measurement(prompt: str) -> float:
    while True:
        raw = input(prompt).strip()
        try:
            value = float(raw)
            if value > 0:
                return value
        except ValueError:
            pass
        print("请输入一个大于 0 的数字。")


def countdown(seconds: int = 3) -> None:
    for n in range(seconds, 0, -1):
        print(f"  {n}...")
        time.sleep(1.0)
    print("  GO")


def calibrate_linear(
    node: CmdVelCalibrator,
    command: float,
    duration: float,
    current_max: float,
) -> CalibrationResult | None:
    print("\n========== 线速度标定 ==========")
    print(f"将发布 linear.x = +{command:.3f} m/s, angular.z = 0")
    print(f"计划持续 {duration:.2f} s，结束后自动发送停车指令。")
    print("请在地面上给车留足直线距离，并标记车体同一个固定点的起点。")
    if not ask_yes("确认场地安全并开始？[y/N]: "):
        print("已跳过线速度测试。")
        return None

    countdown()
    elapsed = node.run_motion(command, 0.0, duration)
    print(f"运动结束。实际发布时长约 {elapsed:.3f} s。")
    distance = ask_positive_measurement(
        "请输入同一车体固定点从起点到终点的实际直线距离（米，例如 1.08）："
    )

    actual_speed = distance / elapsed
    recommended = current_max * actual_speed / abs(command)
    result = CalibrationResult(
        commanded=command,
        duration=elapsed,
        measured_motion=distance,
        actual_speed=actual_speed,
        old_max=current_max,
        recommended_max=recommended,
    )

    print("\n--- 线速度结果 ---")
    print(f"实测距离             : {distance:.4f} m")
    print(f"实测平均线速度       : {actual_speed:.4f} m/s")
    print(f"ROS 命令线速度       : {command:.4f} m/s")
    print(f"当前 max_linear_speed: {current_max:.4f} m/s")
    print(f"建议 max_linear_speed: {recommended:.4f} m/s")
    print("建议至少重复 3 次，取推荐值的平均数后再修改 chassis.yaml。")
    return result


def calibrate_angular(
    node: CmdVelCalibrator,
    command: float,
    duration: float,
    current_max: float,
) -> CalibrationResult | None:
    print("\n========== 角速度标定 ==========")
    print(f"将发布 linear.x = 0, angular.z = +{command:.3f} rad/s（按 ROS 约定应左转）")
    print(f"计划持续 {duration:.2f} s，结束后自动发送停车指令。")
    print("在车体上选一个明确朝向（例如车头中线），并在地面标记初始方向。")
    print("结束后记录累计旋转角度；可以超过 360°，例如两圈半输入 900。")
    if not ask_yes("确认场地安全并开始？[y/N]: "):
        print("已跳过角速度测试。")
        return None

    countdown()
    elapsed = node.run_motion(0.0, command, duration)
    print(f"运动结束。实际发布时长约 {elapsed:.3f} s。")
    degrees = ask_positive_measurement(
        "请输入小车累计实际旋转角度（度；1圈=360，2圈=720）："
    )

    radians = math.radians(degrees)
    actual_speed = radians / elapsed
    recommended = current_max * actual_speed / abs(command)
    result = CalibrationResult(
        commanded=command,
        duration=elapsed,
        measured_motion=degrees,
        actual_speed=actual_speed,
        old_max=current_max,
        recommended_max=recommended,
    )

    print("\n--- 角速度结果 ---")
    print(f"实测累计转角          : {degrees:.2f} deg ({radians:.4f} rad)")
    print(f"实测平均角速度        : {actual_speed:.4f} rad/s")
    print(f"ROS 命令角速度        : {command:.4f} rad/s")
    print(f"当前 max_angular_speed: {current_max:.4f} rad/s")
    print(f"建议 max_angular_speed: {recommended:.4f} rad/s")
    print("建议至少重复 3 次，取推荐值的平均数后再修改 chassis.yaml。")
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="在地面上通过实测距离/转角标定 chassis.yaml 的速度比例。"
    )
    parser.add_argument("--mode", choices=["all", "linear", "angular"], default="all")
    parser.add_argument("--topic", default="/cmd_vel")
    parser.add_argument("--rate", type=positive_float, default=DEFAULT_RATE)
    parser.add_argument("--linear-command", type=positive_float, default=DEFAULT_LINEAR_COMMAND)
    parser.add_argument("--linear-duration", type=positive_float, default=DEFAULT_LINEAR_DURATION)
    parser.add_argument("--angular-command", type=positive_float, default=DEFAULT_ANGULAR_COMMAND)
    parser.add_argument("--angular-duration", type=positive_float, default=DEFAULT_ANGULAR_DURATION)
    parser.add_argument("--current-max-linear", type=positive_float, default=DEFAULT_MAX_LINEAR)
    parser.add_argument("--current-max-angular", type=positive_float, default=DEFAULT_MAX_ANGULAR)
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    print("person_follow_ws 底盘地面速度标定")
    print("--------------------------------")
    print(f"当前默认 max_linear_speed : {args.current_max_linear:.4f} m/s")
    print(f"当前默认 max_angular_speed: {args.current_max_angular:.4f} rad/s")
    print(f"线速度测试命令            : {args.linear_command:.3f} m/s × {args.linear_duration:.1f} s")
    print(f"角速度测试命令            : {args.angular_command:.3f} rad/s × {args.angular_duration:.1f} s")
    print("注意：本脚本不会读取 /odom，也不会自动修改 chassis.yaml。")

    rclpy.init()
    node = CmdVelCalibrator(args.topic, args.rate)
    results: list[tuple[str, CalibrationResult]] = []

    try:
        print(f"\n正在检查 {args.topic} 是否有底盘订阅者...")
        if not node.wait_for_chassis(timeout=5.0):
            print(
                f"错误：5 秒内没有检测到 {args.topic} 订阅者。\n"
                "请先在另一个终端启动 chassis_node，再运行本脚本。",
                file=sys.stderr,
            )
            return 2

        print(f"已检测到订阅者数量：{node.publisher.get_subscription_count()}")
        rclpy.spin_once(node, timeout_sec=0.2)
        other_publishers = node.other_cmd_vel_publishers()
        if other_publishers:
            print(
                "错误：检测到其他 /cmd_vel 发布节点：" + ", ".join(other_publishers) + "\n"
                "请先关闭 person_follower 或其他遥控节点，确保标定时只有本脚本发布 /cmd_vel。",
                file=sys.stderr,
            )
            return 3
        node.stop()

        if args.mode in {"all", "linear"}:
            result = calibrate_linear(
                node,
                args.linear_command,
                args.linear_duration,
                args.current_max_linear,
            )
            if result is not None:
                results.append(("linear", result))

        if args.mode in {"all", "angular"}:
            node.stop()
            print("\n准备角速度测试前，请重新摆正小车并清空周围区域。")
            result = calibrate_angular(
                node,
                args.angular_command,
                args.angular_duration,
                args.current_max_angular,
            )
            if result is not None:
                results.append(("angular", result))

        if results:
            print("\n========== 本次标定汇总 ==========")
            for kind, result in results:
                if kind == "linear":
                    print(
                        f"max_linear_speed : {result.old_max:.4f} -> "
                        f"建议 {result.recommended_max:.4f}"
                    )
                else:
                    print(
                        f"max_angular_speed: {result.old_max:.4f} -> "
                        f"建议 {result.recommended_max:.4f}"
                    )
            print("\n先记录结果。建议重复测试 3 次取平均，再修改 src/vehicle_bringup/config/chassis.yaml。")
        return 0

    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，正在停车...")
        return 130
    finally:
        try:
            node.stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
