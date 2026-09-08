#!/usr/bin/env python3
"""Local HTTP dashboard and ROS 2 bridge.

The HTTP server binds to loopback by default. Hardware actions are explicit POST
requests; merely opening the page never moves either robot.
"""

import json
import math
import os
import signal
import subprocess
import threading
import time
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional
from urllib.parse import unquote, urlparse

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, JointState
from std_msgs.msg import Bool, Int32, String


class ManagedProcess:
    def __init__(self, name: str, command: list[str], logs: deque):
        self.name = name
        self.command = command
        self.logs = logs
        self.process: Optional[subprocess.Popen] = None
        self.lock = threading.Lock()

    def running(self) -> bool:
        with self.lock:
            return self.process is not None and self.process.poll() is None

    def start(self) -> tuple[bool, str]:
        with self.lock:
            if self.process is not None and self.process.poll() is None:
                return False, f"{self.name} 已在运行"
            try:
                self.process = subprocess.Popen(
                    self.command,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                    start_new_session=True,
                )
            except OSError as exc:
                self.process = None
                return False, f"{self.name} 启动失败：{exc}"
            process = self.process
            self.logs.append(f"[{self.name}] 启动：{' '.join(self.command)}")
            threading.Thread(
                target=self._drain_output, args=(process,), daemon=True
            ).start()
            return True, f"{self.name} 启动指令已发送"

    def _drain_output(self, process: subprocess.Popen) -> None:
        if process.stdout is not None:
            for line in process.stdout:
                self.logs.append(f"[{self.name}] {line.rstrip()}")
        code = process.wait()
        self.logs.append(f"[{self.name}] 进程退出，code={code}")

    def stop(self) -> tuple[bool, str]:
        with self.lock:
            process = self.process
            if process is None or process.poll() is not None:
                return False, f"{self.name} 未运行"
            try:
                os.killpg(process.pid, signal.SIGINT)
                threading.Thread(
                    target=self._force_stop_after_timeout,
                    args=(process,),
                    daemon=True,
                ).start()
            except ProcessLookupError:
                pass
            return True, f"{self.name} 停止指令已发送"

    def _force_stop_after_timeout(self, process: subprocess.Popen) -> None:
        try:
            process.wait(timeout=8.0)
        except subprocess.TimeoutExpired:
            self.logs.append(f"[{self.name}] SIGINT 超时，发送 SIGTERM")
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass


class DashboardNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_dashboard")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8080)
        self.declare_parameter("workspace_root", "/home/zheng/person_follow_ws")
        self.declare_parameter("control_enabled", True)
        self.control_enabled = bool(self.get_parameter("control_enabled").value)
        self.state_lock = threading.Lock()
        self.image_condition = threading.Condition()
        self.joint_condition = threading.Condition()
        self.images = {"follow": None, "arm": None}
        self.image_seq = {"follow": 0, "arm": 0}
        self.image_time = {"follow": 0.0, "arm": 0.0}
        self.logs = deque(maxlen=120)
        self.cmd_vel = {"linear": 0.0, "angular": 0.0, "updated": 0.0}
        self.odom = {"x": 0.0, "y": 0.0, "yaw": 0.0, "updated": 0.0}
        self.joints = {"names": [], "positions": [], "updated": 0.0}
        self.joint_stream_seq = 0
        self.joint_stream_time = 0.0
        self.joint_stream_data = None
        self.end_pose = {
            "position": [0.0, 0.0, 0.0],
            "orientation": [0.0, 0.0, 0.0, 1.0],
            "updated": 0.0,
        }
        self.grasp_status = "机械臂未启动"
        self.grasp_result = 0
        self.arm_stopping = False
        self.arm_shutdown_ready = threading.Event()

        root = str(self.get_parameter("workspace_root").value)
        self.follower_process = ManagedProcess(
            "人员跟随", ["bash", f"{root}/scripts/start_follow.sh", "show_window:=false"], self.logs
        )
        self.arm_process = ManagedProcess(
            "机械臂",
            ["ros2", "launch", "graspgen_bringup", "system.launch.py", "start_rviz:=false"],
            self.logs,
        )

        self.zero_velocity_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.task_publisher = self.create_publisher(Int32, "/vision_task", 5)
        self.ready_publisher = self.create_publisher(Bool, "/arm_ready", 5)
        self.shutdown_publisher = self.create_publisher(Bool, "/arm_shutdown", 5)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_callback, 10)
        self.create_subscription(Odometry, "/odom", self._odom_callback, 10)
        self.create_subscription(
            CompressedImage,
            "/person_follower/debug/compressed",
            self._follow_image_callback,
            2,
        )
        # graspgen_perception publishes with rclcpp::SensorDataQoS (Best Effort).
        # A default Reliable subscription is incompatible and receives no frames.
        self.create_subscription(
            Image,
            "/detection_result",
            self._arm_image_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(JointState, "/joint_states", self._joint_callback, 10)
        self.create_subscription(Pose, "/end_pose", self._end_pose_callback, 10)
        self.create_subscription(String, "/grasp_status", self._status_callback, 10)
        self.create_subscription(Int32, "/vision_result", self._result_callback, 10)
        self.create_subscription(
            Bool, "/arm_shutdown_ready", self._shutdown_ready_callback, 5
        )

        web_root = Path(get_package_share_directory("robot_dashboard")) / "web"
        piper_model_root = Path(get_package_share_directory("piper_description"))
        handler = self._make_handler(web_root, piper_model_root)
        self.http_server = ThreadingHTTPServer(
            (str(self.get_parameter("host").value), int(self.get_parameter("port").value)),
            handler,
        )
        self.http_thread = threading.Thread(
            target=self.http_server.serve_forever, name="dashboard-http", daemon=True
        )
        self.http_thread.start()
        self.get_logger().info(
            f"控制台已启动：http://{self.get_parameter('host').value}:{self.get_parameter('port').value}"
        )

    def _cmd_callback(self, msg: Twist) -> None:
        with self.state_lock:
            self.cmd_vel = {
                "linear": self._finite(msg.linear.x),
                "angular": self._finite(msg.angular.z),
                "updated": time.monotonic(),
            }

    def _odom_callback(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self.state_lock:
            self.odom = {
                "x": self._finite(msg.pose.pose.position.x),
                "y": self._finite(msg.pose.pose.position.y),
                "yaw": self._finite(yaw),
                "updated": time.monotonic(),
            }

    def _follow_image_callback(self, msg: CompressedImage) -> None:
        self._set_image("follow", bytes(msg.data))

    def _arm_image_callback(self, msg: Image) -> None:
        try:
            frame = self._ros_image_to_bgr(msg)
            ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 82])
            if ok:
                self._set_image("arm", encoded.tobytes())
        except (ValueError, TypeError) as exc:
            self.get_logger().warning(f"机械臂画面转换失败：{exc}")

    @staticmethod
    def _ros_image_to_bgr(msg: Image):
        channels = {"bgr8": 3, "rgb8": 3, "bgra8": 4, "rgba8": 4, "mono8": 1}
        if msg.encoding not in channels:
            raise ValueError(f"不支持的图像编码 {msg.encoding}")
        count = channels[msg.encoding]
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        rows = raw.reshape((msg.height, msg.step))[:, : msg.width * count]
        frame = rows.reshape((msg.height, msg.width, count)) if count > 1 else rows.reshape((msg.height, msg.width))
        if msg.encoding == "rgb8":
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if msg.encoding == "rgba8":
            return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        if msg.encoding == "bgra8":
            return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        if msg.encoding == "mono8":
            return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        return frame

    def _set_image(self, kind: str, data: bytes) -> None:
        with self.image_condition:
            self.images[kind] = data
            self.image_seq[kind] += 1
            self.image_time[kind] = time.monotonic()
            self.image_condition.notify_all()

    def _joint_callback(self, msg: JointState) -> None:
        names = list(msg.name)
        positions = [self._finite(value) for value in msg.position]
        updated = time.monotonic()
        with self.state_lock:
            self.joints = {
                "names": names,
                "positions": positions,
                "updated": updated,
            }
        # The Piper driver can publish much faster than a browser needs. Keep
        # only the newest sample and wake SSE clients at no more than 30 Hz.
        if updated - self.joint_stream_time >= 1.0 / 30.0:
            with self.joint_condition:
                self.joint_stream_time = updated
                self.joint_stream_seq += 1
                self.joint_stream_data = {
                    "names": names,
                    "positions": positions,
                    "server_time": time.time(),
                }
                self.joint_condition.notify_all()

    def _end_pose_callback(self, msg: Pose) -> None:
        with self.state_lock:
            self.end_pose = {
                "position": [
                    self._finite(msg.position.x), self._finite(msg.position.y),
                    self._finite(msg.position.z),
                ],
                "orientation": [
                    self._finite(msg.orientation.x), self._finite(msg.orientation.y),
                    self._finite(msg.orientation.z), self._finite(msg.orientation.w, 1.0),
                ],
                "updated": time.monotonic(),
            }

    def _status_callback(self, msg: String) -> None:
        with self.state_lock:
            self.grasp_status = msg.data

    def _result_callback(self, msg: Int32) -> None:
        with self.state_lock:
            self.grasp_result = int(msg.data)

    def _shutdown_ready_callback(self, msg: Bool) -> None:
        if msg.data:
            self.arm_shutdown_ready.set()

    @staticmethod
    def _age(updated: float) -> Optional[float]:
        return round(time.monotonic() - updated, 2) if updated else None

    @staticmethod
    def _finite(value, default: float = 0.0) -> float:
        numeric = float(value)
        return numeric if math.isfinite(numeric) else float(default)

    @staticmethod
    def _direction(linear: float, angular: float) -> str:
        if abs(linear) < 0.015 and abs(angular) < 0.03:
            return "停止"
        turn = "左转" if angular > 0.03 else "右转" if angular < -0.03 else ""
        move = "前进" if linear > 0.015 else "后退" if linear < -0.015 else ""
        return "并".join(part for part in (move, turn) if part)

    def status_snapshot(self) -> dict:
        with self.state_lock:
            cmd = dict(self.cmd_vel)
            odom = dict(self.odom)
            joints = {"names": list(self.joints["names"]), "positions": list(self.joints["positions"]), "updated": self.joints["updated"]}
            end_pose = {"position": list(self.end_pose["position"]), "orientation": list(self.end_pose["orientation"]), "updated": self.end_pose["updated"]}
            grasp_status = self.grasp_status
            grasp_result = self.grasp_result
        return {
            "follow": {
                "running": self.follower_process.running(),
                "linear": round(cmd["linear"], 4),
                "angular": round(cmd["angular"], 4),
                "direction": self._direction(cmd["linear"], cmd["angular"]),
                "command_age": self._age(cmd["updated"]),
                "odom": {"x": round(odom["x"], 4), "y": round(odom["y"], 4), "yaw_deg": round(math.degrees(odom["yaw"]), 1), "age": self._age(odom["updated"])},
                "image_age": self._age(self.image_time["follow"]),
            },
            "arm": {
                "running": self.arm_process.running(),
                "stopping": self.arm_stopping,
                "joint_names": joints["names"],
                "joint_positions": [round(value, 5) for value in joints["positions"]],
                "joint_age": self._age(joints["updated"]),
                "end_pose": {"position": [round(value, 4) for value in end_pose["position"]], "orientation": [round(value, 4) for value in end_pose["orientation"]], "age": self._age(end_pose["updated"])},
                "grasp_status": grasp_status,
                "grasp_result": grasp_result,
                "image_age": self._age(self.image_time["arm"]),
            },
            "logs": list(self.logs)[-30:],
            "server_time": time.time(),
        }

    def perform_action(self, action: str) -> tuple[bool, str]:
        if not self.control_enabled:
            return False, "当前控制台处于只读预览模式"
        if action == "follow/start":
            return self.follower_process.start()
        if action == "follow/stop":
            self._publish_zero_velocity()
            if self.follower_process.running():
                return self.follower_process.stop()
            return True, "跟随未运行；零速度已发送"
        if action == "arm/start":
            if self.arm_stopping:
                return False, "机械臂正在回零并停止，请稍候"
            return self.arm_process.start()
        if action == "arm/stop":
            if not self.arm_process.running():
                return True, "机械臂节点未运行"
            with self.state_lock:
                if self.arm_stopping:
                    return False, "机械臂正在回零并停止，请勿重复操作"
                self.arm_stopping = True
                self.grasp_status = "正在取消任务并返回机械臂零点"
            self.arm_shutdown_ready.clear()
            self.shutdown_publisher.publish(Bool(data=True))
            threading.Thread(
                target=self._finish_arm_stop_after_zero, daemon=True
            ).start()
            return True, "已发送回零指令；确认到达零点后将自动停止节点"
        if action == "arm/ready":
            if not self.arm_process.running():
                return False, "请先启动机械臂"
            self.ready_publisher.publish(Bool(data=True))
            return True, "已发送准备姿态指令"
        if action == "grasp/start":
            if not self.arm_process.running():
                return False, "请先启动机械臂"
            if self.follower_process.running():
                self._publish_zero_velocity()
                self.follower_process.stop()
                with self.state_lock:
                    self.grasp_status = "正在停止底盘，停止后自动抓取"
                threading.Thread(
                    target=self._start_grasp_after_follow_stop, daemon=True
                ).start()
                return True, "安全联锁：正在停止跟随，底盘停稳后自动抓取"
            with self.state_lock:
                recent_external_motion = (
                    self.cmd_vel["updated"]
                    and time.monotonic() - self.cmd_vel["updated"] < 1.0
                    and (
                        abs(self.cmd_vel["linear"]) >= 0.015
                        or abs(self.cmd_vel["angular"]) >= 0.03
                    )
                )
            if recent_external_motion:
                self._publish_zero_velocity()
                return False, "检测到外部速度指令；已置零，请先停止外部底盘节点"
            self.task_publisher.publish(Int32(data=1))
            return True, "已发送 bottle 抓取指令"
        if action == "grasp/cancel":
            self.task_publisher.publish(Int32(data=0))
            return True, "已发送抓取取消指令"
        return False, "未知操作"

    def _finish_arm_stop_after_zero(self) -> None:
        reached = self.arm_shutdown_ready.wait(timeout=15.0)
        if reached:
            self.logs.append("[机械臂] 已收到零点到达确认，正在停止节点")
            self.arm_process.stop()
            with self.state_lock:
                self.grasp_status = "机械臂已回零，节点停止中"
                self.arm_stopping = False
            return
        with self.state_lock:
            self.grasp_status = "回零超时：为安全起见未停止节点，请检查关节反馈"
            self.arm_stopping = False
        self.logs.append("[机械臂] 15 秒内未收到零点确认；节点保持运行")

    def _publish_zero_velocity(self) -> None:
        for _ in range(3):
            self.zero_velocity_publisher.publish(Twist())

    def _start_grasp_after_follow_stop(self) -> None:
        deadline = time.monotonic() + 10.0
        while self.follower_process.running() and time.monotonic() < deadline:
            time.sleep(0.1)
        self._publish_zero_velocity()
        if self.follower_process.running():
            with self.state_lock:
                self.grasp_status = "安全联锁失败：跟随进程未停止，未执行抓取"
            self.logs.append("[安全联锁] 跟随进程未在 10 秒内停止；抓取已取消")
            return
        self.task_publisher.publish(Int32(data=1))
        with self.state_lock:
            self.grasp_status = "底盘已停止，抓取指令已发送"
        self.logs.append("[安全联锁] 底盘已停止，开始 bottle 抓取")

    def wait_for_image(self, kind: str, sequence: int, timeout: float = 2.0):
        with self.image_condition:
            self.image_condition.wait_for(lambda: self.image_seq[kind] != sequence, timeout)
            return self.image_seq[kind], self.images[kind]

    def wait_for_joints(self, sequence: int, timeout: float = 2.0):
        with self.joint_condition:
            self.joint_condition.wait_for(
                lambda: self.joint_stream_seq != sequence, timeout
            )
            data = dict(self.joint_stream_data) if self.joint_stream_data else None
            if data:
                data["names"] = list(data["names"])
                data["positions"] = list(data["positions"])
            return self.joint_stream_seq, data

    def _make_handler(self, web_root: Path, piper_model_root: Path):
        dashboard = self

        class Handler(BaseHTTPRequestHandler):
            server_version = "RobotDashboard/1.0"
            protocol_version = "HTTP/1.1"

            def log_message(self, _format, *_args):
                return

            def _json(self, data, status=HTTPStatus.OK):
                body = json.dumps(
                    data, ensure_ascii=False, allow_nan=False
                ).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                path = urlparse(self.path).path
                if path == "/api/status":
                    self._json(dashboard.status_snapshot())
                    return
                if path in ("/stream/follow.mjpg", "/stream/arm.mjpg"):
                    self._stream("follow" if "follow" in path else "arm")
                    return
                if path == "/stream/joints.sse":
                    self._joint_stream()
                    return
                if path == "/model/piper_description.urdf":
                    self._file(piper_model_root / "urdf" / "piper_description.urdf", "application/xml; charset=utf-8")
                    return
                if path.startswith("/model/meshes/"):
                    mesh_name = unquote(path[len("/model/meshes/"):])
                    # colcon --symlink-install links each mesh back to src/, so
                    # resolve()-based containment would reject valid package files.
                    # A single basename plus an STL suffix is sufficient here.
                    if Path(mesh_name).name != mesh_name or Path(mesh_name).suffix.lower() != ".stl":
                        self.send_error(HTTPStatus.NOT_FOUND)
                        return
                    candidate = piper_model_root / "meshes" / mesh_name
                    self._file(candidate, "model/stl", cache="public, max-age=3600")
                    return
                files = {
                    "/": "index.html",
                    "/app.js": "app.js",
                    "/style.css": "style.css",
                    "/urdf_viewer.js": "urdf_viewer.js",
                    "/urdf_viewer.bundle.js": "urdf_viewer.bundle.js",
                    "/vendor/three.module.min.js": "vendor/three.module.min.js",
                    "/vendor/STLLoader.js": "vendor/STLLoader.js",
                    "/vendor/OrbitControls.js": "vendor/OrbitControls.js",
                }
                filename = files.get(path)
                if filename is None:
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return
                content_type = "text/html; charset=utf-8" if filename.endswith(".html") else "text/javascript; charset=utf-8" if filename.endswith(".js") else "text/css; charset=utf-8"
                self._file(web_root / filename, content_type)

            def _file(self, filename: Path, content_type: str, cache="no-store"):
                try:
                    body = filename.read_bytes()
                except (FileNotFoundError, IsADirectoryError):
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", cache)
                self.end_headers()
                self.wfile.write(body)

            def do_POST(self):
                path = urlparse(self.path).path
                prefix = "/api/"
                if not path.startswith(prefix):
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return
                ok, message = dashboard.perform_action(path[len(prefix):])
                self._json({"ok": ok, "message": message}, HTTPStatus.OK if ok else HTTPStatus.CONFLICT)

            def _stream(self, kind: str):
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                sequence = -1
                try:
                    while True:
                        sequence, image = dashboard.wait_for_image(kind, sequence)
                        if image is None:
                            continue
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(image)}\r\n\r\n".encode())
                        self.wfile.write(image)
                        self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    return

            def _joint_stream(self):
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/event-stream; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Connection", "keep-alive")
                self.send_header("X-Accel-Buffering", "no")
                self.end_headers()
                sequence = -1
                try:
                    while True:
                        next_sequence, data = dashboard.wait_for_joints(sequence)
                        if next_sequence == sequence or data is None:
                            self.wfile.write(b": keepalive\n\n")
                        else:
                            sequence = next_sequence
                            payload = json.dumps(
                                data, ensure_ascii=False, allow_nan=False
                            ).encode("utf-8")
                            self.wfile.write(b"data: " + payload + b"\n\n")
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    return

        return Handler

    def destroy_node(self) -> None:
        if self.control_enabled:
            if rclpy.ok():
                for _ in range(3):
                    self.zero_velocity_publisher.publish(Twist())
                self.task_publisher.publish(Int32(data=0))
            self.follower_process.stop()
            self.arm_process.stop()
        self.http_server.shutdown()
        self.http_server.server_close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = DashboardNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
