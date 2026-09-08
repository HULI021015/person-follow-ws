import json
import math
import threading
import time
import urllib.request

import cv2
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, JointState

from robot_dashboard.dashboard_node import DashboardNode


def test_direction_labels():
    assert DashboardNode._direction(0.0, 0.0) == "停止"
    assert DashboardNode._direction(0.2, 0.0) == "前进"
    assert DashboardNode._direction(0.2, 0.4) == "前进并左转"
    assert DashboardNode._direction(0.0, -0.4) == "右转"
    assert DashboardNode._finite(math.nan) == 0.0
    assert DashboardNode._finite(math.inf, 1.0) == 1.0


def test_ros_bgr_image_with_padded_step():
    message = Image()
    message.height = 2
    message.width = 2
    message.encoding = "bgr8"
    message.step = 8
    message.data = bytes([1, 2, 3, 4, 5, 6, 0, 0, 7, 8, 9, 10, 11, 12, 0, 0])
    frame = DashboardNode._ros_image_to_bgr(message)
    assert frame.shape == (2, 2, 3)
    assert np.array_equal(frame[1, 1], [10, 11, 12])


def test_http_status_smoke():
    rclpy.init(args=["--ros-args", "-p", "port:=0", "-p", "control_enabled:=false"])
    node = DashboardNode()
    try:
        port = node.http_server.server_address[1]
        with urllib.request.urlopen(
            f"http://127.0.0.1:{port}/api/status", timeout=2
        ) as response:
            payload = json.loads(response.read().decode("utf-8"))
        assert payload["follow"]["running"] is False
        assert payload["arm"]["running"] is False
        assert payload["follow"]["direction"] == "停止"
        with urllib.request.urlopen(
            f"http://127.0.0.1:{port}/model/piper_description.urdf", timeout=2
        ) as response:
            urdf = response.read().decode("utf-8")
        assert '<robot\n  name="piper">' in urdf
        with urllib.request.urlopen(
            f"http://127.0.0.1:{port}/model/meshes/base_link.STL", timeout=2
        ) as response:
            assert len(response.read(128)) == 128
        with urllib.request.urlopen(
            f"http://127.0.0.1:{port}/urdf_viewer.bundle.js", timeout=2
        ) as response:
            assert len(response.read()) > 500_000
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_live_stream_and_joint_topics_reach_dashboard():
    """Exercise the same ROS topic/QoS path used by both browser views."""
    rclpy.init(args=["--ros-args", "-p", "port:=0", "-p", "control_enabled:=false"])
    dashboard = DashboardNode()
    source = Node("dashboard_transport_test_source")
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(dashboard)
    executor.add_node(source)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    arm_publisher = source.create_publisher(
        Image, "/detection_result", qos_profile_sensor_data
    )
    follow_publisher = source.create_publisher(
        CompressedImage, "/person_follower/debug/compressed", 2
    )
    joint_publisher = source.create_publisher(JointState, "/joint_states", 10)

    arm_image = Image()
    arm_image.height = 4
    arm_image.width = 4
    arm_image.encoding = "bgr8"
    arm_image.step = 12
    arm_image.data = bytes(4 * 4 * 3)

    ok, jpeg = cv2.imencode(".jpg", np.zeros((4, 4, 3), dtype=np.uint8))
    assert ok
    follow_image = CompressedImage()
    follow_image.format = "jpeg"
    follow_image.data = jpeg.tobytes()

    joints = JointState()
    joints.name = [f"joint{i}" for i in range(1, 7)]
    joints.position = [0.1] * 6

    try:
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            arm_publisher.publish(arm_image)
            follow_publisher.publish(follow_image)
            joint_publisher.publish(joints)
            if (
                dashboard.images["arm"] is not None
                and dashboard.images["follow"] is not None
                and dashboard.joints["names"] == joints.name
                and dashboard.joints["positions"] == list(joints.position)
            ):
                break
            time.sleep(0.05)

        assert dashboard.images["arm"] is not None
        assert dashboard.images["follow"] is not None
        assert dashboard.joints["names"] == joints.name
        assert dashboard.joints["positions"] == list(joints.position)

        port = dashboard.http_server.server_address[1]
        with urllib.request.urlopen(
            f"http://127.0.0.1:{port}/stream/joints.sse", timeout=2
        ) as response:
            event = None
            for _ in range(5):
                line = response.readline()
                if line.startswith(b"data: "):
                    event = json.loads(line[len(b"data: "):])
                    break
        assert event is not None
        assert event["names"] == joints.name
        assert event["positions"] == list(joints.position)
    finally:
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        source.destroy_node()
        dashboard.destroy_node()
        rclpy.shutdown()
