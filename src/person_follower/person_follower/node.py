#!/usr/bin/env python3
"""Dahua NetSDK, YOLOv8 and BoT-SORT ROS 2 person-following node."""

import os
import signal
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from .controller import ControlConfig, TargetLock, compute_velocity


class PersonFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__("person_follower_node")
        self._declare_parameters()
        self._validate_parameters()
        self.publisher = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self.debug_image_publisher = self.create_publisher(
            CompressedImage,
            str(self.get_parameter("debug_image_topic").value),
            2,
        )
        self.camera = None
        self.model = None
        self.last_frame_version = -1
        self.last_command = (0.0, 0.0)
        self.no_frame_stop_sent = False
        self.metrics_started = time.monotonic()
        self.metrics_frames = 0
        self.last_inference_ms = 0.0
        self.last_display_age_ms = 0.0

        self.control = ControlConfig(
            target_width_ratio=float(self.get_parameter("target_width_ratio").value),
            linear_gain=float(self.get_parameter("linear_gain").value),
            angular_gain=float(self.get_parameter("angular_gain").value),
            width_deadzone_ratio=float(self.get_parameter("width_deadzone_ratio").value),
            center_deadzone_ratio=float(self.get_parameter("center_deadzone_ratio").value),
            max_forward_speed=float(self.get_parameter("max_forward_speed").value),
            max_reverse_speed=float(self.get_parameter("max_reverse_speed").value),
            max_angular_speed=float(self.get_parameter("max_angular_speed").value),
            allow_reverse=bool(self.get_parameter("allow_reverse").value),
        )
        self.target_lock = TargetLock(int(self.get_parameter("lost_limit_frames").value))
        self._open_camera()
        self._load_model()
        self.timer = self.create_timer(
            1.0 / float(self.get_parameter("control_rate").value), self._tick
        )
        self.get_logger().info(
            f"人员跟随节点就绪：大华 NetSDK + PlaySDK + YOLOv8 + BoT-SORT，device={self.device}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("camera_ip", os.environ.get("DAHUA_IP", "192.168.1.21"))
        self.declare_parameter("camera_port", 37777)
        self.declare_parameter("camera_user", os.environ.get("DAHUA_USER", "admin"))
        self.declare_parameter(
            "camera_password", os.environ.get("DAHUA_PASSWORD", "123456baoye!!")
        )
        self.declare_parameter("camera_channel", 0)
        self.declare_parameter("camera_stream", "extra")
        self.declare_parameter("model_path", os.environ.get("YOLO_MODEL", "yolov8n.pt"))
        self.declare_parameter("tracker", "botsort.yaml")
        self.declare_parameter("device", "auto")
        self.declare_parameter("confidence", 0.20)
        self.declare_parameter("image_size", 640)
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("frame_timeout", 0.7)
        self.declare_parameter("max_control_latency", 0.50)
        self.declare_parameter("lost_limit_frames", 15)
        self.declare_parameter("target_width_ratio", 0.385)
        self.declare_parameter("linear_gain", 0.65)
        self.declare_parameter("angular_gain", 1.10)
        self.declare_parameter("width_deadzone_ratio", 0.10)
        self.declare_parameter("center_deadzone_ratio", 0.05)
        self.declare_parameter("max_forward_speed", 0.35)
        self.declare_parameter("max_reverse_speed", 0.12)
        self.declare_parameter("max_angular_speed", 0.80)
        self.declare_parameter("allow_reverse", False)
        self.declare_parameter("smoothing_alpha", 0.35)
        self.declare_parameter("show_window", False)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("debug_image_topic", "/person_follower/debug/compressed")
        self.declare_parameter("debug_jpeg_quality", 80)

    def _validate_parameters(self) -> None:
        target_ratio = float(self.get_parameter("target_width_ratio").value)
        if not 0.02 <= target_ratio <= 0.95:
            raise ValueError("target_width_ratio must be in [0.02, 0.95]")
        alpha = float(self.get_parameter("smoothing_alpha").value)
        if not 0.0 < alpha <= 1.0:
            raise ValueError("smoothing_alpha must be in (0, 1]")
        if float(self.get_parameter("control_rate").value) <= 0.0:
            raise ValueError("control_rate must be positive")
        if float(self.get_parameter("max_control_latency").value) <= 0.0:
            raise ValueError("max_control_latency must be positive")
        if str(self.get_parameter("camera_stream").value) not in ("main", "extra"):
            raise ValueError("camera_stream must be 'main' or 'extra'")
        jpeg_quality = int(self.get_parameter("debug_jpeg_quality").value)
        if not 20 <= jpeg_quality <= 100:
            raise ValueError("debug_jpeg_quality must be in [20, 100]")

    def _camera_event(self, message: str, is_error: bool) -> None:
        (self.get_logger().error if is_error else self.get_logger().info)(message)

    def _open_camera(self) -> None:
        try:
            from .dahua_camera import DahuaCamera
        except (ImportError, OSError, AttributeError) as exc:
            raise RuntimeError(
                "大华 NetSDK/PlaySDK 加载失败。请安装 vendor 中的 Linux wheel，"
                "并在启动前 source scripts/netsdk_env.sh"
            ) from exc
        camera = None
        try:
            configured_password = str(self.get_parameter("camera_password").value)
            camera_ip = os.environ.get(
                "DAHUA_IP", str(self.get_parameter("camera_ip").value)
            )
            camera_port = int(
                os.environ.get(
                    "DAHUA_PORT", str(self.get_parameter("camera_port").value)
                )
            )
            camera_user = os.environ.get(
                "DAHUA_USER", str(self.get_parameter("camera_user").value)
            )
            camera = DahuaCamera(
                ip=camera_ip,
                port=camera_port,
                username=camera_user,
                password=os.environ.get("DAHUA_PASSWORD", configured_password),
                channel=int(self.get_parameter("camera_channel").value),
                stream=str(self.get_parameter("camera_stream").value),
                event_callback=self._camera_event,
            )
            camera.login()
            camera.start()
            self.camera = camera
        except Exception:
            if camera is not None:
                camera.close()
            raise

    def _load_model(self) -> None:
        try:
            import torch
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError("未安装 ultralytics/torch；请运行 scripts/install_dependencies.sh") from exc
        requested = str(self.get_parameter("device").value)
        self.device = "cuda:0" if requested == "auto" and torch.cuda.is_available() else requested
        if self.device == "auto":
            self.device = "cpu"
        if self.device.startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError("配置要求 CUDA，但 torch.cuda.is_available() 为 false")
        self.model = YOLO(str(self.get_parameter("model_path").value))
        self.model.to(self.device)
        image_size = int(self.get_parameter("image_size").value)
        warmup_started = time.monotonic()
        self.model.predict(
            np.zeros((image_size, image_size, 3), dtype=np.uint8),
            classes=[0],
            imgsz=image_size,
            device=self.device,
            verbose=False,
        )
        self.get_logger().info(
            f"YOLO 预热完成，耗时 {(time.monotonic() - warmup_started) * 1000.0:.0f} ms"
        )

    def _publish(self, linear: float, angular: float, immediate: bool = False) -> None:
        if immediate:
            filtered = (linear, angular)
        else:
            alpha = float(self.get_parameter("smoothing_alpha").value)
            filtered = (
                alpha * linear + (1.0 - alpha) * self.last_command[0],
                alpha * angular + (1.0 - alpha) * self.last_command[1],
            )
        self.last_command = filtered
        msg = Twist()
        msg.linear.x = filtered[0]
        msg.angular.z = filtered[1]
        self.publisher.publish(msg)

    def _stop_for_camera_timeout(self, now: float) -> None:
        _version, received, _drops = self.camera.status()
        timed_out = received is None or now - received >= float(
            self.get_parameter("frame_timeout").value
        )
        if timed_out and not self.no_frame_stop_sent:
            self.get_logger().error("NetSDK 解码帧超时，已发布零速")
            self._publish(0.0, 0.0, immediate=True)
            self.no_frame_stop_sent = True

    def _tick(self) -> None:
        now = time.monotonic()
        camera_frame = self.camera.read_latest(self.last_frame_version)
        if camera_frame is None:
            self._stop_for_camera_timeout(now)
            return
        self.last_frame_version = camera_frame.version
        self.no_frame_stop_sent = False

        inference_started = time.monotonic()
        result = self.model.track(
            camera_frame.image,
            conf=float(self.get_parameter("confidence").value),
            classes=[0],
            persist=True,
            tracker=str(self.get_parameter("tracker").value),
            imgsz=int(self.get_parameter("image_size").value),
            device=self.device,
            verbose=False,
        )[0]
        inference_finished = time.monotonic()
        self.last_inference_ms = (inference_finished - inference_started) * 1000.0
        self.last_display_age_ms = (
            inference_finished - camera_frame.received_monotonic
        ) * 1000.0

        boxes = result.boxes
        ids = []
        xyxy = []
        if boxes is not None and boxes.id is not None:
            ids = boxes.id.cpu().numpy().astype(int).tolist()
            xyxy = boxes.xyxy.cpu().numpy().tolist()

        target_id, target_box = self.target_lock.update(ids, xyxy)
        stale = self.last_display_age_ms > 1000.0 * float(
            self.get_parameter("max_control_latency").value
        )
        if stale:
            target_box = None
            self.get_logger().warning(
                f"本帧处理延迟 {self.last_display_age_ms:.0f} ms 超限，已停车"
            )
        linear, angular = compute_velocity(
            target_box, camera_frame.image.shape[1], self.control
        )
        self._publish(linear, angular, immediate=target_box is None)

        self.metrics_frames += 1
        show_window = bool(self.get_parameter("show_window").value)
        publish_debug = bool(self.get_parameter("publish_debug_image").value)
        if show_window or publish_debug:
            annotated = camera_frame.image.copy()
            self._draw(
                annotated,
                xyxy,
                ids,
                target_id,
                target_box,
                linear,
                angular,
            )
            if publish_debug:
                self._publish_debug_image(annotated)
        if show_window:
            cv2.imshow("person-follow-netsdk", annotated)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.get_logger().info("收到 q，停止跟随")
                rclpy.shutdown()
        self._report_metrics(inference_finished)

    def _publish_debug_image(self, frame) -> None:
        quality = int(self.get_parameter("debug_jpeg_quality").value)
        ok, encoded = cv2.imencode(
            ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality]
        )
        if not ok:
            self.get_logger().warning("人员跟随调试画面 JPEG 编码失败")
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = encoded.tobytes()
        self.debug_image_publisher.publish(msg)

    def _report_metrics(self, now: float) -> None:
        elapsed = now - self.metrics_started
        if elapsed < 1.0:
            return
        _version, _received, drops = self.camera.status()
        fps = self.metrics_frames / elapsed
        self.get_logger().info(
            f"处理 {fps:.1f} FPS，推理 {self.last_inference_ms:.0f} ms，"
            f"当前帧龄 {self.last_display_age_ms:.0f} ms，PlaySDK 输入丢弃 {drops}"
        )
        self.metrics_started = now
        self.metrics_frames = 0

    def _draw(self, frame, boxes, ids, target_id, target_box, linear, angular) -> None:
        for track_id, box in zip(ids, boxes):
            x1, y1, x2, y2 = [int(value) for value in box]
            color = (0, 255, 0) if track_id == target_id else (120, 120, 120)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f"ID {track_id}", (x1, max(20, y1 - 8)), 0, 0.6, color, 2)
        cv2.putText(
            frame,
            f"target={target_id} v={linear:+.2f} w={angular:+.2f}",
            (10, 30), 0, 0.7, (0, 255, 255), 2,
        )
        cv2.putText(
            frame,
            f"inference={self.last_inference_ms:.0f}ms frame_age={self.last_display_age_ms:.0f}ms",
            (10, 58), 0, 0.6, (0, 255, 255), 2,
        )

    def destroy_node(self) -> None:
        try:
            if rclpy.ok():
                self._publish(0.0, 0.0, immediate=True)
        finally:
            try:
                if self.camera is not None:
                    self.camera.close()
            finally:
                cv2.destroyAllWindows()
                super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = PersonFollowerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            # launch may deliver another SIGINT while native PlaySDK cleanup is
            # in progress. Ignore it here so decoder ports and SDK threads are
            # released deterministically; the chassis watchdog is already safe.
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
