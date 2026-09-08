#!/usr/bin/env python3

import copy
import math
from collections import deque

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Int32, String
from tf2_ros import Buffer, TransformException, TransformListener


class GraspTaskNode(Node):
    """Headless, non-blocking grasp state machine."""

    def __init__(self):
        super().__init__("grasp_task")
        self._declare_parameters()
        self.base_frame = self.get_parameter("base_frame").value
        self.pose_timeout = float(self.get_parameter("pose_timeout").value)
        self.ik_timeout = float(self.get_parameter("ik_timeout").value)
        self.motion_settle = float(
            self.get_parameter("motion_settle_time").value
        )
        self.gripper_settle = float(
            self.get_parameter("gripper_settle_time").value
        )
        self.open_width = float(self.get_parameter("open_width").value)
        self.closed_width = float(self.get_parameter("closed_width").value)
        self.workspace_min = np.array(
            self.get_parameter("workspace_min").value, dtype=float
        )
        self.workspace_max = np.array(
            self.get_parameter("workspace_max").value, dtype=float
        )
        self.enable_workspace_check = bool(
            self.get_parameter("enable_workspace_check").value
        )
        self.auto_execute = bool(self.get_parameter("auto_execute").value)
        self.startup_move_home = bool(
            self.get_parameter("startup_move_home").value
        )
        self.return_home_after_place = bool(
            self.get_parameter("return_home_after_place").value
        )
        self.home_joint_names = list(
            self.get_parameter("home_joint_names").value
        )
        self.home_joint_positions = [
            float(value)
            for value in self.get_parameter("home_joint_positions").value
        ]
        self.home_joint_velocity = [
            float(value)
            for value in self.get_parameter("home_joint_velocity").value
        ]
        self.home_joint_effort = [
            float(value)
            for value in self.get_parameter("home_joint_effort").value
        ]
        self.home_motion_time = float(
            self.get_parameter("home_motion_time").value
        )
        self.home_command_period = float(
            self.get_parameter("home_command_period").value
        )
        self.home_joint_tolerance = float(
            self.get_parameter("home_joint_tolerance").value
        )
        self.home_wait_for_feedback = bool(
            self.get_parameter("home_wait_for_feedback").value
        )
        self.shutdown_joint_positions = [
            float(value)
            for value in self.get_parameter("shutdown_joint_positions").value
        ]
        self.shutdown_motion_time = float(
            self.get_parameter("shutdown_motion_time").value
        )
        self.place_orientation_retry_enabled = bool(
            self.get_parameter("place_orientation_retry_enabled").value
        )
        self.place_yaw_offset = math.radians(
            float(self.get_parameter("place_yaw_offset_deg").value)
        )
        self.place_approach_angle = math.radians(
            float(self.get_parameter("place_approach_angle_deg").value)
        )
        self.keep_gripper_closed_on_abort_after_grasp = bool(
            self.get_parameter("keep_gripper_closed_on_abort_after_grasp").value
        )
        self.sam3_tool_length = float(
            self.get_parameter("sam3_tool_length").value
        )
        self.sam3_penetration = float(
            self.get_parameter("sam3_penetration").value
        )
        self.sam3_approach_angle = math.radians(
            float(self.get_parameter("sam3_approach_angle_deg").value)
        )
        self.sam3_pull_back = float(
            self.get_parameter("sam3_pull_back").value
        )
        self.sam3_lateral_offset = float(
            self.get_parameter("sam3_lateral_offset").value
        )
        self.sam3_pregrasp_distance = float(
            self.get_parameter("sam3_pregrasp_distance").value
        )
        self.sam3_retreat_extra_z = float(
            self.get_parameter("sam3_retreat_extra_z").value
        )
        self.sam3_min_flange_z = float(
            self.get_parameter("sam3_min_flange_z").value
        )
        self.sam3_min_fingertip_z = float(
            self.get_parameter("sam3_min_fingertip_z").value
        )
        self.sam3_finger_half_width = float(
            self.get_parameter("sam3_finger_half_width").value
        )
        self.sam3_max_target_radius = float(
            self.get_parameter("sam3_max_target_radius").value
        )
        self.sam3_yaw_offset = math.radians(
            float(self.get_parameter("sam3_yaw_offset_deg").value)
        )
        self.orientation_retry_enabled = bool(
            self.get_parameter("orientation_retry_enabled").value
        )
        self.max_pitch_adjustment = math.radians(
            float(self.get_parameter("orientation_retry_max_pitch_deg").value)
        )
        self.max_roll_adjustment = math.radians(
            float(self.get_parameter("orientation_retry_max_roll_deg").value)
        )
        self.orientation_retry_spiral_step = math.radians(
            float(self.get_parameter("orientation_retry_spiral_step_deg").value)
        )
        self.orientation_retry_max_candidates = int(
            self.get_parameter("orientation_retry_max_candidates").value
        )
        self.orientation_retry_offsets = self._generate_orientation_retry_offsets()

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target_pub = self.create_publisher(
            PoseStamped, "/target_pose", 5
        )
        self.gripper_pub = self.create_publisher(
            Float64, "/gripper_cmd", 5
        )
        self.joint_cmd_pub = self.create_publisher(
            JointState, "/joint_ctrl_cmd", 5
        )
        self.result_pub = self.create_publisher(Int32, "/vision_result", 5)
        self.status_pub = self.create_publisher(String, "/grasp_status", 5)
        self.shutdown_ready_pub = self.create_publisher(
            Bool, "/arm_shutdown_ready", 5
        )
        self.create_subscription(
            PoseStamped, "/grasp_pose", self._grasp_pose_callback, 5
        )
        self.create_subscription(Bool, "/ik_status", self._ik_callback, 5)
        self.create_subscription(
            Int32, "/vision_task", self._task_callback, 5
        )
        self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10
        )
        self.create_subscription(Bool, "/arm_ready", self._ready_callback, 5)
        self.create_subscription(
            Bool, "/arm_shutdown", self._shutdown_callback, 5
        )

        self.latest_pose = None
        self.latest_pose_time = None
        self.latest_joint_positions = {}
        self.pending_request = False
        self.steps = deque()
        self.waiting_for_ik = False
        self.wait_deadline = None
        self.hold_until = None
        self.current_move = None
        self.current_move_name = "move"
        self.current_move_allow_retry = True
        self.orientation_attempt = 0
        self.gripper_holding_object = False
        self.startup_ready = not self.startup_move_home
        self.home_motion_active = False
        self.home_motion_reason = ""
        self.home_motion_deadline = None
        self.home_next_command = None
        self.active_joint_positions = list(self.home_joint_positions)
        self.active_motion_time = self.home_motion_time
        self.active_motion_require_feedback = self.home_wait_for_feedback
        self.create_timer(0.02, self._tick)
        self.get_logger().info(
            "SAM3 grasp orientation retry: "
            f"retry={self.orientation_retry_enabled}, "
            f"candidates={len(self.orientation_retry_offsets)}"
        )
        self.get_logger().info(
            "Only SAM3-style grasp is enabled; approach_angle="
            f"{math.degrees(self.sam3_approach_angle):.1f} deg"
        )
        if self.startup_move_home:
            self._start_home_motion("startup")

    def _declare_parameters(self):
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("pose_timeout", 0.5)
        self.declare_parameter("ik_timeout", 2.0)
        self.declare_parameter("motion_settle_time", 1.5)
        self.declare_parameter("gripper_settle_time", 0.5)
        self.declare_parameter("place_orientation_retry_enabled", True)
        self.declare_parameter("place_yaw_offset_deg", 0.0)
        self.declare_parameter("place_approach_angle_deg", 45.0)
        self.declare_parameter("keep_gripper_closed_on_abort_after_grasp", True)
        self.declare_parameter("sam3_tool_length", 0.075)
        self.declare_parameter("sam3_penetration", 0.020)
        self.declare_parameter("sam3_approach_angle_deg", 45.0)
        self.declare_parameter("sam3_pull_back", 0.030)
        self.declare_parameter("sam3_lateral_offset", 0.0)
        self.declare_parameter("sam3_pregrasp_distance", 0.10)
        self.declare_parameter("sam3_retreat_extra_z", 0.05)
        self.declare_parameter("sam3_min_flange_z", -0.60)
        self.declare_parameter("sam3_min_fingertip_z", -0.60)
        self.declare_parameter("sam3_finger_half_width", 0.0425)
        self.declare_parameter("sam3_max_target_radius", 0.66)
        self.declare_parameter("sam3_yaw_offset_deg", 0.0)
        self.declare_parameter("orientation_retry_enabled", True)
        self.declare_parameter("orientation_retry_max_pitch_deg", 90.0)
        self.declare_parameter("orientation_retry_max_roll_deg", 45.0)
        self.declare_parameter("orientation_retry_spiral_step_deg", 10.0)
        self.declare_parameter("orientation_retry_max_candidates", 0)
        self.declare_parameter("open_width", 0.07)
        self.declare_parameter("closed_width", 0.0)
        self.declare_parameter("workspace_min", [0.12, -0.45, 0.02])
        self.declare_parameter("workspace_max", [0.65, 0.45, 0.65])
        self.declare_parameter("enable_workspace_check", False)
        self.declare_parameter("place_position", [0.30, 0.0, 0.36])
        self.declare_parameter("auto_execute", False)
        self.declare_parameter("startup_move_home", True)
        self.declare_parameter("return_home_after_place", True)
        self.declare_parameter(
            "home_joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"],
        )
        self.declare_parameter(
            "home_joint_positions",
            [0.0, 0.785, -1.20, 0.0, 0.785, 0.0, 0.07],
        )
        self.declare_parameter(
            "home_joint_velocity", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 60.0]
        )
        self.declare_parameter(
            "home_joint_effort", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5]
        )
        self.declare_parameter("home_motion_time", 5.0)
        self.declare_parameter("home_command_period", 0.5)
        self.declare_parameter("home_joint_tolerance", 0.06)
        self.declare_parameter("home_wait_for_feedback", False)
        self.declare_parameter(
            "shutdown_joint_positions",
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.07],
        )
        self.declare_parameter("shutdown_motion_time", 10.0)

    def _grasp_pose_callback(self, msg):
        transformed = self._transform_pose(msg)
        if transformed is None:
            return

        if self.enable_workspace_check and not self._inside_workspace(transformed):
            return
        self.latest_pose = transformed
        self.latest_pose_time = self.get_clock().now()
        if (
            self.auto_execute
            and self.startup_ready
            and not self.steps
            and not self.waiting_for_ik
            and not self.home_motion_active
        ):
            self.pending_request = True

    def _task_callback(self, msg):
        if msg.data == 1:
            if (
                self.startup_ready
                and (
                    self.pending_request
                    or self.steps
                    or self.waiting_for_ik
                    or self.home_motion_active
                )
            ):
                self._publish_status("Grasp request rejected: task is busy")
                self.get_logger().warn("Duplicate grasp request rejected")
                return
            self.pending_request = True
            if not self.startup_ready:
                self.get_logger().info(
                    "Task request received; waiting for startup home pose"
                )
        elif msg.data == 0:
            self._abort("cancelled")

    def _ready_callback(self, msg):
        if not msg.data:
            return
        if (
            self.steps
            or self.waiting_for_ik
            or self.pending_request
            or self.gripper_holding_object
        ):
            self._publish_status("Ready request rejected: grasp task is busy")
            self.get_logger().warn(
                "Ready request rejected while a grasp task is active"
            )
            return
        self.steps.clear()
        self.pending_request = False
        self.waiting_for_ik = False
        self.hold_until = None
        self._publish_status("Moving to ready pose")
        self._start_home_motion("manual")

    def _shutdown_callback(self, msg):
        if not msg.data:
            return
        self.steps.clear()
        self.pending_request = False
        self.waiting_for_ik = False
        self.hold_until = None
        self.current_move = None
        self.current_move_name = "move"
        self.current_move_allow_retry = True
        target = list(self.shutdown_joint_positions)
        if self.gripper_holding_object and len(target) >= 7:
            target[6] = self.closed_width
            self.get_logger().warn(
                "Shutdown requested while holding an object; keeping gripper closed"
            )
        self.shutdown_ready_pub.publish(Bool(data=False))
        self._publish_result(-1)
        self._publish_status("Moving to zero pose before shutdown")
        self._start_joint_motion(
            "shutdown", target, self.shutdown_motion_time, True
        )

    def _joint_state_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.latest_joint_positions[name] = float(position)

    def _ik_callback(self, msg):
        if not self.waiting_for_ik:
            return
        if msg.data:
            self.waiting_for_ik = False
            self.current_move = None
            self.hold_until = (
                self.get_clock().now()
                + Duration(seconds=self.motion_settle)
            )
            return
        if not self.orientation_retry_enabled or not self.current_move_allow_retry:
            self._abort(
                f"IK failed for {self.current_move_name} target pose. "
                "Orientation retry is disabled for this move."
            )
            return
        self.orientation_attempt += 1
        if self.orientation_attempt >= len(self.orientation_retry_offsets):
            self._abort(
                "IK failed for all orientation candidates. "
                "No workspace pre-filter is active; the target may be "
                "outside the reachable arm workspace or the pose orientation "
                "may be unreachable."
            )
            return
        self._publish_move_candidate()

    def _tick(self):
        now = self.get_clock().now()
        if self.home_motion_active:
            self._tick_home_motion(now)
            if self.home_motion_active:
                return

        if self.pending_request and not self.startup_ready:
            return

        if self.pending_request and not self.steps and not self.waiting_for_ik:
            self.pending_request = False
            if not self._pose_is_fresh(now):
                self._publish_result(-1)
                self._publish_status("No fresh grasp pose; task rejected")
                self.get_logger().warn("No fresh grasp pose; task rejected")
            else:
                self._build_sequence()

        if self.waiting_for_ik:
            if now > self.wait_deadline:
                self._abort("IK response timeout")
            return
        if self.hold_until is not None:
            if now < self.hold_until:
                return
            self.hold_until = None
        if not self.steps:
            return

        kind, value = self.steps.popleft()
        if kind == "gripper":
            gripper_value = float(value)
            self.gripper_pub.publish(Float64(data=gripper_value))
            if gripper_value <= self.closed_width + 1e-4:
                self.gripper_holding_object = True
            elif gripper_value >= self.open_width - 1e-4:
                self.gripper_holding_object = False
            self.hold_until = (
                now + Duration(seconds=self.gripper_settle)
            )
        elif kind == "move":
            move = self._parse_move_step(value)
            self.current_move = copy.deepcopy(move["pose"])
            self.current_move_name = move["name"]
            self.current_move_allow_retry = move["allow_retry"]
            self.orientation_attempt = 0
            self.waiting_for_ik = True
            self.wait_deadline = now + Duration(seconds=self.ik_timeout)
            self._publish_move_candidate()
        elif kind == "home":
            self._start_home_motion("return")
        elif kind == "done":
            self._publish_result(1)
            self._publish_status("Grasp sequence completed")
            self.get_logger().info("Grasp sequence completed")

    def _build_sequence(self):
        object_pose = copy.deepcopy(self.latest_pose)
        place = PoseStamped()
        place.header.frame_id = self.base_frame
        place_pos = self.get_parameter("place_position").value
        place.pose.position.x, place.pose.position.y, place.pose.position.z = (
            float(place_pos[0]), float(place_pos[1]), float(place_pos[2])
        )
        self._apply_place_orientation(place)
        self._build_sam3_style_sequence(object_pose, place)

    def _build_sam3_style_sequence(self, object_pose, place):
        plan = self._make_sam3_style_grasp_plan(object_pose)
        if plan is None:
            return False

        if self.enable_workspace_check:
            for pose in (
                plan["pregrasp"],
                plan["grasp"],
                plan["retreat"],
                place,
            ):
                if not self._inside_workspace(pose):
                    self._abort("sam3_style sequence pose outside workspace")
                    return False

        sequence = [
            ("gripper", self.open_width),
            ("move", self._make_move_step("sam3_pregrasp", plan["pregrasp"], True)),
            ("move", self._make_move_step("sam3_grasp", plan["grasp"], True)),
            ("gripper", self.closed_width),
            ("move", self._make_move_step("sam3_retreat", plan["retreat"], False)),
            ("move", self._make_move_step(
                "place", place, self.place_orientation_retry_enabled
            )),
            ("gripper", self.open_width),
        ]
        if self.return_home_after_place:
            sequence.append(("home", None))
        sequence.append(("done", None))
        self.steps.extend(sequence)
        self._publish_result(2)
        self._publish_status("SAM3-style grasp sequence started")
        self.get_logger().info("SAM3-style grasp sequence started")
        return True

    def _make_sam3_style_grasp_plan(self, object_pose):
        target = np.array([
            object_pose.pose.position.x,
            object_pose.pose.position.y,
            object_pose.pose.position.z,
        ], dtype=float)
        if not np.isfinite(target).all():
            self._abort("sam3_style target contains NaN/Inf")
            return None

        target_radius = float(np.hypot(target[0], target[1]))
        if target_radius > self.sam3_max_target_radius:
            self._abort(
                f"sam3_style target radius {target_radius:.3f} m exceeds "
                f"limit {self.sam3_max_target_radius:.3f} m"
            )
            return None

        if target_radius > 1e-3:
            radial = np.array([target[0] / target_radius, target[1] / target_radius])
        else:
            radial = np.array([1.0, 0.0])
        right = np.array([radial[1], -radial[0]])

        effective_retract = max(0.0, self.sam3_tool_length - self.sam3_penetration)
        dx_plane = effective_retract * math.cos(self.sam3_approach_angle)
        dx_plane += self.sam3_pull_back
        dz = effective_retract * math.sin(self.sam3_approach_angle)

        lateral = self.sam3_lateral_offset * right
        flange_xy = target[:2] - dx_plane * radial + lateral
        flange_z = target[2] + dz

        lowest_z = (
            target[2]
            - self.sam3_penetration * math.sin(self.sam3_approach_angle)
            - self.sam3_finger_half_width * math.cos(self.sam3_approach_angle)
        )
        if lowest_z < self.sam3_min_fingertip_z:
            lift_amount = self.sam3_min_fingertip_z - lowest_z
            flange_z += lift_amount
            self.get_logger().warn(
                "SAM3-style fingertip guard lifted flange by "
                f"{lift_amount * 1000.0:.1f} mm"
            )
        if flange_z < self.sam3_min_flange_z:
            self.get_logger().warn(
                f"SAM3-style flange z {flange_z:.3f} below min "
                f"{self.sam3_min_flange_z:.3f}; clamping"
            )
            flange_z = self.sam3_min_flange_z

        pre_dx = self.sam3_pregrasp_distance * math.cos(self.sam3_approach_angle)
        pre_dz = self.sam3_pregrasp_distance * math.sin(self.sam3_approach_angle)
        pre_xy = flange_xy - pre_dx * radial
        pre_z = flange_z + pre_dz

        yaw = float(math.atan2(radial[1], radial[0])) + self.sam3_yaw_offset
        orientation = Rotation.from_euler(
            "xyz", [0.0, math.pi / 2.0 + self.sam3_approach_angle, yaw]
        )

        grasp = self._pose_from_xyz_rotation(
            [flange_xy[0], flange_xy[1], flange_z], orientation
        )
        pregrasp = self._pose_from_xyz_rotation(
            [pre_xy[0], pre_xy[1], pre_z], orientation
        )
        retreat = copy.deepcopy(pregrasp)
        retreat.pose.position.z += self.sam3_retreat_extra_z

        self.get_logger().info(
            "SAM3-style target geometry: "
            f"object=({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}) m, "
            f"radius={target_radius:.3f} m, "
            f"flange_grasp=({flange_xy[0]:.3f}, {flange_xy[1]:.3f}, {flange_z:.3f}) m, "
            f"pregrasp=({pre_xy[0]:.3f}, {pre_xy[1]:.3f}, {pre_z:.3f}) m, "
            f"yaw={math.degrees(yaw):.1f} deg, "
            f"approach={math.degrees(self.sam3_approach_angle):.1f} deg"
        )
        return {
            "pregrasp": pregrasp,
            "grasp": grasp,
            "retreat": retreat,
        }

    def _pose_from_xyz_rotation(self, xyz, rotation):
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(xyz[0])
        pose.pose.position.y = float(xyz[1])
        pose.pose.position.z = float(xyz[2])
        qx, qy, qz, qw = rotation.as_quat()
        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)
        return pose

    def _make_move_step(self, name, pose, allow_retry):
        return {
            "name": str(name),
            "pose": copy.deepcopy(pose),
            "allow_retry": bool(allow_retry),
        }

    def _apply_place_orientation(self, place):
        x = float(place.pose.position.x)
        y = float(place.pose.position.y)
        radius = float(math.hypot(x, y))
        yaw = math.atan2(y, x) if radius > 1e-3 else 0.0
        yaw += self.place_yaw_offset
        orientation = Rotation.from_euler(
            "xyz", [0.0, math.pi / 2.0 + self.place_approach_angle, yaw]
        )
        qx, qy, qz, qw = orientation.as_quat()
        place.pose.orientation.x = float(qx)
        place.pose.orientation.y = float(qy)
        place.pose.orientation.z = float(qz)
        place.pose.orientation.w = float(qw)
        self.get_logger().info(
            "Place orientation generated from place_position: "
            f"radial_down yaw={math.degrees(yaw):.1f} deg, "
            f"approach={math.degrees(self.place_approach_angle):.1f} deg"
        )

    def _parse_move_step(self, value):
        if isinstance(value, dict):
            return {
                "name": value.get("name", "move"),
                "pose": value["pose"],
                "allow_retry": bool(value.get("allow_retry", True)),
            }
        return {
            "name": "move",
            "pose": value,
            "allow_retry": True,
        }

    def _make_home_joint_command(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.home_joint_names)
        count = len(msg.name)
        positions = list(self.active_joint_positions)
        velocities = list(self.home_joint_velocity)
        efforts = list(self.home_joint_effort)
        if len(positions) < count:
            positions.extend([self.open_width] * (count - len(positions)))
        if len(velocities) < count:
            velocities.extend([0.0] * (count - len(velocities)))
        if len(efforts) < count:
            efforts.extend([1.5] * (count - len(efforts)))
        msg.position = positions[:count]
        msg.velocity = velocities[:count]
        msg.effort = efforts[:count]
        return msg

    def _start_home_motion(self, reason):
        self._start_joint_motion(
            reason,
            self.home_joint_positions,
            self.home_motion_time,
            self.home_wait_for_feedback,
        )

    def _start_joint_motion(self, reason, positions, motion_time, require_feedback):
        self.home_motion_active = True
        self.home_motion_reason = reason
        self.active_joint_positions = list(positions)
        self.active_motion_time = float(motion_time)
        self.active_motion_require_feedback = bool(require_feedback)
        now = self.get_clock().now()
        self.home_motion_deadline = now + Duration(seconds=self.active_motion_time)
        self.home_next_command = now
        self.waiting_for_ik = False
        self.current_move = None
        self.get_logger().info(
            f"Moving Piper to configured joint pose ({reason})"
        )

    def _tick_home_motion(self, now):
        if self.home_next_command is None or now >= self.home_next_command:
            self.joint_cmd_pub.publish(self._make_home_joint_command())
            self.home_next_command = now + Duration(
                seconds=self.home_command_period
            )

        at_home = self._at_home()
        if at_home:
            self._finish_home_motion(reached=True)
            return

        if now >= self.home_motion_deadline:
            if self.active_motion_require_feedback:
                reason = self.home_motion_reason
                self._abort(f"{reason} joint pose was not reached")
                if reason == "shutdown":
                    self.shutdown_ready_pub.publish(Bool(data=False))
                    self._publish_status(
                        "Zero pose timeout; arm nodes remain active"
                    )
                return
            self.get_logger().warn(
                "Home motion timed out before feedback matched; continuing "
                "because home_wait_for_feedback=false"
            )
            self._finish_home_motion(reached=False)

    def _finish_home_motion(self, reached):
        reason = self.home_motion_reason
        self.home_motion_active = False
        self.home_motion_reason = ""
        self.home_motion_deadline = None
        self.home_next_command = None
        if reason in ("startup", "manual"):
            self.startup_ready = True
        state = "reached" if reached else "commanded"
        if reason == "shutdown":
            self.shutdown_ready_pub.publish(Bool(data=bool(reached)))
            self._publish_status("Zero pose reached; stopping arm nodes")
            self.get_logger().info("Zero joint pose reached; shutdown acknowledged")
        else:
            self._publish_status(f"Ready pose {state}")
            self.get_logger().info(f"Home joint pose {state}; reason={reason}")

    def _at_home(self):
        if not self.latest_joint_positions:
            return False
        compared = 0
        for name, target in zip(self.home_joint_names, self.active_joint_positions):
            if name == "gripper" or name not in self.latest_joint_positions:
                continue
            compared += 1
            if abs(self.latest_joint_positions[name] - float(target)) > self.home_joint_tolerance:
                return False
        return compared > 0

    def _publish_move_candidate(self):
        pose = copy.deepcopy(self.current_move)
        if self.current_move_allow_retry:
            pitch_adjustment, roll_adjustment = self.orientation_retry_offsets[
                self.orientation_attempt
            ]
        else:
            pitch_adjustment, roll_adjustment = (0.0, 0.0)
        adjusted = self._adjust_pose_roll_pitch(
            pose, roll_adjustment, pitch_adjustment
        )
        qx, qy, qz, qw = adjusted.as_quat()
        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)
        pose.header.stamp = self.get_clock().now().to_msg()
        self.target_pub.publish(pose)
        self.wait_deadline = self.get_clock().now() + Duration(
            seconds=self.ik_timeout
        )
        rpy = adjusted.as_euler("xyz", degrees=True)
        if self.orientation_attempt == 0:
            self.get_logger().info(
                f"IK attempt 0 for {self.current_move_name}: "
                f"retry={self.current_move_allow_retry}, "
                f"position=({pose.pose.position.x:.3f}, "
                f"{pose.pose.position.y:.3f}, {pose.pose.position.z:.3f}) m, "
                f"RPY=({rpy[0]:.1f}, {rpy[1]:.1f}, {rpy[2]:.1f}) deg"
            )
        else:
            self.get_logger().info(
                f"IK attempt {self.orientation_attempt}/"
                f"{len(self.orientation_retry_offsets) - 1}: "
                f"move={self.current_move_name}, "
                f"roll_adjust={math.degrees(roll_adjustment):.1f} deg, "
                f"pitch_adjust={math.degrees(pitch_adjustment):.1f} deg, "
                f"total_RPY=({rpy[0]:.1f}, {rpy[1]:.1f}, {rpy[2]:.1f}) deg"
            )

    def _generate_orientation_retry_offsets(self):
        offsets = [(0.0, 0.0)]  # (pitch_adjustment, roll_adjustment)
        if self.orientation_retry_spiral_step <= 0.0:
            return offsets

        max_adjustment = max(
            self.max_pitch_adjustment,
            self.max_roll_adjustment,
        )
        max_layers = int(max_adjustment / self.orientation_retry_spiral_step)
        total_adjustment = self.max_pitch_adjustment + self.max_roll_adjustment
        if total_adjustment <= 0.0:
            return offsets

        for layer in range(1, max_layers + 1):
            current_radius = layer * self.orientation_retry_spiral_step
            points_in_layer = max(8, int(8 * layer))
            for index in range(points_in_layer):
                angle = 2.0 * math.pi * index / points_in_layer
                pitch = (
                    current_radius
                    * math.sin(angle)
                    * (self.max_pitch_adjustment / total_adjustment)
                )
                roll = (
                    current_radius
                    * math.cos(angle)
                    * (self.max_roll_adjustment / total_adjustment)
                )
                pitch = max(
                    min(pitch, self.max_pitch_adjustment),
                    -self.max_pitch_adjustment,
                )
                roll = max(
                    min(roll, self.max_roll_adjustment),
                    -self.max_roll_adjustment,
                )
                if self._valid_orientation_adjustment(pitch, roll):
                    offsets.append((pitch, roll))
                    if (
                        self.orientation_retry_max_candidates > 0
                        and len(offsets) >= self.orientation_retry_max_candidates
                    ):
                        return offsets
        return offsets

    def _valid_orientation_adjustment(self, pitch, roll):
        if abs(pitch) > math.pi / 4.0 and abs(roll) > math.pi / 6.0:
            return False
        if abs(pitch) < 0.01 and abs(roll) < 0.01:
            return False
        return True

    def _adjust_pose_roll_pitch(self, pose, roll_adjustment, pitch_adjustment):
        q = pose.pose.orientation
        original = Rotation.from_quat([q.x, q.y, q.z, q.w])
        roll, pitch, yaw = original.as_euler("xyz", degrees=False)
        return Rotation.from_euler(
            "xyz",
            [
                roll + roll_adjustment,
                pitch + pitch_adjustment,
                yaw,
            ],
        )

    def _transform_pose(self, pose):
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

    def _inside_workspace(self, pose):
        point = np.array([
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        ])
        valid = np.isfinite(point).all() and np.all(
            point >= self.workspace_min
        ) and np.all(point <= self.workspace_max)
        if not valid:
            self.get_logger().warn(f"Rejected out-of-workspace pose: {point}")
        return bool(valid)

    def _pose_is_fresh(self, now):
        return (
            self.latest_pose is not None
            and self.latest_pose_time is not None
            and (now - self.latest_pose_time).nanoseconds
            <= int(self.pose_timeout * 1e9)
        )

    def _abort(self, reason):
        self.steps.clear()
        self.pending_request = False
        self.waiting_for_ik = False
        self.home_motion_active = False
        self.current_move = None
        self.current_move_name = "move"
        self.current_move_allow_retry = True
        self.hold_until = None
        if (
            self.gripper_holding_object
            and self.keep_gripper_closed_on_abort_after_grasp
        ):
            self.get_logger().error(
                "Task aborted after gripper close; keeping gripper closed "
                "to avoid dropping the object. Send /gripper_cmd manually if "
                "you want to release it."
            )
        else:
            self.gripper_pub.publish(Float64(data=self.open_width))
            self.gripper_holding_object = False
        self._publish_result(-1)
        self._publish_status(f"Task aborted: {reason}")
        self.get_logger().error(f"Task aborted: {reason}")

    def _publish_result(self, value):
        self.result_pub.publish(Int32(data=int(value)))

    def _publish_status(self, text):
        self.status_pub.publish(String(data=str(text)))


def main(args=None):
    rclpy.init(args=args)
    node = GraspTaskNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
