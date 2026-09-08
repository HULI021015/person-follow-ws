import rclpy
from std_msgs.msg import Bool

from graspgen_tasks.grasp_task_node import GraspTaskNode


def test_shutdown_moves_to_zero_and_requires_feedback():
    rclpy.init()
    node = GraspTaskNode()
    try:
        node._shutdown_callback(Bool(data=True))
        assert node.home_motion_active
        assert node.home_motion_reason == "shutdown"
        assert node.active_motion_require_feedback is True
        assert node.active_joint_positions[:6] == [0.0] * 6

        node.latest_joint_positions = {
            f"joint{index}": 0.0 for index in range(1, 7)
        }
        node._tick_home_motion(node.get_clock().now())
        assert node.home_motion_active is False
        assert node.home_motion_reason == ""
    finally:
        node.destroy_node()
        rclpy.shutdown()
