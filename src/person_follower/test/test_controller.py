from person_follower.controller import ControlConfig, TargetLock, compute_velocity


def test_no_target_stops():
    assert compute_velocity(None, 1920, ControlConfig()) == (0.0, 0.0)


def test_far_center_target_moves_forward():
    linear, angular = compute_velocity((800, 100, 1120, 900), 1920, ControlConfig())
    assert linear > 0.0
    assert angular == 0.0


def test_image_right_turns_robot_right_in_ros_convention():
    _linear, angular = compute_velocity((1400, 100, 1700, 900), 1920, ControlConfig())
    assert angular < 0.0


def test_target_lock_chooses_largest_box():
    lock = TargetLock(lost_limit=2)
    target_id, box = lock.update([3, 7], [(0, 0, 10, 10), (0, 0, 20, 20)])
    assert target_id == 7
    assert box == (0, 0, 20, 20)


def test_target_lock_does_not_switch_during_short_loss():
    lock = TargetLock(lost_limit=2)
    lock.update([7], [(0, 0, 20, 20)])
    target_id, box = lock.update([3], [(0, 0, 30, 30)])
    assert target_id == 7
    assert box is None

