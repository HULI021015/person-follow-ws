"""Pure control and target-selection logic, kept independent for unit tests."""

from dataclasses import dataclass
from typing import Iterable, Optional, Sequence


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass
class ControlConfig:
    target_width_ratio: float = 0.385
    linear_gain: float = 0.65
    angular_gain: float = 1.10
    width_deadzone_ratio: float = 0.10
    center_deadzone_ratio: float = 0.05
    max_forward_speed: float = 0.35
    max_reverse_speed: float = 0.12
    max_angular_speed: float = 0.80
    allow_reverse: bool = False


def compute_velocity(
    box: Optional[Sequence[float]], image_width: int, config: ControlConfig
) -> tuple[float, float]:
    """Return ROS-convention (linear.x, angular.z) from an xyxy person box."""
    if box is None or image_width <= 0:
        return 0.0, 0.0
    x1, _y1, x2, _y2 = [float(value) for value in box]
    box_width_ratio = max(0.0, x2 - x1) / float(image_width)
    horizontal_error = ((x1 + x2) * 0.5 - image_width * 0.5) / (image_width * 0.5)

    # Image-right means the robot must turn right; ROS positive angular.z turns left.
    angular = 0.0
    if abs(horizontal_error) >= config.center_deadzone_ratio:
        angular = -config.angular_gain * horizontal_error
    angular = clamp(angular, -config.max_angular_speed, config.max_angular_speed)

    distance_error = (config.target_width_ratio - box_width_ratio) / config.target_width_ratio
    linear = 0.0
    if abs(distance_error) >= config.width_deadzone_ratio:
        linear = config.linear_gain * distance_error
    if not config.allow_reverse:
        linear = max(0.0, linear)
    linear = clamp(linear, -config.max_reverse_speed, config.max_forward_speed)

    # Reduce forward speed while making a large turn to avoid cutting the corner.
    turn_scale = 1.0 - 0.55 * min(1.0, abs(horizontal_error))
    return linear * turn_scale, angular


class TargetLock:
    """Keep one BoT-SORT ID, then reacquire the largest person after N lost frames."""

    def __init__(self, lost_limit: int = 15) -> None:
        self.target_id: Optional[int] = None
        self.lost_frames = 0
        self.lost_limit = max(0, int(lost_limit))

    def update(
        self, ids: Iterable[int], boxes: Iterable[Sequence[float]]
    ) -> tuple[Optional[int], Optional[Sequence[float]]]:
        ids_list = [int(item) for item in ids]
        boxes_list = list(boxes)
        if not ids_list:
            if self.target_id is not None:
                self.lost_frames += 1
                if self.lost_frames > self.lost_limit:
                    self.target_id = None
            return self.target_id, None

        if self.target_id in ids_list:
            self.lost_frames = 0
        elif self.target_id is not None and self.lost_frames < self.lost_limit:
            self.lost_frames += 1
            return self.target_id, None
        else:
            areas = [max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1]) for b in boxes_list]
            selected = max(range(len(areas)), key=areas.__getitem__)
            self.target_id = ids_list[selected]
            self.lost_frames = 0

        selected = ids_list.index(self.target_id)
        return self.target_id, boxes_list[selected]

