from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64
    from trajectory_msgs.msg import JointTrajectory
except ModuleNotFoundError:
    rclpy = None
    Node = object
    JointState = None
    Float64 = None
    JointTrajectory = None


DEFAULT_JOINT_NAMES = [
    "front_left_shoulder",
    "front_left_leg",
    "front_left_foot",
    "front_right_shoulder",
    "front_right_leg",
    "front_right_foot",
    "rear_left_shoulder",
    "rear_left_leg",
    "rear_left_foot",
    "rear_right_shoulder",
    "rear_right_leg",
    "rear_right_foot",
]


DEFAULT_COMMAND_TOPICS = [f"/{joint_name}_cmd" for joint_name in DEFAULT_JOINT_NAMES]


DEFAULT_STAND_JOINT_POSITIONS = [
    0.0,
    0.75,
    -1.45,
    0.0,
    0.75,
    -1.45,
    0.0,
    0.75,
    -1.45,
    0.0,
    0.75,
    -1.45,
]


DEFAULT_KP = [4.0, 8.0, 7.0] * 4
DEFAULT_KD = [0.12, 0.18, 0.16] * 4
DEFAULT_MAX_FORCE = [2.2, 4.0, 4.0] * 4


def clamp_value(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def clamp_force(force: float, max_force: float) -> float:
    return clamp_value(force, -abs(max_force), abs(max_force))


def parameter_list(values: Iterable[float], expected_size: int, name: str) -> List[float]:
    result = [float(value) for value in values]
    if len(result) == 1:
        return result * expected_size
    if len(result) != expected_size:
        raise ValueError(f"{name} must contain 1 or {expected_size} values")
    return result


@dataclass
class JointFeedback:
    position: float = 0.0
    velocity: float = 0.0
    has_position: bool = False
    has_velocity: bool = False


class SpotMicroChampBridge(Node):
    def __init__(self) -> None:
        if rclpy is None:
            raise RuntimeError("rclpy is required to run spotmicro_champ_bridge")

        super().__init__("spotmicro_champ_bridge")

        self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)
        self.declare_parameter("command_topics", DEFAULT_COMMAND_TOPICS)
        self.declare_parameter("joint_trajectory_topic", "/spotmicro/champ/joint_targets")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("kp", DEFAULT_KP)
        self.declare_parameter("kd", DEFAULT_KD)
        self.declare_parameter("max_force", DEFAULT_MAX_FORCE)
        self.declare_parameter("joint_signs", [1.0])
        self.declare_parameter("joint_offsets", [0.0])
        self.declare_parameter("initial_joint_positions", DEFAULT_STAND_JOINT_POSITIONS)
        self.declare_parameter("startup_stand_duration", 2.0)
        self.declare_parameter("publish_rate", 200.0)
        self.declare_parameter("command_timeout", 0.35)

        self.joint_names = list(self.get_parameter("joint_names").value)
        command_topics = list(self.get_parameter("command_topics").value)

        if len(command_topics) != len(self.joint_names):
            raise ValueError("command_topics must match joint_names length")

        joint_count = len(self.joint_names)
        self.kp = parameter_list(self.get_parameter("kp").value, joint_count, "kp")
        self.kd = parameter_list(self.get_parameter("kd").value, joint_count, "kd")
        self.max_force = parameter_list(
            self.get_parameter("max_force").value, joint_count, "max_force"
        )
        self.joint_signs = parameter_list(
            self.get_parameter("joint_signs").value, joint_count, "joint_signs"
        )
        self.joint_offsets = parameter_list(
            self.get_parameter("joint_offsets").value, joint_count, "joint_offsets"
        )
        self.initial_joint_positions = parameter_list(
            self.get_parameter("initial_joint_positions").value,
            joint_count,
            "initial_joint_positions",
        )

        self.feedback: Dict[str, JointFeedback] = {
            joint_name: JointFeedback() for joint_name in self.joint_names
        }
        self.targets: Dict[str, float] = {joint_name: 0.0 for joint_name in self.joint_names}
        self.target_velocities: Dict[str, float] = {
            joint_name: 0.0 for joint_name in self.joint_names
        }
        for index, joint_name in enumerate(self.joint_names):
            self.targets[joint_name] = self.initial_joint_positions[index]
        self.start_time = self.get_clock().now()
        self.last_command_time: Optional[object] = self.start_time

        self.command_publishers = [
            self.create_publisher(Float64, command_topic, 10)
            for command_topic in command_topics
        ]

        self.create_subscription(
            JointTrajectory,
            str(self.get_parameter("joint_trajectory_topic").value),
            self.joint_trajectory_callback,
            10,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self.joint_state_callback,
            20,
        )

        publish_rate = float(self.get_parameter("publish_rate").value)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_forces)

    def joint_trajectory_callback(self, message: JointTrajectory) -> None:
        if not message.points:
            return
        if self.in_startup_stand():
            return

        point = message.points[0]
        for index, joint_name in enumerate(message.joint_names):
            if joint_name not in self.targets or index >= len(point.positions):
                continue
            target_index = self.joint_names.index(joint_name)
            self.targets[joint_name] = (
                self.joint_signs[target_index] * point.positions[index]
                + self.joint_offsets[target_index]
            )
            if index < len(point.velocities):
                self.target_velocities[joint_name] = point.velocities[index]

        self.last_command_time = self.get_clock().now()

    def joint_state_callback(self, message: JointState) -> None:
        for index, joint_name in enumerate(message.name):
            if joint_name not in self.feedback:
                continue

            feedback = self.feedback[joint_name]
            if index < len(message.position):
                feedback.position = message.position[index]
                feedback.has_position = True
            if index < len(message.velocity):
                feedback.velocity = message.velocity[index]
                feedback.has_velocity = True

    def publish_forces(self) -> None:
        if self.last_command_time is None:
            return

        elapsed = self.get_clock().now() - self.last_command_time
        if (
            not self.in_startup_stand()
            and elapsed.nanoseconds * 1e-9 > float(self.get_parameter("command_timeout").value)
        ):
            self.publish_zero_forces()
            return

        for index, joint_name in enumerate(self.joint_names):
            feedback = self.feedback[joint_name]
            if not feedback.has_position:
                continue

            position_error = self.targets[joint_name] - feedback.position
            velocity_error = self.target_velocities[joint_name]
            if feedback.has_velocity:
                velocity_error -= feedback.velocity

            force = self.kp[index] * position_error + self.kd[index] * velocity_error
            self.command_publishers[index].publish(
                Float64(data=clamp_force(force, self.max_force[index]))
            )

    def in_startup_stand(self) -> bool:
        elapsed = self.get_clock().now() - self.start_time
        return elapsed.nanoseconds * 1e-9 < float(
            self.get_parameter("startup_stand_duration").value
        )

    def publish_zero_forces(self) -> None:
        for publisher in self.command_publishers:
            publisher.publish(Float64(data=0.0))


def main(args: Optional[List[str]] = None) -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is required to run spotmicro_champ_bridge")

    rclpy.init(args=args)
    node = SpotMicroChampBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()