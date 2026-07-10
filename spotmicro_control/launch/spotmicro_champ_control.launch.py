import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    locomotion_share = get_package_share_directory("spotmicro_locomotion_ros2")
    description_share = get_package_share_directory("spotmicro_description")

    joints_config = os.path.join(locomotion_share, "config", "joints", "joints.yaml")
    links_config = os.path.join(locomotion_share, "config", "links", "links.yaml")
    gait_config = os.path.join(locomotion_share, "config", "gait", "gait.yaml")
    description_path = os.path.join(description_share, "urdf", "spotmicro.urdf.xacro")

    robot_urdf = Command(["xacro ", description_path])

    quadruped_controller = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"gazebo": True},
            {"publish_joint_states": False},
            {"publish_joint_control": True},
            {"publish_foot_contacts": False},
            {"joint_controller_topic": "/spotmicro/champ/joint_targets"},
            {"loop_rate": 120.0},
            {"urdf": robot_urdf},
            joints_config,
            links_config,
            gait_config,
        ],
        remappings=[
            ("cmd_vel/smooth", "/cmd_vel"),
            ("body_pose", "/body_pose"),
        ],
    )

    champ_bridge = Node(
        package="spotmicro_locomotion_ros2",
        executable="spotmicro_champ_bridge",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"joint_trajectory_topic": "/spotmicro/champ/joint_targets"},
            {"joint_states_topic": "/joint_states"},
            {"initial_joint_positions": [0.0, 0.75, -1.45] * 4},
            {"startup_stand_duration": 2.0},
            {"kp": [4.0, 8.0, 7.0] * 4},
            {"kd": [0.12, 0.18, 0.16] * 4},
            {"max_force": [2.2, 4.0, 4.0] * 4},
            {"publish_rate": 200.0},
            {"command_timeout": 0.35},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        quadruped_controller,
        champ_bridge,
    ])