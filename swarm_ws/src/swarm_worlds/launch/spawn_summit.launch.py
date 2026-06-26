"""Spawn one Summit XL (no arm) into Gazebo Harmonic with ROS↔Gz bridges.

Usage:
    ros2 launch swarm_worlds spawn_summit.launch.py
    ros2 launch swarm_worlds spawn_summit.launch.py headless:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _spawn_summit(context, ns_str: str, x: float, y: float):
    desc_share = get_package_share_directory("summit_xl_description")
    xacro_path = os.path.join(desc_share, "robots", "summit_xl_noarm.urdf.xacro")

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            xacro_path, " ",
            f"robot_ns:={ns_str}",
        ]),
        value_type=str,
    )

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=ns_str,
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "frame_prefix": f"{ns_str}/",
                "use_sim_time": True,
            }],
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name", ns_str,
                "-topic", f"/{ns_str}/robot_description",
                "-x", str(x), "-y", str(y), "-z", "0.2",
            ],
            output="screen",
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            namespace=ns_str,
            name=f"bridge_{ns_str}",
            output="screen",
            arguments=[
                f"/model/{ns_str}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                f"/model/{ns_str}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                f"/model/{ns_str}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                f"/model/{ns_str}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
                f"/model/{ns_str}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            ],
            remappings=[
                (f"/model/{ns_str}/cmd_vel",      "cmd_vel"),
                (f"/model/{ns_str}/odometry",     "odom"),
                (f"/model/{ns_str}/scan",         "scan"),
                (f"/model/{ns_str}/joint_states", "joint_states"),
                (f"/model/{ns_str}/tf",           "tf"),
            ],
        ),
    ]
    return nodes


def _launch_setup(context, *args, **kwargs):
    worlds_share = get_package_share_directory("swarm_worlds")
    world_path = os.path.join(worlds_share, "worlds", "empty_arena.sdf")

    headless = LaunchConfiguration("headless").perform(context).lower() in ("true", "1")
    gz_args = f"-r -v 4 {world_path}" + (" -s" if headless else "")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    return [gz_sim] + _spawn_summit(context, "summit", 0.0, 0.0)


def generate_launch_description():
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false", description="Run Gazebo server only (no GUI)"),
        clock_bridge,
        OpaqueFunction(function=_launch_setup),
    ])
