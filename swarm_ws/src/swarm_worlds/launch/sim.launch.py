"""Spawn N minibots into Gazebo Fortress with ROS↔Gz bridges.

Usage:
    ros2 launch swarm_worlds sim.launch.py n_robots:=3 world:=empty_arena
"""
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _spawn_for_robot(context, ns_str: str, x: float, y: float):
    desc_share = get_package_share_directory("swarm_description")
    xacro_path = os.path.join(desc_share, "urdf", "minibot.urdf.xacro")

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
        # Spawn into Gazebo using the same xacro output
        Node(
            package="ros_ign_gazebo",
            executable="create",
            arguments=[
                "-name", ns_str,
                "-topic", f"/{ns_str}/robot_description",
                "-x", str(x), "-y", str(y), "-z", "0.05",
            ],
            output="screen",
        ),
        # Per-robot bridges
        Node(
            package="ros_ign_bridge",
            executable="parameter_bridge",
            namespace=ns_str,
            name=f"bridge_{ns_str}",
            output="screen",
            arguments=[
                f"/model/{ns_str}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
                f"/model/{ns_str}/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                f"/model/{ns_str}/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
                f"/model/{ns_str}/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model",
                f"/model/{ns_str}/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
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
    n_robots = int(LaunchConfiguration("n_robots").perform(context))
    actions = []

    # Place robots in a circle of radius 1m
    radius = 1.0
    for i in range(n_robots):
        ang = 2.0 * math.pi * i / max(n_robots, 1)
        x = radius * math.cos(ang)
        y = radius * math.sin(ang)
        actions += _spawn_for_robot(context, f"robot{i}", x, y)

    return actions


def generate_launch_description():
    worlds_share = get_package_share_directory("swarm_worlds")
    world_path = os.path.join(worlds_share, "worlds", "empty_arena.sdf")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_ign_gazebo"), "launch", "ign_gazebo.launch.py")
        ),
        launch_arguments={"ign_args": f"-r -v 4 {world_path}"}.items(),
    )

    clock_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
    )

    return LaunchDescription([
        DeclareLaunchArgument("n_robots", default_value="2", description="Number of minibots to spawn"),
        gz_sim,
        clock_bridge,
        OpaqueFunction(function=_launch_setup),
    ])
