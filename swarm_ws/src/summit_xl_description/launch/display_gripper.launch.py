"""Visualiza el Summit XLS con la pinza en RViz, SIN Gazebo.

Solo robot_state_publisher + joint_state_publisher_gui (sliders) + RViz, para
ver la geometria y mover los dedos a mano con los sliders. Es la verificacion
de la fase 1 (geometria); el control gz_ros2_control vendra despues.

  ros2 launch summit_xl_description display_gripper.launch.py

Requiere: ros-jazzy-joint-state-publisher-gui
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _rviz_config() -> str:
    config = {
        "Visualization Manager": {
            "Global Options": {"Fixed Frame": "base_footprint", "Frame Rate": 30},
            "Displays": [
                {"Class": "rviz_default_plugins/Grid", "Name": "Grid",
                 "Enabled": True, "Cell Size": 0.5, "Plane Cell Count": 20},
                {"Class": "rviz_default_plugins/TF", "Name": "TF",
                 "Enabled": True, "Show Names": True, "Marker Scale": 0.4},
                {"Class": "rviz_default_plugins/RobotModel", "Name": "RobotModel",
                 "Enabled": True,
                 "Description Topic": {"Value": "/robot_description"},
                 "Visual Enabled": True, "Collision Enabled": False},
            ],
            "Tools": [{"Class": "rviz_default_plugins/MoveCamera"}],
            "Views": {"Current": {
                "Class": "rviz_default_plugins/Orbit",
                "Distance": 2.5, "Pitch": 0.5,
                "Focal Point": {"X": 0.3, "Y": 0.0, "Z": 0.2},
            }},
        },
    }
    path = os.path.join(tempfile.gettempdir(), "summit_gripper.rviz")
    with open(path, "w") as f:
        yaml.safe_dump(config, f, default_flow_style=False)
    return path


def generate_launch_description():
    xacro_file = os.path.join(
        get_package_share_directory("summit_xl_description"),
        "robots", "summit_xl_omni.urdf.xacro")
    robot_desc = ParameterValue(
        Command(["xacro ", xacro_file, " gripper:=true robot_ns:=summit0"]),
        value_type=str)

    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             output="screen", parameters=[{"robot_description": robot_desc}]),
        Node(package="joint_state_publisher_gui",
             executable="joint_state_publisher_gui", output="screen"),
        Node(package="rviz2", executable="rviz2", output="screen",
             arguments=["-d", _rviz_config()]),
    ])
