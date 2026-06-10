"""Spawn one X3 quadrotor into an ALREADY RUNNING Gazebo Fortress sim.

No lanza Gazebo: se engancha al servidor que ya levantó sim_summit.launch.py,
por lo que no interfiere con los Summit XL. El dron spawnea en el suelo,
despega solo y queda en hover (~2.3 m). Reutiliza la generación de SDF y la
secuencia de despegue de sim_summit.launch.py.

Usage (con la sim ya corriendo):
    ros2 launch swarm_worlds spawn_drone.launch.py
    ros2 launch swarm_worlds spawn_drone.launch.py x:=1.0 y:=2.0 name:=drone_extra
"""
import importlib.util
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

_here = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location(
    "sim_summit_launch", os.path.join(_here, "sim_summit.launch.py")
)
_sim_summit = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_sim_summit)


def _launch_setup(context, *args, **kwargs):
    name = LaunchConfiguration("name").perform(context)
    x = float(LaunchConfiguration("x").perform(context))
    y = float(LaunchConfiguration("y").perform(context))
    return _sim_summit._spawn_for_drone(name, x, y, 0.0)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("name", default_value="drone_1", description="Model name / namespace"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        OpaqueFunction(function=_launch_setup),
    ])
