"""Teleop con mando DualSense (PS5): joy_node + ps5_teleop.

Conecta el mando (USB o Bluetooth) -> aparece como /dev/input/jsN. Luego:

  ros2 launch swarm_behavior teleop_ps5.launch.py robot:=summit0

Stick izq = avanzar/strafe (mecanum), stick der = girar, R1 = agarrar,
L1 = soltar, R2 = turbo. Si algún eje va al revés, ajusta los params
invert_*/axis_* del nodo ps5_teleop (ver `ros2 topic echo /joy`).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot = LaunchConfiguration("robot")
    dev = LaunchConfiguration("device_id")
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="summit0"),
        DeclareLaunchArgument("device_id", default_value="0",
                              description="Índice del joystick (/dev/input/jsN)"),
        DeclareLaunchArgument("frame", default_value="world",
                              description="'world' (orientado al campo) o 'body' (morro del robot)"),
        DeclareLaunchArgument("view_yaw_deg", default_value="0.0",
                              description="Offset para alinear 'stick arriba' con tu cámara (0/90/180/-90)"),
        DeclareLaunchArgument("invert_strafe", default_value="-1.0",
                              description="Signo del strafe; pon 1.0 si izquierda/derecha sale espejado"),
        Node(
            package="joy", executable="joy_node", name="joy_node", output="screen",
            parameters=[{
                "device_id": dev,
                "deadzone": 0.05,
                # republica /joy a 20 Hz aunque el stick no se mueva -> cmd_vel
                # continuo (sin esto el robot solo recibe Twist al mover el stick).
                "autorepeat_rate": 20.0,
            }],
        ),
        Node(
            package="swarm_behavior", executable="ps5_teleop", name="ps5_teleop",
            output="screen",
            parameters=[{
                "robot": robot,
                "frame": LaunchConfiguration("frame"),
                "view_yaw_deg": ParameterValue(
                    LaunchConfiguration("view_yaw_deg"), value_type=float),
                "invert_strafe": ParameterValue(
                    LaunchConfiguration("invert_strafe"), value_type=float),
            }],
        ),
    ])
