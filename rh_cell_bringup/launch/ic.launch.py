"""Phase 2 I&C layer: EPICS IOC (operator PVs) + PLC bridge (motion permit).

    ros2 launch rh_cell_bringup ic.launch.py                       # soft PLC
    ros2 launch rh_cell_bringup ic.launch.py plc_mode:=openplc     # OpenPLC runtime on localhost:502
Then open rh_cell_ic/phoebus/rh_cell.bob in Phoebus, or use caproto-get / caproto-put.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("plc_mode", default_value="sim", choices=["sim", "openplc"]),
        DeclareLaunchArgument("modbus_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("modbus_port", default_value="502"),
        DeclareLaunchArgument("pv_prefix", default_value="RH:"),
        Node(
            package="rh_cell_ic",
            executable="plc_bridge",
            parameters=[{
                "plc_mode": LaunchConfiguration("plc_mode"),
                "modbus_host": LaunchConfiguration("modbus_host"),
                "modbus_port": LaunchConfiguration("modbus_port"),
            }],
            output="screen",
        ),
        Node(
            package="rh_cell_ic",
            executable="epics_ioc",
            arguments=["--prefix", LaunchConfiguration("pv_prefix")],
            output="screen",
        ),
    ])
