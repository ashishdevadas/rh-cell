"""Remote-handling cell in one command: Gazebo (UR5e + F/T sensor + cassette, cell world),
F/T bridge to ROS, MoveIt 2 and RViz.

    ros2 launch rh_cell_bringup sim.launch.py
    ros2 launch rh_cell_bringup sim.launch.py gazebo_gui:=false launch_rviz:=false   # headless
    ros2 launch rh_cell_bringup sim.launch.py ic:=true     # + Phase 2 I&C layer (EPICS IOC + PLC bridge)
    ros2 launch rh_cell_bringup sim.launch.py world:=rh_cell_tight.sdf   # 2 mm clearance + chamfer (RH Cell 2.0)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    launch_rviz = LaunchConfiguration("launch_rviz")
    desc = FindPackageShare("rh_cell_description")

    # 1. Gazebo + ros2_control with OUR robot (sensor + cassette) and OUR world
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur_simulation_gz"), "launch", "ur_sim_control.launch.py"])
        ),
        launch_arguments={
            "ur_type": ur_type,
            "description_file": PathJoinSubstitution([desc, "urdf", "rh_cell_ur.urdf.xacro"]),
            "world_file": PathJoinSubstitution([desc, "worlds", LaunchConfiguration("world")]),
            "gazebo_gui": LaunchConfiguration("gazebo_gui"),
            "launch_rviz": "false",
        }.items(),
    )

    # 2. F/T sensor: Gazebo topic -> ROS topic /rh/ft
    ft_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": PathJoinSubstitution([desc, "config", "gz_bridge.yaml"])}],
        output="screen",
    )

    # 3. MoveIt (move_group). RViz is started below with our saved config instead of the stock one.
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"])
        ),
        launch_arguments={"ur_type": ur_type, "use_sim_time": "true", "launch_rviz": "false"}.items(),
    )

    # 4. RViz with rh_cell.rviz (goal-state handle and planned-path ghost switched off)
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description_semantic("srdf/ur.srdf.xacro", {"name": "ur5e"})
        .to_moveit_configs()
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_rh_cell",
        output="log",
        arguments=["-d", PathJoinSubstitution([FindPackageShare("rh_cell_bringup"), "config", "rh_cell.rviz"])],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
        condition=IfCondition(launch_rviz),
    )

    # 5. Streaming position controller for force control (Step 3c). Loaded inactive; the force
    #    insertion node switches to it. Delayed so the controller_manager is up.
    fwd_pos = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "-c", "/controller_manager", "--inactive"],
        output="screen",
    )

    # 6. Phase 2 I&C layer: EPICS IOC (operator PVs) + PLC bridge (motion permit)
    ic_layer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("rh_cell_bringup"), "launch", "ic.launch.py"])
        ),
        launch_arguments={"plc_mode": LaunchConfiguration("plc_mode")}.items(),
        condition=IfCondition(LaunchConfiguration("ic")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur5e"),
        DeclareLaunchArgument("world", default_value="rh_cell.sdf",
                              description="World file in rh_cell_description/worlds (rh_cell.sdf or rh_cell_tight.sdf)"),
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("ic", default_value="false", description="Start the Phase 2 I&C layer"),
        DeclareLaunchArgument("plc_mode", default_value="sim", choices=["sim", "openplc"]),
        sim,
        ft_bridge,
        moveit,
        rviz,
        TimerAction(period=8.0, actions=[fwd_pos]),
        ic_layer,
    ])
