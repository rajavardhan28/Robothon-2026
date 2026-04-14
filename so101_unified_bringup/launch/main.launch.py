import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _merge_resource_paths(*groups):
    merged = []
    for group in groups:
        if not group:
            continue
        entries = group.split(os.pathsep) if isinstance(group, str) else group
        for entry in entries:
            if entry and entry not in merged:
                merged.append(entry)
    return os.pathsep.join(merged)


def generate_launch_description():

    package_share = get_package_share_directory("so101_unified_bringup")
    so101_description_share = get_package_share_directory("so101_description")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    world_default = os.path.join(package_share, "worlds", "empty_world.sdf")

    moveit_config = MoveItConfigsBuilder(
        "so101_new_calib", package_name="so101_moveit_config"
    ).to_moveit_configs()

    gz_resource_paths = _merge_resource_paths(
        os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
        [os.path.dirname(so101_description_share), package_share],
    )

    ign_resource_paths = _merge_resource_paths(
        os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""),
        [os.path.dirname(so101_description_share), package_share],
    )

    # ================= GAZEBO =================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-r -v 4 ", LaunchConfiguration("world")],
        }.items(),
    )

    # ================= ROBOT =================
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
        output="screen",
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world", "empty",
            "-string", moveit_config.robot_description["robot_description"],
            "-name", "so101",
            "-x", "-0.55",
            "-y", "0.0",
            "-z", "0.7774",
        ],
        output="screen",
    )

    # ================= MOVEIT =================
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(package_share, "config", "moveit.rviz")],
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
        output="screen",
    )

    # ================= ROS2 CONTROL =================
    controllers_file = os.path.join(
        so101_description_share, "config", "ros2_controllers.yaml"
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            controllers_file,
            {"use_sim_time": True},
        ],
        output="screen",
    )

    # ✅ FIXED NAMES (VERY IMPORTANT)
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    trajectory_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
        output="screen",
    )

    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=world_default),

        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_resource_paths),
        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", ign_resource_paths),

        gazebo,
        robot_state_publisher,

        TimerAction(period=3.0, actions=[spawn_robot]),

        # 🔥 START CONTROL FIRST
        TimerAction(period=6.0, actions=[ros2_control_node]),

        # 🔥 THEN CONTROLLERS
        TimerAction(period=9.0, actions=[joint_state_spawner]),
        TimerAction(period=10.0, actions=[trajectory_spawner]),
        TimerAction(period=11.0, actions=[gripper_spawner]),

        # MOVEIT
        TimerAction(period=12.0, actions=[move_group]),

        # RVIZ
        TimerAction(period=14.0, actions=[rviz]),
    ])
