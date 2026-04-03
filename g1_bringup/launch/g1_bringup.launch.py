"""
Full G1 bringup: real hardware + MoveIt2.

Includes g1_hardware/hardware.launch.py (RSP + ros2_control_node),
spawns controllers, and adds move_group, RViz, and static virtual-joint TFs.

Usage:
  ros2 launch g1_bringup g1_bringup.launch.py network_interface:=eth0
  ros2 launch g1_bringup g1_bringup.launch.py network_interface:=eth0 use_rviz:=false
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_moveit_rviz_launch,
    generate_static_virtual_joint_tfs_launch,
)

_HARDWARE_CONFIG = {
    "no_hand": {
        "urdf_xacro":        "g1_no_hand_real.urdf.xacro",
        "srdf":               "g1_no_hand.srdf",
        "moveit_controllers": "moveit_controllers_no_hand.yaml",
        "controllers": [
            "joint_state_broadcaster",
            "left_arm_controller",
            "right_arm_controller",
            "waist_controller",
        ],
    },
}


def _launch_setup(context, *args, **kwargs):
    hand_type       = LaunchConfiguration("hand_type").perform(context)
    network_interface = LaunchConfiguration("network_interface").perform(context)
    kp              = LaunchConfiguration("kp").perform(context)
    kd              = LaunchConfiguration("kd").perform(context)
    weight_rate     = LaunchConfiguration("weight_rate").perform(context)
    use_rviz        = LaunchConfiguration("use_rviz").perform(context).lower()

    cfg = _HARDWARE_CONFIG[hand_type]

    g1_hw_share     = Path(get_package_share_directory("g1_hardware"))
    g1_moveit_share = Path(get_package_share_directory("g1_moveit_config"))

    # Build moveit_config for MoveIt nodes (RSP is handled by hardware launch)
    moveit_config = (
        MoveItConfigsBuilder("g1_29dof", package_name="g1_moveit_config")
        .robot_description(
            file_path=str(g1_hw_share / f"config/{cfg['urdf_xacro']}"),
            mappings={
                "network_interface": network_interface,
                "kp":          kp,
                "kd":          kd,
                "weight_rate": weight_rate,
            },
        )
        .robot_description_semantic(
            file_path=str(g1_moveit_share / f"config/{cfg['srdf']}"),
        )
        .trajectory_execution(
            file_path=str(g1_moveit_share / f"config/{cfg['moveit_controllers']}"),
        )
        .to_moveit_configs()
    )

    # Hardware driver (RSP + ros2_control_node + spawners)
    hw_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(g1_hw_share / "launch/hardware.launch.py")
        ),
        launch_arguments={
            "hand_type":        hand_type,
            "network_interface": network_interface,
            "kp":               kp,
            "kd":               kd,
            "weight_rate":      weight_rate,
        }.items(),
    )

    # Arm controllers start inactive so no hold commands are sent at startup.
    # Activate them explicitly before executing a MoveIt trajectory:
    #   ros2 control set_controller_state left_arm_controller active
    #   ros2 control set_controller_state right_arm_controller active
    _ARM_CONTROLLERS = {"left_arm_controller", "right_arm_controller", "waist_controller"}
    spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name] + (["--inactive"] if name in _ARM_CONTROLLERS else []),
        )
        for name in cfg["controllers"]
    ]

    actions = [
        hw_launch,
        *spawners,
        *generate_static_virtual_joint_tfs_launch(moveit_config).entities,
        *generate_move_group_launch(moveit_config).entities,
    ]

    if use_rviz not in ("false", "0"):
        actions.extend(generate_moveit_rviz_launch(moveit_config).entities)

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "hand_type",
            default_value="no_hand",
            choices=list(_HARDWARE_CONFIG),
            description="G1 hand configuration",
        ),
        DeclareLaunchArgument(
            "network_interface",
            description="Network interface connected to the G1 robot (e.g. 'eth0')",
        ),
        DeclareLaunchArgument("kp",          default_value="60.0",
                              description="Position gain"),
        DeclareLaunchArgument("kd",          default_value="1.5",
                              description="Velocity (damping) gain"),
        DeclareLaunchArgument("weight_rate", default_value="0.2",
                              description="Arm SDK control weight ramp rate (weight/s)"),
        DeclareLaunchArgument("use_rviz",    default_value="true",
                              description="Launch RViz with MoveIt plugin"),
        OpaqueFunction(function=_launch_setup),
    ])
