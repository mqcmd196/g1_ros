# Copyright 2026 Yoshiki Obinata
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Yoshiki Obinata nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

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
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
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
        "urdf_xacro": "g1_no_hand_real.urdf.xacro",
        "srdf": "g1_no_hand.srdf",
        "controllers": [
            "joint_state_broadcaster",
            "upper_body_controller",
        ],
    },
}


def _launch_setup(context, *args, **kwargs):
    hand_type = LaunchConfiguration("hand_type").perform(context)
    network_interface = LaunchConfiguration("network_interface").perform(context)
    kp = LaunchConfiguration("kp").perform(context)
    kd = LaunchConfiguration("kd").perform(context)
    waist_kp = LaunchConfiguration("waist_kp").perform(context)
    waist_kd = LaunchConfiguration("waist_kd").perform(context)
    use_rviz = LaunchConfiguration("use_rviz").perform(context).lower()

    cfg = _HARDWARE_CONFIG[hand_type]

    g1_hw_share = Path(get_package_share_directory("g1_hardware"))
    g1_moveit_share = Path(get_package_share_directory("g1_moveit_config"))

    # Build moveit_config for MoveIt nodes (RSP is handled by hardware launch)
    moveit_config = (
        MoveItConfigsBuilder("g1_29dof", package_name="g1_moveit_config")
        .robot_description(
            file_path=str(g1_hw_share / f"config/{cfg['urdf_xacro']}"),
            mappings={
                "network_interface": network_interface,
                "kp": kp,
                "kd": kd,
                "waist_kp": waist_kp,
                "waist_kd": waist_kd,
            },
        )
        .robot_description_semantic(
            file_path=str(g1_moveit_share / f"config/{cfg['srdf']}"),
        )
        .to_moveit_configs()
    )

    # Hardware driver (RSP + ros2_control_node + spawners)
    hw_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(g1_hw_share / "launch/hardware.launch.py")),
        launch_arguments={
            "hand_type": hand_type,
            "network_interface": network_interface,
            "kp": kp,
            "kd": kd,
            "waist_kp": waist_kp,
            "waist_kd": waist_kd,
        }.items(),
    )

    _ARM_CONTROLLERS = {"upper_body_controller"}
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
    return LaunchDescription(
        [
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
            DeclareLaunchArgument(
                "kp", default_value="60.0", description="Arm position gain"
            ),
            DeclareLaunchArgument(
                "kd", default_value="1.5", description="Arm velocity (damping) gain"
            ),
            DeclareLaunchArgument(
                "waist_kp", default_value="200.0", description="Waist position gain"
            ),
            DeclareLaunchArgument(
                "waist_kd",
                default_value="2.0",
                description="Waist velocity (damping) gain",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz with MoveIt plugin",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
