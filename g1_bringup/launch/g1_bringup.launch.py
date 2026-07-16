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
  # With the gear_sonic (SONIC) locomotion bridge (real robot: bind on all
  # interfaces so the on-robot deploy stack can connect; keep
  # upper_body_controller inactive while SONIC is in control):
  ros2 launch g1_bringup g1_bringup.launch.py network_interface:=eth0 \
      use_gear_sonic:=true zmq_host:=0.0.0.0
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
            "upper_body_controller",
        ],
    },
    "inspire_dfq": {
        "urdf_xacro": "g1_inspire_dfq_real.urdf.xacro",
        "srdf": "g1_inspire_dfq.srdf",
        "controllers": [
            "upper_body_controller",
            "right_hand_controller",
            "left_hand_controller",
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
    inspire_command_topic = LaunchConfiguration("inspire_command_topic").perform(
        context
    )
    inspire_state_topic = LaunchConfiguration("inspire_state_topic").perform(context)
    inspire_state_timeout_sec = LaunchConfiguration(
        "inspire_state_timeout_sec"
    ).perform(context)
    use_rviz = LaunchConfiguration("use_rviz").perform(context).lower()
    use_gear_sonic = LaunchConfiguration("use_gear_sonic").perform(context)
    zmq_host = LaunchConfiguration("zmq_host").perform(context)

    cfg = _HARDWARE_CONFIG[hand_type]

    g1_hw_share = Path(get_package_share_directory("g1_hardware"))
    g1_moveit_share = Path(get_package_share_directory("g1_moveit_config"))

    # Build moveit_config for MoveIt nodes (RSP is handled by hardware launch)
    moveit_config_builder = (
        MoveItConfigsBuilder("g1_29dof", package_name="g1_moveit_config")
        .robot_description(
            file_path=str(g1_hw_share / f"config/{cfg['urdf_xacro']}"),
            mappings={
                "network_interface": network_interface,
                "kp": kp,
                "kd": kd,
                "waist_kp": waist_kp,
                "waist_kd": waist_kd,
                "inspire_command_topic": inspire_command_topic,
                "inspire_state_topic": inspire_state_topic,
                "inspire_state_timeout_sec": inspire_state_timeout_sec,
            },
        )
        .robot_description_semantic(
            file_path=str(g1_moveit_share / f"config/{cfg['srdf']}"),
        )
        .planning_pipelines(
            pipelines=["ompl"],
            default_planning_pipeline="ompl",
        )
    )
    if use_gear_sonic.lower() == "true":
        # Execute trajectories through the gear_sonic FK adapter instead of the
        # ros2_control upper_body_controller (which conflicts with SONIC).
        moveit_config_builder = moveit_config_builder.trajectory_execution(
            file_path=str(
                g1_moveit_share / "config/moveit_controllers_gear_sonic.yaml"
            ),
        )
    moveit_config = moveit_config_builder.to_moveit_configs()

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
            "inspire_command_topic": inspire_command_topic,
            "inspire_state_topic": inspire_state_topic,
            "inspire_state_timeout_sec": inspire_state_timeout_sec,
            "use_gear_sonic": use_gear_sonic,
            "zmq_host": zmq_host,
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
                "inspire_command_topic",
                default_value="rt/inspire/cmd",
                description="DDS topic for RH56DFX commands",
            ),
            DeclareLaunchArgument(
                "inspire_state_topic",
                default_value="rt/inspire/state",
                description="DDS topic for RH56DFX state",
            ),
            DeclareLaunchArgument(
                "inspire_state_timeout_sec",
                default_value="3.0",
                description="Seconds to wait for the first RH56DFX state message",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz with MoveIt plugin",
            ),
            DeclareLaunchArgument(
                "use_gear_sonic",
                default_value="false",
                choices=["true", "false"],
                description="Start the gear_sonic (SONIC) locomotion bridge",
            ),
            DeclareLaunchArgument(
                "zmq_host",
                default_value="127.0.0.1",
                description=(
                    "gear_sonic_interface XPUB bind address (only used with "
                    "use_gear_sonic:=true). Default accepts the deploy stack on "
                    "this machine only (sim); use 0.0.0.0 for the real robot"
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
