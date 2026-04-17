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
G1 real-hardware ROS2 driver launch.

Starts:
  - robot_state_publisher  (URDF with g1_hardware plugin)
  - ros2_control_node      (talks to robot via DDS)

Controller spawning is intentionally left to the bringup layer
so that this launch can be reused regardless of which controllers
the application needs.

Usage:
  ros2 launch g1_hardware hardware.launch.py network_interface:=eth0
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

_HARDWARE_CONFIG = {
    "no_hand": {
        "urdf_xacro": "g1_no_hand_real.urdf.xacro",
        "ros2_controllers": "ros2_controllers_no_hand.yaml",
    },
}


def _launch_setup(context, *args, **kwargs):
    hand_type = LaunchConfiguration("hand_type").perform(context)

    if hand_type not in _HARDWARE_CONFIG:
        raise RuntimeError(
            f"hand_type '{hand_type}' has no real-hardware config yet. "
            f"Available: {list(_HARDWARE_CONFIG)}"
        )

    cfg = _HARDWARE_CONFIG[hand_type]

    network_interface = LaunchConfiguration("network_interface").perform(context)
    kp = LaunchConfiguration("kp").perform(context)
    kd = LaunchConfiguration("kd").perform(context)
    waist_kp = LaunchConfiguration("waist_kp").perform(context)
    waist_kd = LaunchConfiguration("waist_kd").perform(context)

    g1_hw_share = Path(get_package_share_directory("g1_hardware"))

    urdf_path = g1_hw_share / f"config/{cfg['urdf_xacro']}"
    robot_description = xacro.process_file(
        str(urdf_path),
        mappings={
            "network_interface": network_interface,
            "kp": kp,
            "kd": kd,
            "waist_kp": waist_kp,
            "waist_kd": waist_kd,
        },
    ).toxml()

    controllers_yaml = str(g1_hw_share / f"config/{cfg['ros2_controllers']}")

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": robot_description},
                controllers_yaml,
            ],
        ),
    ]


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
            OpaqueFunction(function=_launch_setup),
        ]
    )
