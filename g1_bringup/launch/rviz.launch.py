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
Standalone RViz for an already-running G1 stack.

Runs g1_moveit_config/moveit_rviz.launch.py (the MotionPlanning display and its
MoveIt parameters) with a config that adds the onboard sensors on top of the
MoveIt displays: the zstd-compressed D435i point cloud, the compressed D435i
color image and the Livox Mid-360 cloud.

The rest of the stack (g1_bringup.launch.py or hardware.launch.py, which
publish /robot_description, TF and the sensor topics) must already be running;
the D435i topics need use_d435i:=true there.

Usage:
  ros2 launch g1_bringup rviz.launch.py
  ros2 launch g1_bringup rviz.launch.py rviz_config:=/path/to/other.rviz
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    g1_bringup_share = Path(get_package_share_directory("g1_bringup"))
    g1_moveit_share = Path(get_package_share_directory("g1_moveit_config"))

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(g1_bringup_share / "config/g1.rviz"),
                description=(
                    "RViz config; the default is the MoveIt config plus the "
                    "D435i and Livox Mid-360 displays"
                ),
            ),
            # moveit_rviz.launch.py declares rviz_config itself; the value
            # declared above is inherited, so our config wins unless the user
            # overrides it on the command line.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(g1_moveit_share / "launch/moveit_rviz.launch.py")
                ),
                launch_arguments={
                    "rviz_config": LaunchConfiguration("rviz_config")
                }.items(),
            ),
        ]
    )
