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
Record a rosbag of the G1 topics.

Everything worth keeping is recorded, always, with the bandwidth-heavy
sensor streams stored as their transport-compressed variants;
rosbag_play.launch.py restores the raw streams on playback. Measured on the
robot (D435i 640x480, MID-360):

  /head_camera/d435/color/image_raw               13.9  MB/s
                              .../compressed       1.4  MB/s
  /head_camera/d435/depth/image_rect_raw           9.3  MB/s
                              .../compressedDepth  1.9  MB/s
  /head_camera/d435/depth/color/points            89.2  MB/s
                              .../zstd            11.7  MB/s
  /livox/lidar                                     4.5  MB/s
                    .../zstd                       2.4  MB/s

Total is roughly 15 MB/s, i.e. ~53 GB per hour, of which the D435i point
cloud is ~11.7 MB/s; record_points:=false drops it to ~5.5 MB/s and lets
rosbag_play.launch.py rebuild an approximate cloud instead.

Note that a recorded cloud replays poorly: point_cloud_transport's zstd
decompressor delivers only 0-3 Hz of the 6.8 Hz in the bag, in bursts. Use
record_points:=false if the cloud is meant to be watched in RViz, and
record_points:=true only when the exact cloud the robot published matters
more than smooth playback.

Action feedback and status are included, so a bag shows which MoveIt and
FollowJointTrajectory goals ran and how they ended. Goals and results
themselves are services, and would need service introspection enabled on
the action servers, which this stack does not do.

Usage:
  ros2 launch g1_bringup rosbag_record.launch.py
  ros2 launch g1_bringup rosbag_record.launch.py output:=/home/unitree/bags/pick_demo
  ros2 launch g1_bringup rosbag_record.launch.py record_points:=false
  ros2 launch g1_bringup rosbag_record.launch.py \
      extra_topics:="/my/topic /another/topic"

Stop the recording with Ctrl-C: launch forwards SIGINT to `ros2 bag record`,
which closes the bag cleanly.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_D435I_BASE = "/head_camera/d435"
_LIVOX_POINTS_TOPIC = "/livox/lidar"

# Recorded only with record_points:=true (~11.7 MB/s, ~80% of the bag).
# Recording it is the only way to reproduce what the robot publishes:
# librealsense keeps the full, wider depth FOV and samples color per point,
# while rebuilding on playback has to reproject depth into the color frame,
# which drops everything outside the narrower color FOV -- measured 119k of
# 242k points, in d435_color_optical_frame instead of
# d435_depth_optical_frame. zstd is lossless, so replaying this topic gives
# byte-identical clouds. With record_points:=false,
# rosbag_play.launch.py rebuilds an approximation instead.
#
# g1_bringup.launch.py already republishes this to zstd when use_d435i:=true,
# so this launch must not start a second republisher for it: two publishers
# on one topic would put every cloud into the bag twice.
_D435I_POINTS_ZSTD_TOPIC = f"{_D435I_BASE}/depth/color/points/zstd"

# Action goal/result live on services; only feedback and status are topics.
# They are hidden topics, hence --include-hidden-topics below.
_ACTION_NAMES = [
    "/move_action",
    "/execute_trajectory",
    "/upper_body_controller/follow_joint_trajectory",
    "/gear_sonic_controller/follow_joint_trajectory",
    "/left_hand_controller/follow_joint_trajectory",
    "/right_hand_controller/follow_joint_trajectory",
]

# compressedDepth is PNG, hence lossless; /compressed (JPEG) would not be for
# 16UC1 depth. realsense2_camera advertises both through image_transport, so
# these topics already exist without any republisher of ours.
_TOPICS = [
    # Robot state: enough to replay the kinematics and re-run TF consumers.
    "/tf",
    "/tf_static",
    "/robot_description",
    "/robot_description_semantic",
    "/joint_states",
    "/dynamic_joint_states",
    # Commanded trajectories, to compare against the achieved /joint_states.
    # The hand controllers only exist with hand_type:=inspire_dfq; rosbag2
    # warns about absent topics rather than failing.
    "/upper_body_controller/joint_trajectory",
    "/upper_body_controller/controller_state",
    "/left_hand_controller/joint_trajectory",
    "/left_hand_controller/controller_state",
    "/right_hand_controller/joint_trajectory",
    "/right_hand_controller/controller_state",
    # Head camera. camera_info and extrinsics stay raw: they are tiny, and
    # the image streams cannot be interpreted without them.
    f"{_D435I_BASE}/color/camera_info",
    f"{_D435I_BASE}/depth/camera_info",
    f"{_D435I_BASE}/extrinsics/depth_to_color",
    f"{_D435I_BASE}/color/image_raw/compressed",
    f"{_D435I_BASE}/depth/image_rect_raw/compressedDepth",
    # Livox MID-360. Nothing else in the stack produces /livox/lidar/zstd,
    # so this launch runs that republisher below.
    "/livox/imu",
    f"{_LIVOX_POINTS_TOPIC}/zstd",
    # gear_sonic (SONIC) command side; resulting motion is in /joint_states.
    "/cmd_vel",
    "/gear_sonic_interface/target_left_wrist_yaw_link",
    "/gear_sonic_interface/target_right_wrist_yaw_link",
    "/gear_sonic_interface/target_torso_link",
    "/gear_sonic_interface/target_height",
    "/gear_sonic_interface/smpl_motion",
    # MoveIt planning scene and planned trajectories.
    "/display_planned_path",
    "/monitored_planning_scene",
    "/planning_scene",
    "/collision_object",
    "/attached_collision_object",
    "/trajectory_execution_event",
] + [
    f"{name}/_action/{suffix}"
    for name in _ACTION_NAMES
    for suffix in ("feedback", "status")
]


def _launch_setup(context, *args, **kwargs):
    cmd = [
        "ros2",
        "bag",
        "record",
        "--storage",
        "mcap",
        # zstd-compresses the mcap chunks while keeping the bag seekable,
        # unlike rosbag2's --compression-mode file.
        "--storage-preset-profile",
        "zstd_fast",
        "--include-hidden-topics",
    ]

    output = LaunchConfiguration("output").perform(context)
    if output:
        cmd += ["--output", output]

    topics = list(_TOPICS)
    if LaunchConfiguration("record_points").perform(context).lower() in ("true", "1"):
        topics.append(_D435I_POINTS_ZSTD_TOPIC)

    for topic in LaunchConfiguration("extra_topics").perform(context).split():
        if topic not in topics:
            topics.append(topic)

    cmd += ["--topics"] + topics

    livox_republisher = Node(
        package="point_cloud_transport",
        executable="republish",
        name="livox_points_record_republisher",
        parameters=[{"in_transport": "raw", "out_transport": "zstd"}],
        remappings=[
            ("in", _LIVOX_POINTS_TOPIC),
            ("out/zstd", f"{_LIVOX_POINTS_TOPIC}/zstd"),
        ],
    )

    return [livox_republisher, ExecuteProcess(cmd=cmd, output="screen")]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "record_points",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "Record the D435i point cloud (~11.7 MB/s, ~80% of the "
                    "bag). With false, rosbag_play.launch.py rebuilds an "
                    "approximation from the depth and color images instead"
                ),
            ),
            DeclareLaunchArgument(
                "extra_topics",
                default_value="",
                description=(
                    "Space-separated topics to record in addition to the "
                    "built-in list"
                ),
            ),
            DeclareLaunchArgument(
                "output",
                default_value="",
                description=(
                    "Bag directory to create. Empty lets rosbag2 pick a "
                    "timestamped name in the current directory"
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
