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

Bandwidth-heavy sensor streams are recorded as their transport-compressed
variants; rosbag_play.launch.py restores the raw streams on playback.
Measured on the robot (D435i 848x480, MID-360):

  /head_camera/d435/color/image_raw               13.9  MB/s
                              .../compressed       1.4  MB/s
  /head_camera/d435/depth/image_rect_raw           9.3  MB/s
                              .../compressedDepth  1.9  MB/s
  /head_camera/d435/depth/color/points            89.2  MB/s
                              .../zstd            11.7  MB/s
  /livox/lidar                                     4.5  MB/s
                    .../zstd                       2.4  MB/s

The D435i point cloud dominates the total even compressed, and it is fully
reconstructible from the depth image plus camera_info, so it has its own
opt-in group (record_d435i_points) rather than riding along with the camera.

Action feedback/status is recorded by default (record_actions), so a bag
shows which MoveIt / FollowJointTrajectory goals ran and how they ended.

Usage:
  # Robot state + actions + Livox (the default groups): ~2.6 MB/s
  ros2 launch g1_bringup rosbag_record.launch.py

  # Add the head camera, into a named bag directory: ~5.9 MB/s
  ros2 launch g1_bringup rosbag_record.launch.py \
      record_d435i:=true output:=/home/unitree/bags/pick_demo

  # Raw (uncompressed) streams, e.g. to check the compression itself
  ros2 launch g1_bringup rosbag_record.launch.py record_d435i:=true compress:=false

  # Everything except the raw image/point-cloud streams
  ros2 launch g1_bringup rosbag_record.launch.py record_all:=true

Stop the recording with Ctrl-C: launch forwards SIGINT to `ros2 bag record`,
which closes the bag cleanly.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_D435I_BASE = "/head_camera/d435"

# Robot state: enough to replay the kinematics and re-run TF-dependent nodes.
_CORE_TOPICS = [
    "/tf",
    "/tf_static",
    "/robot_description",
    "/robot_description_semantic",
    "/joint_states",
    "/dynamic_joint_states",
]

# Commanded trajectories, to compare against the achieved /joint_states. The
# hand controllers only exist with hand_type:=inspire_dfq; `ros2 bag record`
# just warns about the others rather than failing.
_CONTROLLER_TOPICS = [
    "/upper_body_controller/joint_trajectory",
    "/upper_body_controller/controller_state",
    "/left_hand_controller/joint_trajectory",
    "/left_hand_controller/controller_state",
    "/right_hand_controller/joint_trajectory",
    "/right_hand_controller/controller_state",
]

# Action goal/result live on services, and no node in this stack enables
# service introspection, so only the feedback and status topics can be
# recorded. Those carry the useful part anyway: which goal is running, its
# progress, and how it terminated. They are hidden topics, hence the
# --include-hidden-topics below. /head_camera/d435/triggered_calibration is
# left out deliberately -- it is a camera maintenance action, not motion.
_ACTION_NAMES = [
    "/move_action",
    "/execute_trajectory",
    "/upper_body_controller/follow_joint_trajectory",
    "/gear_sonic_controller/follow_joint_trajectory",
    "/left_hand_controller/follow_joint_trajectory",
    "/right_hand_controller/follow_joint_trajectory",
]
_ACTION_TOPICS = [
    f"{name}/_action/{suffix}"
    for name in _ACTION_NAMES
    for suffix in ("feedback", "status")
]

_LIVOX_IMU_TOPIC = "/livox/imu"
_LIVOX_POINTS_TOPIC = "/livox/lidar"

_D435I_COLOR_TOPIC = f"{_D435I_BASE}/color/image_raw"
_D435I_DEPTH_TOPIC = f"{_D435I_BASE}/depth/image_rect_raw"
_D435I_POINTS_TOPIC = f"{_D435I_BASE}/depth/color/points"
# Small, and the image streams are unusable without them; always raw.
_D435I_INFO_TOPICS = [
    f"{_D435I_BASE}/color/camera_info",
    f"{_D435I_BASE}/depth/camera_info",
    f"{_D435I_BASE}/extrinsics/depth_to_color",
]

# gear_sonic (SONIC) command side; the resulting motion is in /joint_states.
_GEAR_SONIC_TOPICS = [
    "/cmd_vel",
    "/gear_sonic_interface/target_left_wrist_yaw_link",
    "/gear_sonic_interface/target_right_wrist_yaw_link",
    "/gear_sonic_interface/target_torso_link",
    "/gear_sonic_interface/target_height",
    "/gear_sonic_interface/smpl_motion",
]

_MOVEIT_TOPICS = [
    "/display_planned_path",
    "/monitored_planning_scene",
    "/planning_scene",
    "/collision_object",
    "/attached_collision_object",
    "/trajectory_execution_event",
]

# Raw sensor streams kept out of `record_all:=true` bags. The compressed
# variants carry a transport suffix, so they do not match and are still kept.
_RAW_SENSOR_EXCLUDE_REGEX = (
    "(/color/image_raw|/depth/image_rect_raw|/depth/color/points|/livox/lidar)$"
)


def _zstd_republisher(name, raw_topic):
    """Republish a PointCloud2 as zstd, so rosbag2 can record the small form."""
    return Node(
        package="point_cloud_transport",
        executable="republish",
        name=name,
        parameters=[{"in_transport": "raw", "out_transport": "zstd"}],
        remappings=[
            ("in", raw_topic),
            ("out/zstd", f"{raw_topic}/zstd"),
        ],
    )


def _launch_setup(context, *args, **kwargs):
    def flag(name):
        return LaunchConfiguration(name).perform(context).lower() in ("true", "1")

    def value(name):
        return LaunchConfiguration(name).perform(context)

    compress = flag("compress")
    record_all = flag("record_all")
    record_livox = flag("record_livox")
    record_d435i_points = flag("record_d435i_points")
    record_actions = flag("record_actions")

    topics = []
    republishers = []

    if flag("record_core"):
        topics += _CORE_TOPICS

    if flag("record_controllers"):
        topics += _CONTROLLER_TOPICS

    if record_actions:
        topics += _ACTION_TOPICS

    if record_livox or record_all:
        if record_livox:
            topics.append(_LIVOX_IMU_TOPIC)
            topics.append(
                f"{_LIVOX_POINTS_TOPIC}/zstd" if compress else _LIVOX_POINTS_TOPIC
            )
        if compress:
            # Nothing else in the stack produces /livox/lidar/zstd.
            republishers.append(
                _zstd_republisher(
                    "livox_points_record_republisher", _LIVOX_POINTS_TOPIC
                )
            )

    if flag("record_d435i"):
        topics += _D435I_INFO_TOPICS
        if compress:
            # realsense2_camera advertises its images through image_transport,
            # so the /compressed and /compressedDepth topics already exist
            # (compressed_image_transport and compressed_depth_image_transport
            # are exec_depends of this package). compressedDepth is PNG, hence
            # lossless, which /compressed (JPEG) would not be for 16UC1 depth.
            topics += [
                f"{_D435I_COLOR_TOPIC}/compressed",
                f"{_D435I_DEPTH_TOPIC}/compressedDepth",
            ]
        else:
            topics += [_D435I_COLOR_TOPIC, _D435I_DEPTH_TOPIC]

    if record_d435i_points or record_all:
        if record_d435i_points:
            topics.append(
                f"{_D435I_POINTS_TOPIC}/zstd" if compress else _D435I_POINTS_TOPIC
            )
        if compress:
            # g1_bringup.launch.py already runs this republisher when
            # use_d435i:=true; only start our own if that one is absent.
            republishers.append(
                _zstd_republisher("d435_points_record_republisher", _D435I_POINTS_TOPIC)
            )

    if flag("record_gear_sonic"):
        topics += _GEAR_SONIC_TOPICS

    if flag("record_moveit"):
        topics += _MOVEIT_TOPICS

    topics += value("extra_topics").split()

    cmd = ["ros2", "bag", "record", "--storage", value("storage")]

    preset = value("storage_preset_profile")
    if preset:
        cmd += ["--storage-preset-profile", preset]

    output = value("output")
    if output:
        cmd += ["--output", output]

    max_bag_duration = int(value("max_bag_duration"))
    if max_bag_duration > 0:
        cmd += ["--max-bag-duration", str(max_bag_duration)]

    if record_actions:
        # _action/* are hidden topics; rosbag2 skips them without this.
        cmd.append("--include-hidden-topics")

    if record_all:
        cmd.append("--all")
        if compress:
            cmd += ["--exclude-regex", _RAW_SENSOR_EXCLUDE_REGEX]
    elif topics:
        cmd += ["--topics"] + topics
    else:
        raise RuntimeError(
            "No topics selected: enable a record_* group, pass extra_topics, "
            "or use record_all:=true"
        )

    return republishers + [ExecuteProcess(cmd=cmd, output="screen")]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output",
                default_value="",
                description=(
                    "Bag directory to create. Empty lets rosbag2 pick a "
                    "timestamped name in the current directory"
                ),
            ),
            DeclareLaunchArgument(
                "compress",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "Record the transport-compressed image/point-cloud topics "
                    "instead of the raw ones"
                ),
            ),
            DeclareLaunchArgument(
                "record_core",
                default_value="true",
                choices=["true", "false"],
                description="Record TF, joint states and the robot description",
            ),
            DeclareLaunchArgument(
                "record_controllers",
                default_value="true",
                choices=["true", "false"],
                description="Record the ros2_control trajectory commands and states",
            ),
            DeclareLaunchArgument(
                "record_actions",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "Record the MoveIt and FollowJointTrajectory action "
                    "feedback/status topics. Goals and results are services "
                    "and would need service introspection enabled on the "
                    "action servers, which this stack does not do"
                ),
            ),
            DeclareLaunchArgument(
                "record_livox",
                default_value="true",
                choices=["true", "false"],
                description="Record the Livox MID-360 point cloud and IMU",
            ),
            DeclareLaunchArgument(
                "record_d435i",
                default_value="false",
                choices=["true", "false"],
                description="Record the head D435i color and depth images",
            ),
            DeclareLaunchArgument(
                "record_d435i_points",
                default_value="false",
                choices=["true", "false"],
                description=(
                    "Record the D435i point cloud. Costs ~11.7 MB/s even as "
                    "zstd and is reconstructible from depth + camera_info"
                ),
            ),
            DeclareLaunchArgument(
                "record_gear_sonic",
                default_value="false",
                choices=["true", "false"],
                description="Record the gear_sonic (SONIC) locomotion commands",
            ),
            DeclareLaunchArgument(
                "record_moveit",
                default_value="false",
                choices=["true", "false"],
                description="Record the planning scene and planned trajectories",
            ),
            DeclareLaunchArgument(
                "extra_topics",
                default_value="",
                description="Space-separated additional topics to record",
            ),
            DeclareLaunchArgument(
                "record_all",
                default_value="false",
                choices=["true", "false"],
                description=(
                    "Record every topic instead of the record_* groups. With "
                    "compress:=true the raw image and point-cloud streams are "
                    "excluded"
                ),
            ),
            DeclareLaunchArgument(
                "max_bag_duration",
                default_value="0",
                description="Split the bag every N seconds; 0 disables splitting",
            ),
            DeclareLaunchArgument(
                "storage",
                default_value="mcap",
                choices=["mcap", "sqlite3"],
                description="rosbag2 storage plugin",
            ),
            DeclareLaunchArgument(
                "storage_preset_profile",
                default_value="zstd_fast",
                description=(
                    "Storage-level preset. 'zstd_fast' zstd-compresses the mcap "
                    "chunks while keeping the bag seekable, unlike rosbag2's "
                    "--compression-mode file. Clear it for storage:=sqlite3"
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
