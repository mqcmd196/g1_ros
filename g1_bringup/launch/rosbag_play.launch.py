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
Play back a rosbag recorded by rosbag_record.launch.py.

Bags hold the transport-compressed sensor streams, so by default this launch
also starts `republish` nodes that decode them back to the raw topics
(sensor_msgs/Image, sensor_msgs/PointCloud2) that downstream nodes and RViz
expect. Set decompress:=false to replay only what is in the bag.

Do not run this against the live robot: the bag republishes /joint_states,
/tf and /robot_description, which would fight the real robot_state_publisher.

Usage:
  ros2 launch g1_bringup rosbag_play.launch.py bag:=/home/unitree/bags/pick_demo

  # Half speed, looping, starting 10 s in
  ros2 launch g1_bringup rosbag_play.launch.py bag:=./pick_demo \
      rate:=0.5 loop:=true start_offset:=10.0

  # Drive nodes off the bag clock (they need use_sim_time:=true themselves)
  ros2 launch g1_bringup rosbag_play.launch.py bag:=./pick_demo clock:=true

Playback is interactive: SPACE pauses/resumes, and Ctrl-C stops.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
import yaml

_D435I_BASE = "/head_camera/d435"
_D435I_COLOR_TOPIC = f"{_D435I_BASE}/color/image_raw"
_D435I_DEPTH_TOPIC = f"{_D435I_BASE}/depth/image_rect_raw"
_D435I_POINTS_TOPIC = f"{_D435I_BASE}/depth/color/points"
_LIVOX_POINTS_TOPIC = "/livox/lidar"


def _bag_topics(bag):
    """
    Return the set of topic names stored in the bag.

    Returns None when the bag's metadata cannot be read (e.g. a bare .mcap
    file was passed instead of a bag directory), so callers can fall back to
    assuming every stream might be present.
    """
    metadata = Path(bag) / "metadata.yaml"
    if not metadata.is_file():
        return None
    try:
        info = yaml.safe_load(metadata.read_text())
        topics = info["rosbag2_bagfile_information"]["topics_with_message_count"]
        return {entry["topic_metadata"]["name"] for entry in topics}
    except (AttributeError, KeyError, TypeError, yaml.YAMLError):
        return None


def _image_decompressor(name, raw_topic, transport):
    """Decode an image_transport-compressed topic back to sensor_msgs/Image."""
    return Node(
        package="image_transport",
        executable="republish",
        name=name,
        parameters=[{"in_transport": transport, "out_transport": "raw"}],
        remappings=[
            (f"in/{transport}", f"{raw_topic}/{transport}"),
            ("out", raw_topic),
        ],
    )


def _points_decompressor(name, raw_topic):
    """Decode a zstd point cloud back to sensor_msgs/PointCloud2."""
    return Node(
        package="point_cloud_transport",
        executable="republish",
        name=name,
        parameters=[{"in_transport": "zstd", "out_transport": "raw"}],
        remappings=[
            ("in/zstd", f"{raw_topic}/zstd"),
            ("out", raw_topic),
        ],
    )


def _launch_setup(context, *args, **kwargs):
    def flag(name):
        return LaunchConfiguration(name).perform(context).lower() in ("true", "1")

    def value(name):
        return LaunchConfiguration(name).perform(context)

    bag = value("bag")
    if not bag:
        raise RuntimeError("bag:=<path to the bag directory> is required")
    if not Path(bag).exists():
        raise RuntimeError(f"bag not found: {bag}")

    clock = flag("clock")

    cmd = ["ros2", "bag", "play", bag, "--rate", value("rate")]

    start_offset = float(value("start_offset"))
    if start_offset > 0.0:
        cmd += ["--start-offset", str(start_offset)]

    if flag("loop"):
        cmd.append("--loop")

    if flag("start_paused"):
        cmd.append("--start-paused")

    if clock:
        cmd += ["--clock", value("clock_rate")]

    topics = value("topics").split()
    if topics:
        cmd += ["--topics"] + topics

    qos_overrides = value("qos_overrides")
    if qos_overrides:
        cmd += ["--qos-profile-overrides-path", qos_overrides]

    actions = []
    if flag("decompress"):
        candidates = [
            (
                f"{_D435I_COLOR_TOPIC}/compressed",
                lambda: _image_decompressor(
                    "d435_color_decompressor", _D435I_COLOR_TOPIC, "compressed"
                ),
            ),
            (
                f"{_D435I_DEPTH_TOPIC}/compressedDepth",
                lambda: _image_decompressor(
                    "d435_depth_decompressor", _D435I_DEPTH_TOPIC, "compressedDepth"
                ),
            ),
            (
                f"{_D435I_POINTS_TOPIC}/zstd",
                lambda: _points_decompressor(
                    "d435_points_decompressor", _D435I_POINTS_TOPIC
                ),
            ),
            (
                f"{_LIVOX_POINTS_TOPIC}/zstd",
                lambda: _points_decompressor(
                    "livox_points_decompressor", _LIVOX_POINTS_TOPIC
                ),
            ),
        ]
        # Only decompress what the bag holds. A republish node advertises its
        # output topic even with no input, so starting all of them would make
        # e.g. /head_camera/d435/color/image_raw appear in `ros2 topic list`
        # and in RViz's topic dropdown while never carrying a single message.
        bag_topics = _bag_topics(bag)
        missing = []
        for in_topic, make_node in candidates:
            if bag_topics is None or in_topic in bag_topics:
                actions.append(make_node())
            else:
                missing.append(in_topic)
        if missing:
            actions.append(
                LogInfo(
                    msg=("not in the bag, so not decompressed: " + " ".join(missing))
                )
            )

    # Only reaches the republishers above; other nodes need their own
    # use_sim_time:=true to follow the bag clock.
    if clock:
        actions.insert(0, SetParameter(name="use_sim_time", value=True))

    actions.append(ExecuteProcess(cmd=cmd, output="screen"))
    return actions


def generate_launch_description():
    default_qos_overrides = str(
        Path(get_package_share_directory("g1_bringup")) / "config/rosbag_play_qos.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag",
                description="Path to the bag directory to play",
            ),
            DeclareLaunchArgument(
                "rate", default_value="1.0", description="Playback rate multiplier"
            ),
            DeclareLaunchArgument(
                "loop",
                default_value="false",
                choices=["true", "false"],
                description="Restart the bag when it reaches the end",
            ),
            DeclareLaunchArgument(
                "start_offset",
                default_value="0.0",
                description="Skip this many seconds from the start of the bag",
            ),
            DeclareLaunchArgument(
                "start_paused",
                default_value="false",
                choices=["true", "false"],
                description="Start paused; press SPACE to begin",
            ),
            DeclareLaunchArgument(
                "clock",
                default_value="false",
                choices=["true", "false"],
                description="Publish /clock from the bag timestamps",
            ),
            DeclareLaunchArgument(
                "clock_rate",
                default_value="100",
                description="/clock publish rate in Hz (with clock:=true)",
            ),
            DeclareLaunchArgument(
                "topics",
                default_value="",
                description="Space-separated subset of topics to play; empty plays all",
            ),
            DeclareLaunchArgument(
                "decompress",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "Republish the compressed image and point-cloud topics as "
                    "raw sensor_msgs, for nodes and RViz that need them"
                ),
            ),
            DeclareLaunchArgument(
                "qos_overrides",
                default_value=default_qos_overrides,
                description=(
                    "rosbag2 QoS override file. The default republishes the "
                    "sensor topics as reliable so the decompress nodes, whose "
                    "subscribers are reliable, actually receive them"
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
