#!/usr/bin/env python3

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
Replay a recorded gear_sonic motion clip to GearSonicInterface over ROS.

Loads pose_*.npz chunks (from extract_jump_clip.py / pico_manager --record_dir) or a
single consolidated .npz, concatenates them into one frame timeline, and publishes one
gear_sonic_interfaces/SmplMotion per frame at the target rate on <prefix>/smpl_motion.

The motion is framed so the robot stays upright: enable_control(true), wait idle_before,
enable_smpl_stream(true), publish the frames, then enable_smpl_stream(false).

Run it with ``ros2 run g1_hardware replay_smpl_motion.py --ros-args -p clip:=<path>``.
"""

import glob
import os

from gear_sonic_interfaces.msg import SmplMotion
import numpy as np
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


def _load_clip(clip_path):
    # Accept either a single consolidated .npz file (e.g. the bundled data/smpl_jump1.npz)
    # or a directory of pose_*.npz chunks (extract_jump_clip.py output).
    if os.path.isdir(clip_path):
        paths = sorted(glob.glob(os.path.join(clip_path, "pose_*.npz")))
        if not paths:
            raise RuntimeError(f"No pose_*.npz found in {clip_path!r}")
    elif os.path.isfile(clip_path):
        paths = [clip_path]
    else:
        raise RuntimeError(f"clip path not found: {clip_path!r}")
    # Concatenate chunks, dedup by frame_index (chunks overlap as sliding windows).
    seen = {}
    for p in paths:
        d = np.load(p)
        fi = d["frame_index"]
        for j, f in enumerate(fi):
            seen[int(f)] = {
                "smpl_pose": np.asarray(d["smpl_pose"][j], np.float64).reshape(-1),
                "smpl_joints": np.asarray(d["smpl_joints"][j], np.float64).reshape(-1),
                "body_quat_w": np.asarray(d["body_quat_w"][j], np.float64).reshape(-1),
                "joint_pos": (
                    np.asarray(d["joint_pos"][j], np.float64).reshape(-1)
                    if "joint_pos" in d
                    else np.zeros(29)
                ),
                "joint_vel": (
                    np.asarray(d["joint_vel"][j], np.float64).reshape(-1)
                    if "joint_vel" in d
                    else np.zeros(29)
                ),
            }
    order = sorted(seen)
    return [seen[f] for f in order]


class ReplaySmplMotionNode(Node):
    def __init__(self):
        super().__init__("replay_smpl_motion")
        self.clip_path = self.declare_parameter("clip", "").value
        self.prefix = self.declare_parameter("prefix", "/gear_sonic_interface").value
        self.rate = self.declare_parameter("rate", 50.0).value
        self.idle_before = self.declare_parameter("idle_before", 3.0).value
        self.idle_after = self.declare_parameter("idle_after", 3.0).value
        self.manage = self.declare_parameter("manage_stream", True).value

        if not self.clip_path:
            raise RuntimeError(
                "Set the 'clip' parameter to a pose_*.npz directory or .npz file"
            )
        self.frames = _load_clip(self.clip_path)
        self.get_logger().info(
            f"Loaded {len(self.frames)} frames from {self.clip_path} "
            f"(~{len(self.frames) / max(1.0, self.rate):.2f}s @ {self.rate:.0f} Hz)"
        )

        self.pub = self.create_publisher(SmplMotion, f"{self.prefix}/smpl_motion", 10)
        self.enable_control = self.create_client(
            SetBool, f"{self.prefix}/enable_control"
        )
        self.enable_stream = self.create_client(
            SetBool, f"{self.prefix}/enable_smpl_stream"
        )

    def _call(self, client, value):
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"service {client.srv_name} unavailable")
            return False
        fut = client.call_async(SetBool.Request(data=value))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return fut.result() is not None and fut.result().success

    def run(self):
        if self.manage:
            self.get_logger().info("enable_control(true): balancing before motion")
            self._call(self.enable_control, True)
            self._sleep(self.idle_before)

        # Everything after streaming is enabled goes in try/finally so the robot is ALWAYS
        # handed back to planner balance — even on Ctrl-C or an exception. Otherwise the node
        # stays stuck in streamed-motion mode and ignores subsequent planner/VR commands.
        try:
            if self.manage:
                self.get_logger().info("enable_smpl_stream(true): streamed-motion mode")
                self._call(self.enable_stream, True)

            period = 1.0 / max(1e-6, self.rate)
            for i, fr in enumerate(self.frames):
                msg = SmplMotion()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.frame_index = i
                msg.smpl_pose = fr["smpl_pose"].tolist()
                msg.smpl_joints = fr["smpl_joints"].tolist()
                q = fr["body_quat_w"]  # recorded as scalar-first (w, x, y, z)
                msg.body_quat.w = float(q[0])
                msg.body_quat.x = float(q[1])
                msg.body_quat.y = float(q[2])
                msg.body_quat.z = float(q[3])
                msg.joint_pos = fr["joint_pos"].tolist()
                msg.joint_vel = fr["joint_vel"].tolist()
                self.pub.publish(msg)
                self._sleep(period)
            self.get_logger().info(f"published {len(self.frames)} SmplMotion frames")
        finally:
            if self.manage:
                self.get_logger().info(
                    "enable_smpl_stream(false): back to planner balance"
                )
                self._call(self.enable_stream, False)
                self._sleep(self.idle_after)

    def _sleep(self, seconds):
        # Spin while sleeping so service futures/timers progress.
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.005)


def main(args=None):
    rclpy.init(args=args)
    node = ReplaySmplMotionNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
