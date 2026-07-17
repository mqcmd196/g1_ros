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

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node


class ExampleSonicPoseNode(Node):
    def __init__(self):
        super().__init__("example_sonic_pose_node")
        self.pub_left = self.create_publisher(
            PoseStamped, "/gear_sonic_interface/target_left_wrist_yaw_link", 10
        )
        self.pub_right = self.create_publisher(
            PoseStamped, "/gear_sonic_interface/target_right_wrist_yaw_link", 10
        )
        self.pub_torso = self.create_publisher(
            PoseStamped, "/gear_sonic_interface/target_torso_link", 10
        )
        self.timer = self.create_timer(0.02, self.publish)  # 50 Hz

    def _make_pose(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "pelvis"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def publish(self):
        # Positions in the G1 "pelvis" frame (robot body frame, origin at pelvis).
        # Adjust these values to test different poses.
        self.pub_left.publish(
            self._make_pose(x=0.0, y=0.2, z=0.0)
        )  # left_wrist_yaw_link
        self.pub_right.publish(
            self._make_pose(x=0.0, y=-0.2, z=0.0)
        )  # right_wrist_yaw_link
        self.pub_torso.publish(self._make_pose(x=0.0, y=0.0, z=0.0))  # torso_link


def main(args=None):
    rclpy.init(args=args)
    node = ExampleSonicPoseNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
