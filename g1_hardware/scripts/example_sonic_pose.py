#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node


class ExampleSonicPoseNode(Node):
    def __init__(self):
        super().__init__("example_sonic_pose_node")
        self.pub_left = self.create_publisher(
            PoseStamped, "/gear_sonic_interface/left_wrist", 10
        )
        self.pub_right = self.create_publisher(
            PoseStamped, "/gear_sonic_interface/right_wrist", 10
        )
        self.pub_head = self.create_publisher(
            PoseStamped, "/gear_sonic_interface/head", 10
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
        self.pub_left.publish(self._make_pose(x=0.3, y=0.3, z=0.0))  # left wrist
        self.pub_right.publish(self._make_pose(x=0.3, y=-0.3, z=0.0))  # right wrist
        self.pub_head.publish(self._make_pose(x=0.0, y=0.0, z=0.5))  # head


def main(args=None):
    rclpy.init(args=args)
    node = ExampleSonicPoseNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
