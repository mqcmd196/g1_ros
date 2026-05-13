#!/usr/bin/env python3
"""
Periodically open/close Inspire RH56DFX hands via FollowJointTrajectory.

Examples
--------
  ros2 run g1_bringup inspire_dfx_hand_open_close_demo --ros-args -p hand:=both
  ros2 run g1_bringup inspire_dfx_hand_open_close_demo --ros-args -p hand:=right
  ros2 run g1_bringup inspire_dfx_hand_open_close_demo --ros-args -p hand:=left -p period_sec:=2.0

"""

import math
from typing import Dict, Iterable, List

from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

RIGHT_JOINTS = [
    "R_thumb_proximal_yaw_joint",
    "R_thumb_proximal_pitch_joint",
    "R_index_proximal_joint",
    "R_middle_proximal_joint",
    "R_ring_proximal_joint",
    "R_pinky_proximal_joint",
]

LEFT_JOINTS = [
    "L_thumb_proximal_yaw_joint",
    "L_thumb_proximal_pitch_joint",
    "L_index_proximal_joint",
    "L_middle_proximal_joint",
    "L_ring_proximal_joint",
    "L_pinky_proximal_joint",
]

OPEN_POSITIONS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
CLOSED_POSITIONS = [0.3, 0.6, 1.7, 1.7, 1.7, 1.7]
# CLOSED_POSITIONS = [1.3, 0.6, 1.7, 1.7, 1.7, 1.7]  # Index finger interferes with thumb


def validate_positive(name: str, value: float) -> float:
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be a positive finite value, got {value}")
    return value


class InspireDFXHandOpenCloseDemo(Node):
    def __init__(self) -> None:
        super().__init__("inspire_dfx_hand_open_close_demo")

        self.declare_parameter("hand", "both")
        self.declare_parameter(
            "right_action_name", "/right_hand_controller/follow_joint_trajectory"
        )
        self.declare_parameter(
            "left_action_name", "/left_hand_controller/follow_joint_trajectory"
        )
        self.declare_parameter("period_sec", 1.0)
        self.declare_parameter("move_duration_sec", 0.5)
        self.declare_parameter("server_timeout_sec", 10.0)
        self.declare_parameter("start_open", True)

        self.hand = self.get_parameter("hand").value.lower()
        if self.hand not in ("right", "left", "both"):
            raise ValueError("hand must be one of: right, left, both")

        self.period_sec = validate_positive(
            "period_sec", float(self.get_parameter("period_sec").value)
        )
        self.move_duration_sec = validate_positive(
            "move_duration_sec", float(self.get_parameter("move_duration_sec").value)
        )
        self.server_timeout_sec = validate_positive(
            "server_timeout_sec", float(self.get_parameter("server_timeout_sec").value)
        )
        self.open_next = bool(self.get_parameter("start_open").value)

        self.action_clients: Dict[str, ActionClient] = {}
        self.action_names: Dict[str, str] = {}
        if self.hand in ("right", "both"):
            self.action_names["right"] = self.get_parameter("right_action_name").value
            self.action_clients["right"] = ActionClient(
                self,
                FollowJointTrajectory,
                self.action_names["right"],
            )
        if self.hand in ("left", "both"):
            self.action_names["left"] = self.get_parameter("left_action_name").value
            self.action_clients["left"] = ActionClient(
                self,
                FollowJointTrajectory,
                self.action_names["left"],
            )

        self.wait_for_action_servers()
        self.timer = self.create_timer(self.period_sec, self.timer_callback)

        self.get_logger().info(
            "Started RH56DFX open/close demo: hand=%s, period=%.3fs, "
            "move_duration=%.3fs"
            % (
                self.hand,
                self.period_sec,
                self.move_duration_sec,
            )
        )

    def wait_for_action_servers(self) -> None:
        for side, client in self.action_clients.items():
            if not client.wait_for_server(timeout_sec=self.server_timeout_sec):
                raise RuntimeError(
                    f"{side} hand action server is not available: {self.action_names[side]}"
                )

    def timer_callback(self) -> None:
        label = "open" if self.open_next else "close"
        open_goal = self.open_next
        self.open_next = not self.open_next

        goal_handles = []
        for side, client in self.action_clients.items():
            joint_names = RIGHT_JOINTS if side == "right" else LEFT_JOINTS
            goal_msg = (
                self.make_open_goal(joint_names)
                if open_goal
                else self.make_close_goal(joint_names)
            )
            future = client.send_goal_async(goal_msg)
            goal_handles.append((side, future))

        for side, future in goal_handles:
            future.add_done_callback(
                lambda done_future, side=side, label=label: self.goal_response_callback(
                    side, label, done_future
                )
            )

        self.get_logger().info("Sent %s command to %s hand(s)" % (label, self.hand))

    def make_point(
        self, positions: Iterable[float], time_from_start_sec: float
    ) -> JointTrajectoryPoint:
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start.sec = int(time_from_start_sec)
        point.time_from_start.nanosec = int(
            round((time_from_start_sec - point.time_from_start.sec) * 1e9)
        )
        if point.time_from_start.nanosec >= 1000000000:
            point.time_from_start.sec += 1
            point.time_from_start.nanosec -= 1000000000
        return point

    def make_goal(
        self, joint_names: Iterable[str], points: Iterable[JointTrajectoryPoint]
    ) -> FollowJointTrajectory.Goal:
        trajectory = JointTrajectory()
        trajectory.joint_names = list(joint_names)
        trajectory.points = list(points)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        return goal

    def make_open_goal(self, joint_names: Iterable[str]) -> FollowJointTrajectory.Goal:
        ordered_joint_names = list(joint_names)
        point = self.make_point(OPEN_POSITIONS, self.move_duration_sec)
        return self.make_goal(ordered_joint_names, [point])

    def make_close_goal(self, joint_names: Iterable[str]) -> FollowJointTrajectory.Goal:
        ordered_joint_names = list(joint_names)
        point = self.make_point(CLOSED_POSITIONS, self.move_duration_sec)
        return self.make_goal(ordered_joint_names, [point])

    def goal_response_callback(self, side: str, label: str, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("%s %s goal was rejected" % (side, label))
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done_future, side=side, label=label: self.result_callback(
                side, label, done_future
            )
        )

    def result_callback(self, side: str, label: str, future) -> None:
        result = future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                "%s %s goal failed: error_code=%d, error_string=%s"
                % (side, label, result.error_code, result.error_string)
            )


def main(args: List[str] = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = InspireDFXHandOpenCloseDemo()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
