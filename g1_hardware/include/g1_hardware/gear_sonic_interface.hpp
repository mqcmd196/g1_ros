// Copyright 2026 Yoshiki Obinata
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Yoshiki Obinata nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <array>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <zmq.hpp>

namespace g1_hardware
{

/**
 * ROS 2 node that bridges ROS commands to the SONIC deploy stack via ZMQ.
 * Mirrors pico_manager StreamMode::PLANNER_VR_3PT: locomotion commands and
 * VR 3-point upper-body targets are combined in a single planner message,
 * so walking and arm control work simultaneously (unlike the PICO UI which
 * disables one while the other is active).
 *
 * ── Services ──────────────────────────────────────────────────────────────
 *   ~/enable_control  (std_srvs/SetBool)
 *       true  → (1) send planner{IDLE, zero vel} first, then command{start=1, planner=1}
 *               Mirrors pico_manager order: planner message before start command, so the
 *               deploy stack has planner data when entering CONTROL (prevents bad initial pose).
 *               Also resets heading and starts the timer.
 *       false → stop sending planner messages only (no stop=1 command).
 *               Deploy reverts to IDLE after its 1-second planner timeout; robot stays balanced.
 *
 *   ~/mode/default    (std_srvs/Trigger) — auto-select gait from cmd_vel magnitude (default)
 *       mag < 0.05       → IDLE
 *       0.05 ≤ mag < 0.6 → SLOW_WALK
 *       0.6  ≤ mag < 1.5 → WALK  (speed auto)
 *       mag ≥ 1.5        → RUN
 *
 *   ~/mode/<name>     (std_srvs/Trigger) — fix a specific locomotion mode
 *       idle_squat, idle_kneel_two_legs, idle_kneel, idle_lying_face_down,
 *       crawling, idle_boxing, walk_boxing, left_punch, right_punch,
 *       random_punch, elbow_crawling, left_hook, right_hook, forward_jump,
 *       stealth_walk, injured_walk
 *
 * ── Topics ────────────────────────────────────────────────────────────────
 *   ~/cmd_vel     (geometry_msgs/Twist, subscribe)
 *       linear.x  → forward speed  (m/s, robot body frame)
 *       linear.y  → strafe speed   (m/s, robot body frame, positive = left)
 *       angular.z → turn rate      (rad/s); accumulated into heading angle
 *       speed is always -1 (auto) as per SONIC's default
 *
 *   ~/left_wrist  (geometry_msgs/PoseStamped, subscribe)
 *   ~/right_wrist (geometry_msgs/PoseStamped, subscribe)
 *   ~/head        (geometry_msgs/PoseStamped, subscribe)
 *       VR 3-point upper-body targets.  All three must be received before
 *       vr_position/vr_orientation are included in the planner message.
 *       Until then, bare planner messages (locomotion only) are sent.
 *
 * ── Coordinate frame for PoseStamped positions ────────────────────────────
 *   Positions must be in the G1's "pelvis" TF frame (URDF root link).
 *   Formula: local_pos = inv(root_quat) * (world_pos - root_pos)
 *   This is equivalent to a tf2 transform into the "pelvis" frame:
 *     pose_pelvis = tf_buffer.transform(pose_world, "pelvis")
 *   The deploy stack uses vr_position values directly without conversion
 *   (g1_deploy_onnx_ref.cpp GatherVR3PointPosition():
 *    "The sender has already applied offsets and root normalization").
 *
 * ── Parameters ────────────────────────────────────────────────────────────
 *   zmq_port        (int,    default 5556)
 *   publish_rate    (double, default 50.0 Hz)
 *   cmd_vel_timeout (double, default 0.5 s) — cmd_vel zeroed if no message within this window
 *
 * Deploy stack launch (on robot):
 *   ./deploy.sh --input-type zmq_manager --zmq-host <IP of this machine> real
 */
class GearSonicInterface : public rclcpp::Node
{
public:
  explicit GearSonicInterface(const rclcpp::NodeOptions& options);
  ~GearSonicInterface();

private:
  void OnEnableControl(std_srvs::srv::SetBool::Request::SharedPtr req,
                       std_srvs::srv::SetBool::Response::SharedPtr res);
  void OnCmdVel(geometry_msgs::msg::Twist::ConstSharedPtr msg);
  void OnLeftWrist(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void OnRightWrist(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void OnHead(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void TimerCallback();

  // ZMQ wire-format builders (replicates gear_sonic/utils/teleop/zmq/zmq_planner_sender.py)
  static std::vector<uint8_t> BuildZmqHeader(const std::string& fields_json, int version = 1);
  static std::vector<uint8_t> BuildCommandMessage(bool start, bool stop, bool planner);
  // vr_position and vr_orientation are optional (nullptr = field omitted).
  // This matches PLANNER_VR_3PT where both are None until VR data arrives.
  static std::vector<uint8_t>
  BuildPlannerMessage(int mode, std::array<float, 3> movement, std::array<float, 3> facing,
                      float speed, float height, const std::array<float, 9>* vr_position = nullptr,
                      const std::array<float, 12>* vr_orientation = nullptr);

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_control_srv_;
  // One Trigger service per LocomotionMode (indexed by mode value 0–19)
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> mode_services_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_wrist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_wrist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr head_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex data_mutex_;

  // Locomotion state
  int locomotion_mode_{-1};       // -1 = auto (IDLE/SLOW_WALK/WALK/RUN from cmd_vel mag)
  double cmd_vel_x_{0.0};         // forward speed  (body frame, m/s)
  double cmd_vel_y_{0.0};         // strafe speed   (body frame, m/s, positive = left)
  double cmd_angular_z_{0.0};     // turn rate      (rad/s); integrated each timer tick
  double heading_rad_{0.0};       // accumulated heading angle (rad)
  double publish_dt_{1.0 / 50.0}; // updated in constructor from publish_rate param
  rclcpp::Time last_cmd_vel_time_{0, 0, RCL_ROS_TIME}; // last cmd_vel receive time
  double cmd_vel_timeout_{0.5}; // seconds; cmd_vel zeroed if no message within this window

  // VR 3-point upper-body targets
  // vr_position: [Lw_x,y,z, Rw_x,y,z, H_x,y,z]  (pelvis frame)
  std::array<float, 9> vr_position_{};
  // vr_orientation: [Lw_qw,qx,qy,qz, Rw_qw,qx,qy,qz, H_qw,qx,qy,qz]
  std::array<float, 12> vr_orientation_{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0};
  bool left_received_{false};
  bool right_received_{false};
  bool head_received_{false};

  bool control_active_{false};

  zmq::context_t zmq_ctx_;
  std::unique_ptr<zmq::socket_t> zmq_sock_;

  static constexpr int kZmqHeaderSize = 1280;
};

} // namespace g1_hardware
