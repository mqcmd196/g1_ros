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

#include "g1_hardware/gear_sonic_interface.hpp"

#include <chrono>
#include <cstring>
#include <thread>

#include <rclcpp_components/register_node_macro.hpp>

namespace g1_hardware
{

// LocomotionMode enum values and their service name suffixes.
// Source: gear_sonic/scripts/pico_manager_thread_server.py LocomotionMode
static const std::vector<std::pair<int, std::string>> kLocomotionModes = {
    {0, "idle"},
    {1, "slow_walk"},
    {2, "walk"},
    {3, "run"},
    {4, "idle_squat"},
    {5, "idle_kneel_two_legs"},
    {6, "idle_kneel"},
    {7, "idle_lying_face_down"},
    {8, "crawling"},
    {9, "idle_boxing"},
    {10, "walk_boxing"},
    {11, "left_punch"},
    {12, "right_punch"},
    {13, "random_punch"},
    {14, "elbow_crawling"},
    {15, "left_hook"},
    {16, "right_hook"},
    {17, "forward_jump"},
    {18, "stealth_walk"},
    {19, "injured_walk"},
};

GearSonicInterface::GearSonicInterface(const rclcpp::NodeOptions& options)
: rclcpp::Node("gear_sonic_interface", options)
{
  const int zmq_port = declare_parameter("zmq_port", 5556);
  const double publish_rate = declare_parameter("publish_rate", 50.0);
  publish_dt_ = 1.0 / publish_rate;

  // ZMQ: bind PUB socket so the deploy stack SUB can connect to us
  zmq_sock_ = std::make_unique<zmq::socket_t>(zmq_ctx_, zmq::socket_type::pub);
  const std::string addr = "tcp://0.0.0.0:" + std::to_string(zmq_port);
  zmq_sock_->bind(addr);
  RCLCPP_INFO(get_logger(), "Bound ZMQ PUB to %s", addr.c_str());

  // ── Services ────────────────────────────────────────────────────────────

  enable_control_srv_ = create_service<std_srvs::srv::SetBool>(
      "~/enable_control",
      [this](std_srvs::srv::SetBool::Request::SharedPtr req,
             std_srvs::srv::SetBool::Response::SharedPtr res) { OnEnableControl(req, res); });

  start_balance_srv_ = create_service<std_srvs::srv::SetBool>(
      "~/start_balance",
      [this](std_srvs::srv::SetBool::Request::SharedPtr req,
             std_srvs::srv::SetBool::Response::SharedPtr res) { OnStartBalance(req, res); });

  // One Trigger service per locomotion mode: ~/mode/<name>
  for (const auto& [mode_val, mode_name] : kLocomotionModes) {
    auto srv = create_service<std_srvs::srv::Trigger>(
        "~/mode/" + mode_name, [this, mode_val](std_srvs::srv::Trigger::Request::SharedPtr,
                                                std_srvs::srv::Trigger::Response::SharedPtr res) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          locomotion_mode_ = mode_val;
          res->success = true;
          RCLCPP_INFO(get_logger(), "Locomotion mode → %d", mode_val);
        });
    mode_services_.push_back(srv);
  }

  // ── Subscriptions ────────────────────────────────────────────────────────

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "~/cmd_vel", 1, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) { OnCmdVel(msg); });

  left_wrist_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/left_wrist", 1,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { OnLeftWrist(msg); });
  right_wrist_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/right_wrist", 1,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { OnRightWrist(msg); });
  head_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/head", 1, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { OnHead(msg); });

  // Timer is always spinning; does nothing until control_active_ is set
  const auto period = std::chrono::duration<double>(publish_dt_);
  timer_ = create_wall_timer(period, [this]() { TimerCallback(); });

  RCLCPP_INFO(get_logger(), "Ready. Call ~/enable_control then ~/start_balance to begin.");
}

GearSonicInterface::~GearSonicInterface()
{
  if (zmq_sock_) {
    const auto stop_msg = BuildCommandMessage(/*start=*/false, /*stop=*/true, /*planner=*/false);
    zmq_sock_->send(zmq::buffer(stop_msg), zmq::send_flags::none);
    zmq_sock_->close();
  }
}

// ──────────────────────────────────────────────
// Services
// ──────────────────────────────────────────────

void GearSonicInterface::OnEnableControl(std_srvs::srv::SetBool::Request::SharedPtr req,
                                         std_srvs::srv::SetBool::Response::SharedPtr res)
{
  if (req->data) {
    // Equivalent to PICO A+B+X+Y: deploy transitions WAIT_FOR_CONTROL → CONTROL
    const auto msg = BuildCommandMessage(/*start=*/true, /*stop=*/false, /*planner=*/true);
    zmq_sock_->send(zmq::buffer(msg), zmq::send_flags::none);
    RCLCPP_INFO(get_logger(), "enable_control(true): sent start command.");
  } else {
    std::lock_guard<std::mutex> lock(data_mutex_);
    control_active_ = false;
    const auto msg = BuildCommandMessage(/*start=*/false, /*stop=*/true, /*planner=*/false);
    zmq_sock_->send(zmq::buffer(msg), zmq::send_flags::none);
    RCLCPP_INFO(get_logger(), "enable_control(false): sent stop command.");
  }
  res->success = true;
}

void GearSonicInterface::OnStartBalance(std_srvs::srv::SetBool::Request::SharedPtr req,
                                        std_srvs::srv::SetBool::Response::SharedPtr res)
{
  if (req->data) {
    // Equivalent to PICO A+X entering PLANNER_VR_3PT:
    //   - send command{start=1, planner=1}
    //   - begin sending bare planner messages (no upper_body_position, matching
    //     pico_manager run_once() where upper_body_position=None in VR_3PT mode)
    //   - SONIC autonomously holds the natural standing pose until vr_position arrives
    const auto cmd_msg = BuildCommandMessage(/*start=*/true, /*stop=*/false, /*planner=*/true);
    zmq_sock_->send(zmq::buffer(cmd_msg), zmq::send_flags::none);
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      control_active_ = true;
      heading_rad_ = 0.0; // reset heading on balance start
    }
    RCLCPP_INFO(get_logger(), "start_balance(true): SONIC is in control. "
                              "Use ~/cmd_vel for locomotion, ~/mode/* for mode, "
                              "~/left_wrist + ~/right_wrist + ~/head for VR 3-point tracking.");
  } else {
    std::lock_guard<std::mutex> lock(data_mutex_);
    control_active_ = false;
    RCLCPP_INFO(get_logger(), "start_balance(false): stopped sending planner messages.");
  }
  res->success = true;
}

// ──────────────────────────────────────────────
// Subscription callbacks
// ──────────────────────────────────────────────

void GearSonicInterface::OnCmdVel(geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  cmd_vel_x_ = msg->linear.x;
  cmd_vel_y_ = msg->linear.y;
  cmd_angular_z_ = msg->angular.z; // integrated into heading_rad_ each timer tick
}

// ──────────────────────────────────────────────
// Pose callbacks
// Position frame: G1 "pelvis" TF frame (URDF root link = SONIC root).
// local_pos = inv(root_quat) * (world_pos - root_pos)
// = tf2 transform of the target pose into the "pelvis" TF frame.
// Reference: g1_deploy_onnx_ref.cpp GatherVR3PointPosition() line 1114:
//   "The sender (ZMQ/ROS2/planner) has already applied offsets and root normalization"
//
// Quaternion order:
//   geometry_msgs: (x, y, z, w)
//   SONIC vr_orientation: (w, x, y, z) ← reordered in each callback below
// ──────────────────────────────────────────────

void GearSonicInterface::OnLeftWrist(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  vr_position_[0] = static_cast<float>(msg->pose.position.x);
  vr_position_[1] = static_cast<float>(msg->pose.position.y);
  vr_position_[2] = static_cast<float>(msg->pose.position.z);
  vr_orientation_[0] = static_cast<float>(msg->pose.orientation.w);
  vr_orientation_[1] = static_cast<float>(msg->pose.orientation.x);
  vr_orientation_[2] = static_cast<float>(msg->pose.orientation.y);
  vr_orientation_[3] = static_cast<float>(msg->pose.orientation.z);
  left_received_ = true;
}

void GearSonicInterface::OnRightWrist(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  vr_position_[3] = static_cast<float>(msg->pose.position.x);
  vr_position_[4] = static_cast<float>(msg->pose.position.y);
  vr_position_[5] = static_cast<float>(msg->pose.position.z);
  vr_orientation_[4] = static_cast<float>(msg->pose.orientation.w);
  vr_orientation_[5] = static_cast<float>(msg->pose.orientation.x);
  vr_orientation_[6] = static_cast<float>(msg->pose.orientation.y);
  vr_orientation_[7] = static_cast<float>(msg->pose.orientation.z);
  right_received_ = true;
}

void GearSonicInterface::OnHead(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  vr_position_[6] = static_cast<float>(msg->pose.position.x);
  vr_position_[7] = static_cast<float>(msg->pose.position.y);
  vr_position_[8] = static_cast<float>(msg->pose.position.z);
  vr_orientation_[8] = static_cast<float>(msg->pose.orientation.w);
  vr_orientation_[9] = static_cast<float>(msg->pose.orientation.x);
  vr_orientation_[10] = static_cast<float>(msg->pose.orientation.y);
  vr_orientation_[11] = static_cast<float>(msg->pose.orientation.z);
  head_received_ = true;
}

void GearSonicInterface::TimerCallback()
{
  if (!control_active_) {
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);

  // ── Locomotion: convert cmd_vel (body frame) to movement/facing (world frame) ──
  //
  // SONIC planner expects:
  //   movement[0,1]: velocity direction in world/global frame
  //   facing[0,1]:   unit vector of the robot's desired facing direction
  //
  // cmd_vel uses robot body frame:
  //   linear.x  = forward  (along facing direction)
  //   linear.y  = strafe   (perpendicular to facing, positive = left)
  //   angular.z = turn rate (rad/s) → integrated into heading_rad_ each tick
  //
  // Mirrors pico_manager run_once() heading accumulation (yaw_accumulator) and
  // movement rotation:
  //   facing = [cos(heading), sin(heading)]
  //   perp   = [-sin(heading), cos(heading)]  (left perpendicular)
  //   movement_global = linear.x * facing + linear.y * perp

  // Integrate angular.z into heading each timer tick
  heading_rad_ += cmd_angular_z_ * publish_dt_;

  const double cx = std::cos(heading_rad_);
  const double sy = std::sin(heading_rad_);
  // facing direction and its left perpendicular
  const std::array<float, 3> facing = {static_cast<float>(cx), static_cast<float>(sy), 0.0f};
  const std::array<float, 3> movement = {
      static_cast<float>(cmd_vel_x_ * cx - cmd_vel_y_ * sy), // world x
      static_cast<float>(cmd_vel_x_ * sy + cmd_vel_y_ * cx), // world y
      0.0f};

  // ── Build planner message ──────────────────────────────────────────────
  std::vector<uint8_t> msg;
  if (left_received_ && right_received_ && head_received_) {
    // VR 3-point tracking active: locomotion + upper-body targets together
    // This is the key feature: locomotion and arm control run simultaneously,
    // unlike the PICO UI which disables one while the other is active.
    msg = BuildPlannerMessage(locomotion_mode_, movement, facing,
                              /*speed=*/-1.0f, /*height=*/-1.0f, &vr_position_, &vr_orientation_);
  } else {
    // VR data not yet available: locomotion-only (mirrors PLANNER_VR_3PT with sample==None)
    msg = BuildPlannerMessage(locomotion_mode_, movement, facing,
                              /*speed=*/-1.0f, /*height=*/-1.0f);
  }
  zmq_sock_->send(zmq::buffer(msg), zmq::send_flags::dontwait);
}

// ──────────────────────────────────────────────
// ZMQ message builders
// Replicates gear_sonic/utils/teleop/zmq/zmq_planner_sender.py
// Message layout: [topic_bytes][1280-byte JSON header][little-endian binary payload]
// ──────────────────────────────────────────────

std::vector<uint8_t> GearSonicInterface::BuildZmqHeader(const std::string& fields_json, int version)
{
  const std::string s = "{\"v\":" + std::to_string(version) +
                        ",\"endian\":\"le\",\"count\":1,\"fields\":" + fields_json + "}";
  std::vector<uint8_t> header(kZmqHeaderSize, 0);
  std::memcpy(header.data(), s.data(), std::min(s.size(), static_cast<size_t>(kZmqHeaderSize)));
  return header;
}

std::vector<uint8_t> GearSonicInterface::BuildCommandMessage(bool start, bool stop, bool planner)
{
  const std::string fields = "[{\"name\":\"start\",\"dtype\":\"u8\",\"shape\":[1]},"
                             "{\"name\":\"stop\",\"dtype\":\"u8\",\"shape\":[1]},"
                             "{\"name\":\"planner\",\"dtype\":\"u8\",\"shape\":[1]}]";
  std::vector<uint8_t> msg;
  const std::string topic = "command";
  msg.insert(msg.end(), topic.begin(), topic.end());
  const auto hdr = BuildZmqHeader(fields);
  msg.insert(msg.end(), hdr.begin(), hdr.end());
  msg.push_back(start ? 1u : 0u);
  msg.push_back(stop ? 1u : 0u);
  msg.push_back(planner ? 1u : 0u);
  return msg;
}

std::vector<uint8_t> GearSonicInterface::BuildPlannerMessage(
    int mode, std::array<float, 3> movement, std::array<float, 3> facing, float speed, float height,
    const std::array<float, 9>* vr_position, const std::array<float, 12>* vr_orientation)
{
  std::string fields = "[{\"name\":\"mode\",\"dtype\":\"i32\",\"shape\":[1]},"
                       "{\"name\":\"movement\",\"dtype\":\"f32\",\"shape\":[3]},"
                       "{\"name\":\"facing\",\"dtype\":\"f32\",\"shape\":[3]},"
                       "{\"name\":\"speed\",\"dtype\":\"f32\",\"shape\":[1]},"
                       "{\"name\":\"height\",\"dtype\":\"f32\",\"shape\":[1]}";
  if (vr_position) {
    fields += ",{\"name\":\"vr_position\",\"dtype\":\"f32\",\"shape\":[9]}";
  }
  if (vr_orientation) {
    fields += ",{\"name\":\"vr_orientation\",\"dtype\":\"f32\",\"shape\":[12]}";
  }
  fields += "]";

  auto append = [](std::vector<uint8_t>& v, const void* data, size_t n) {
    const auto* p = static_cast<const uint8_t*>(data);
    v.insert(v.end(), p, p + n);
  };

  std::vector<uint8_t> msg;
  const std::string topic = "planner";
  msg.insert(msg.end(), topic.begin(), topic.end());
  const auto hdr = BuildZmqHeader(fields);
  msg.insert(msg.end(), hdr.begin(), hdr.end());

  // All values are little-endian; x86 is natively LE so memcpy is correct
  append(msg, &mode, 4);
  for (float v : movement)
    append(msg, &v, 4);
  for (float v : facing)
    append(msg, &v, 4);
  append(msg, &speed, 4);
  append(msg, &height, 4);
  if (vr_position) {
    for (float v : *vr_position)
      append(msg, &v, 4);
  }
  if (vr_orientation) {
    for (float v : *vr_orientation)
      append(msg, &v, 4);
  }
  return msg;
}

} // namespace g1_hardware

RCLCPP_COMPONENTS_REGISTER_NODE(g1_hardware::GearSonicInterface)
