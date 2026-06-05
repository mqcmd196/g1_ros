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

#include <rclcpp_components/register_node_macro.hpp>

namespace g1_hardware
{

// LocomotionMode enum values and their service name suffixes.
// Source: gear_sonic/scripts/pico_manager_thread_server.py
//         class LocomotionMode(IntEnum)
//
// Modes 0-3 (IDLE, SLOW_WALK, WALK, RUN) are NOT exposed as individual services.
// Instead ~/mode/default enables auto-selection based on cmd_vel magnitude:
//   mag < 0.05  → IDLE (0)
//   0.05-1.5    → WALK (2)  pico uses speed=-1 (auto)
//   mag >= 1.5  → RUN  (3)  pico minimum 1.5 m/s
// SLOW_WALK is skipped to avoid SLOW_WALK↔WALK oscillation at the boundary.
static const std::vector<std::pair<int, std::string>> kLocomotionModes = {
    {4, "idle_squat"},      {5, "idle_kneel_two_legs"},
    {6, "idle_kneel"},      {7, "idle_lying_face_down"},
    {8, "crawling"},        {9, "idle_boxing"},
    {10, "walk_boxing"},    {11, "left_punch"},
    {12, "right_punch"},    {13, "random_punch"},
    {14, "elbow_crawling"}, {15, "left_hook"},
    {16, "right_hook"},     {17, "forward_jump"},
    {18, "stealth_walk"},   {19, "injured_walk"},
};

static constexpr int kAutoMode = -1; // auto: IDLE/SLOW_WALK/WALK/RUN selected by cmd_vel mag

GearSonicInterface::GearSonicInterface(const rclcpp::NodeOptions& options)
: rclcpp::Node("gear_sonic_interface", options)
{
  const int zmq_port = declare_parameter("zmq_port", 5556);
  const double publish_rate = declare_parameter("publish_rate", 50.0);
  cmd_vel_timeout_ = declare_parameter("cmd_vel_timeout", 0.5);
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

  // ~/mode/default: auto-select IDLE/SLOW_WALK/WALK/RUN from cmd_vel magnitude
  mode_services_.push_back(create_service<std_srvs::srv::Trigger>(
      "~/mode/default", [this](std_srvs::srv::Trigger::Request::SharedPtr,
                               std_srvs::srv::Trigger::Response::SharedPtr res) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        locomotion_mode_ = kAutoMode;
        res->success = true;
        RCLCPP_INFO(get_logger(), "Locomotion mode → default (auto from cmd_vel)");
      }));

  // One Trigger service per non-locomotion mode: ~/mode/<name>
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

  RCLCPP_INFO(get_logger(), "Ready. Call ~/enable_control service true to start.");
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
    // Mirrors the pico_manager manager loop order
    // (gear_sonic/scripts/pico_manager_thread_server.py, manager main loop):
    //   run_once() sends a planner message FIRST, then the start command is sent AFTER.
    // This ensures the deploy stack's planner buffer is populated before it enters CONTROL,
    // preventing the policy from running with empty input and going to a bad pose.
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      heading_rad_ = 0.0;
      control_active_ = true;
    }
    // 1. Send one planner message (IDLE, zero velocity) so the deploy has data on entry
    const auto planner_msg = BuildPlannerMessage(
        /*mode=*/0, {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f},
        /*speed=*/-1.0f, /*height=*/-1.0f);
    zmq_sock_->send(zmq::buffer(planner_msg), zmq::send_flags::none);
    // 2. Send start command — deploy transitions WAIT_FOR_CONTROL → CONTROL
    const auto cmd_msg = BuildCommandMessage(/*start=*/true, /*stop=*/false, /*planner=*/true);
    zmq_sock_->send(zmq::buffer(cmd_msg), zmq::send_flags::none);
    RCLCPP_INFO(get_logger(), "enable_control(true): SONIC is in control. "
                              "Use ~/cmd_vel for locomotion, ~/mode/* for mode, "
                              "~/left_wrist + ~/right_wrist + ~/head for VR 3-point tracking.");
  } else {
    // Stop sending planner messages. Do NOT send command{stop=1}:
    // stop=1 → g1_deploy_onnx_ref.cpp G1DeployOnnxRef::Stop() → CreateDampingCommand()
    //   → kp=0, kd=8 for all joints → robot goes limp under gravity.
    // The deploy's planner timeout (~1 s) reverts locomotion to IDLE while keeping balance.
    std::lock_guard<std::mutex> lock(data_mutex_);
    control_active_ = false;
    RCLCPP_INFO(get_logger(), "enable_control(false): stopped. Robot reverts to IDLE after ~1 s.");
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
  cmd_angular_z_ = msg->angular.z;
  last_cmd_vel_time_ = now();
}

// ──────────────────────────────────────────────
// Pose callbacks
//
// vr_position layout: [Lw_x,y,z, Rw_x,y,z, H_x,y,z]  (9 floats)
// vr_orientation layout: [Lw_qw,qx,qy,qz, Rw_qw,qx,qy,qz, H_qw,qx,qy,qz]  (12 floats)
// Source: gear_sonic/scripts/pico_manager_thread_server.py
//         PoseStreamer._pose_stream_common()
//           "vr_position": vr_3pt_pose[:, :3].flatten()    # [Lw, Rw, Head] xyz
//           "vr_orientation": vr_3pt_pose[:, 3:].flatten()  # [Lw, Rw, Head] wxyz
//
// Position frame: G1 "pelvis" TF frame (URDF root link = SONIC root).
// local_pos = inv(root_quat) * (world_pos - root_pos)
// = tf2 transform of the target pose into the "pelvis" TF frame.
// Source: gear_sonic_deploy g1_deploy_onnx_ref.cpp GatherVR3PointPosition():
//   "The sender (ZMQ/ROS2/planner) has already applied offsets and root normalization"
// Also matches pico_manager ThreePointPose._process_smpl_pose():
//   kp_poses[i, :3] = Rot(root_quat).inv() * (world_pos - root_pos)
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
  // Source: gear_sonic/scripts/pico_manager_thread_server.py
  //         PlannerLoop.run_once()
  //
  //   facing = self.yaw_accumulator.update(rx, self.dt)
  //     → accumulates right-stick X into a 2-D unit vector [cos(θ), sin(θ)]
  //   perp_x, perp_y = -facing[1], facing[0]
  //   rotation_facing = [[perp_x, perp_y],
  //                       [facing[0], facing[1]]]
  //   movement_global = rotation_facing @ movement_local
  //     where movement_local = [-lx, ly] (joystick → body frame)
  //
  // We map cmd_vel body frame onto the same model:
  //   angular.z → integrates into heading_rad_ (replaces yaw_accumulator)
  //   linear.x/y → movement_local forward/strafe components

  // Zero velocity if no cmd_vel has arrived within the timeout window
  // (standard ROS differential-drive behaviour).
  if (last_cmd_vel_time_.nanoseconds() > 0 &&
      (now() - last_cmd_vel_time_).seconds() > cmd_vel_timeout_)
  {
    cmd_vel_x_ = 0.0;
    cmd_vel_y_ = 0.0;
    cmd_angular_z_ = 0.0;
  }

  // Integrate angular.z into heading each timer tick (mirrors YawAccumulator.update())
  heading_rad_ += cmd_angular_z_ * publish_dt_;

  const double cx = std::cos(heading_rad_);
  const double sy = std::sin(heading_rad_);
  const std::array<float, 3> facing = {static_cast<float>(cx), static_cast<float>(sy), 0.0f};
  // movement_global = rotation_facing @ [linear.y, linear.x]  (strafe, forward)
  // = [-sin(h)*linear.y + cos(h)*linear.x,
  //    cos(h)*linear.y  + sin(h)*linear.x]  (matches PlannerLoop.run_once() movement rotation)
  const std::array<float, 3> movement = {
      static_cast<float>(cmd_vel_x_ * cx - cmd_vel_y_ * sy), // world x
      static_cast<float>(cmd_vel_x_ * sy + cmd_vel_y_ * cx), // world y
      0.0f};

  // ── Speed field ───────────────────────────────────────────────────────
  // Source: gear_sonic/scripts/pico_manager_thread_server.py
  //         PlannerLoop.run_once()
  //
  // pico_manager uses normalised joystick magnitude mag ∈ [0, 1] as follows:
  //   SLOW_WALK (1): speed = 0.1 + 0.5 * mag   (0.1 ~ 0.6 m/s)
  //   WALK      (2): speed = -1.0               (SONIC auto — no explicit speed)
  //   RUN       (3): speed = 1.5 + 3 * mag      (1.5 ~ 4.5 m/s)
  //   others       : speed = mag                (0 ~ 1.0, normalised)
  //
  // cmd_vel.linear carries actual speeds in m/s (not normalised [0,1]).
  // Mapping:
  //   WALK:      keep speed = -1.0 to match pico_manager exactly.
  //   RUN:       pass m/s directly; enforce the 1.5 m/s minimum that the run gait
  //              requires — sending below this causes "drunk" locomotion.
  //   SLOW_WALK: pass m/s directly; clamp to pico's 0.1-0.6 m/s range.
  //   others:    pass m/s magnitude directly (pico's "speed = mag" maps [0,1] →
  //              units unknown, so m/s passthrough is the best approximation).
  //   zero input: speed = -1.0 (auto) for all modes.
  const float mag =
      std::sqrt(static_cast<float>(cmd_vel_x_ * cmd_vel_x_ + cmd_vel_y_ * cmd_vel_y_));

  // ── Mode and speed selection ───────────────────────────────────────────
  // Source: gear_sonic/scripts/pico_manager_thread_server.py PlannerLoop.run_once()
  int active_mode;
  float speed;
  if (locomotion_mode_ == kAutoMode) {
    // ~/mode/default: auto-select locomotion gait from cmd_vel magnitude.
    // Source: gear_sonic/scripts/pico_manager_thread_server.py PlannerLoop.run_once()
    //   < 0.05   → IDLE      (0)  deadzone
    //   0.05-0.6 → SLOW_WALK (1)  speed = clamp(mag, 0.1, 0.6)
    //   0.6-1.5  → WALK      (2)  speed = mag
    //   >= 1.5   → RUN       (3)  speed = clamp(mag, 1.5, 4.5)
    // RUN mode is not used in auto selection: speeds above ~2.0 m/s cause instability
    // in simulation (possibly a MuJoCo floor / policy issue). Cap WALK at 2.0 m/s.
    // RUN can still be triggered explicitly via a future ~/mode/run service if needed.
    if (mag < 0.05f) {
      active_mode = 0;
      speed = -1.0f; // IDLE
    } else if (mag < 0.6f) {
      active_mode = 1;
      speed = std::max(0.1f, mag); // SLOW_WALK
    } else {
      if (mag > 2.0f) {
        RCLCPP_WARN_ONCE(get_logger(),
                         "cmd_vel magnitude %.2f m/s exceeds 2.0 m/s cap "
                         "(high speeds cause instability in simulation; "
                         "clamped to 2.0 m/s). On real hardware, increase the cap via "
                         "scene friction or remove the limit.",
                         static_cast<double>(mag));
      }
      active_mode = 2;
      speed = std::min(mag, 2.0f); // WALK, capped at 2.0 m/s
    }
  } else {
    // Explicit non-locomotion mode set via ~/mode/* service (modes 4-19)
    active_mode = locomotion_mode_;
    speed = (mag < 1e-6f) ? -1.0f : mag;
  }

  // ── Build planner message ──────────────────────────────────────────────
  std::vector<uint8_t> msg;
  if (left_received_ && right_received_ && head_received_) {
    msg = BuildPlannerMessage(active_mode, movement, facing, speed, /*height=*/-1.0f, &vr_position_,
                              &vr_orientation_);
  } else {
    msg = BuildPlannerMessage(active_mode, movement, facing, speed, /*height=*/-1.0f);
  }
  zmq_sock_->send(zmq::buffer(msg), zmq::send_flags::dontwait);
}

// ──────────────────────────────────────────────
// ZMQ message builders
// Replicates gear_sonic/utils/teleop/zmq/zmq_planner_sender.py
//   build_command_message()
//   build_planner_message()
//   _build_header()   (HEADER_SIZE = 1280)
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
