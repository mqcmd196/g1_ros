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

#include "g1_hardware/inspire_rh56dfx_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <thread>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unordered_map>

namespace g1_hardware
{
namespace
{

constexpr size_t kMotorCount = 12;

using JointConfig = InspireRH56DFXHardwareInterface::JointConfig;

// The Inspire DDS bridge uses the normalized position convention from
// hand_example.cpp: 0.0 = closed, 1.0 = open.
// The table stores ROS joint positions for open and closed hand states.
const std::unordered_map<std::string, JointConfig> kJointNameToConfig = {
    {"R_pinky_proximal_joint", {0, 0.0, 1.7}},
    {"R_ring_proximal_joint", {1, 0.0, 1.7}},
    {"R_middle_proximal_joint", {2, 0.0, 1.7}},
    {"R_index_proximal_joint", {3, 0.0, 1.7}},
    {"R_thumb_proximal_pitch_joint", {4, 0.0, 0.6}},
    {"R_thumb_proximal_yaw_joint", {5, 0.0, 1.3}},
    {"L_pinky_proximal_joint", {6, 0.0, 1.7}},
    {"L_ring_proximal_joint", {7, 0.0, 1.7}},
    {"L_middle_proximal_joint", {8, 0.0, 1.7}},
    {"L_index_proximal_joint", {9, 0.0, 1.7}},
    {"L_thumb_proximal_pitch_joint", {10, 0.0, 0.6}},
    {"L_thumb_proximal_yaw_joint", {11, 0.0, 1.3}},
};

double clamp01(double value)
{
  if (!std::isfinite(value)) {
    return 0.0;
  }
  return std::min(1.0, std::max(0.0, value));
}

double open_fraction_to_joint_position(double open_fraction, const JointConfig& cfg)
{
  const double normalized_closed = 1.0 - clamp01(open_fraction);
  return cfg.open_position + normalized_closed * (cfg.closed_position - cfg.open_position);
}

double joint_position_to_open_fraction(double position, const JointConfig& cfg)
{
  const double normalized_closed =
      (position - cfg.open_position) / (cfg.closed_position - cfg.open_position);
  return 1.0 - clamp01(normalized_closed);
}

} // namespace

hardware_interface::CallbackReturn
InspireRH56DFXHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  if (info.hardware_parameters.count("network_interface")) {
    network_interface_ = info.hardware_parameters.at("network_interface");
  }
  if (info.hardware_parameters.count("command_topic")) {
    command_topic_ = info.hardware_parameters.at("command_topic");
  }
  if (info.hardware_parameters.count("state_topic")) {
    state_topic_ = info.hardware_parameters.at("state_topic");
  }
  if (info.hardware_parameters.count("state_timeout_sec")) {
    try {
      state_timeout_sec_ = std::stod(info.hardware_parameters.at("state_timeout_sec"));
    } catch (const std::exception& e) {
      RCLCPP_FATAL(rclcpp::get_logger("InspireRH56DFXHardwareInterface"),
                   "Invalid state_timeout_sec parameter '%s': %s",
                   info.hardware_parameters.at("state_timeout_sec").c_str(), e.what());
      return CallbackReturn::ERROR;
    }
  }

  const size_t n = info_.joints.size();
  joint_configs_.resize(n);
  hw_positions_.resize(n, 0.0);
  hw_velocities_.resize(n, 0.0);
  hw_commands_.resize(n, 0.0);

  for (size_t i = 0; i < n; ++i) {
    const std::string& name = info_.joints[i].name;
    auto it = kJointNameToConfig.find(name);
    if (it == kJointNameToConfig.end()) {
      RCLCPP_FATAL(rclcpp::get_logger("InspireRH56DFXHardwareInterface"),
                   "Unknown RH56DFX joint '%s' - no DDS motor mapping defined.", name.c_str());
      return CallbackReturn::ERROR;
    }
    joint_configs_[i] = it->second;
    hw_positions_[i] = it->second.open_position;
    hw_commands_[i] = hw_positions_[i];

    RCLCPP_INFO(rclcpp::get_logger("InspireRH56DFXHardwareInterface"),
                "Joint '%s' -> Inspire motor %zu, open %.3f, closed %.3f", name.c_str(),
                it->second.motor_index, it->second.open_position, it->second.closed_position);
  }

  RCLCPP_INFO(rclcpp::get_logger("InspireRH56DFXHardwareInterface"),
              "Initialized: command_topic=%s, state_topic=%s, network_interface=%s",
              command_topic_.c_str(), state_topic_.c_str(), network_interface_.c_str());

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
InspireRH56DFXHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    si.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    si.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return si;
}

std::vector<hardware_interface::CommandInterface>
InspireRH56DFXHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    ci.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }
  return ci;
}

hardware_interface::CallbackReturn
InspireRH56DFXHardwareInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("InspireRH56DFXHardwareInterface"),
              "Activating RH56DFX hand interface.");

  unitree::robot::ChannelFactory::Instance()->Init(0, network_interface_);

  state_received_ = false;
  command_msg_.cmds().resize(kMotorCount);
  state_msg_.states().resize(kMotorCount);

  hand_command_publisher_.reset(
      new unitree::robot::ChannelPublisher<unitree_go::msg::dds_::MotorCmds_>(command_topic_));
  hand_command_publisher_->InitChannel();

  hand_state_subscriber_.reset(
      new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::MotorStates_>(state_topic_));
  hand_state_subscriber_->InitChannel(
      [this](const void* msg) {
        auto s = reinterpret_cast<const unitree_go::msg::dds_::MotorStates_*>(msg);
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_msg_ = *s;
        state_received_ = true;
      },
      1);

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                            std::chrono::duration<double>(state_timeout_sec_));
  while (!state_received_ && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  if (!state_received_) {
    RCLCPP_FATAL(rclcpp::get_logger("InspireRH56DFXHardwareInterface"),
                 "No hand state received on '%s' within %.1f s. Is inspire_g1 running?",
                 state_topic_.c_str(), state_timeout_sec_);
    hand_command_publisher_.reset();
    hand_state_subscriber_.reset();
    return CallbackReturn::ERROR;
  }

  size_t received_motor_count = 0;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    received_motor_count = state_msg_.states().size();
    if (received_motor_count >= kMotorCount) {
      for (size_t i = 0; i < info_.joints.size(); ++i) {
        const auto& cfg = joint_configs_[i];
        const double open_fraction = state_msg_.states().at(cfg.motor_index).q();
        hw_positions_[i] = open_fraction_to_joint_position(open_fraction, cfg);
        hw_velocities_[i] = 0.0;
        hw_commands_[i] = hw_positions_[i];
      }
    }
  }

  if (received_motor_count < kMotorCount) {
    RCLCPP_FATAL(rclcpp::get_logger("InspireRH56DFXHardwareInterface"),
                 "Hand state on '%s' has %zu motors, expected at least %zu.", state_topic_.c_str(),
                 received_motor_count, kMotorCount);
    hand_command_publisher_.reset();
    hand_state_subscriber_.reset();
    return CallbackReturn::ERROR;
  }

  active_command_interfaces_ = 0;
  RCLCPP_INFO(rclcpp::get_logger("InspireRH56DFXHardwareInterface"), "Activated.");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
InspireRH56DFXHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  hand_command_publisher_.reset();
  hand_state_subscriber_.reset();
  active_command_interfaces_ = 0;
  RCLCPP_INFO(rclcpp::get_logger("InspireRH56DFXHardwareInterface"), "Deactivated.");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type InspireRH56DFXHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces)
{
  auto is_own_command_interface = [this](const std::string& interface_name) {
    const auto separator = interface_name.find('/');
    const std::string joint_name =
        separator == std::string::npos ? interface_name : interface_name.substr(0, separator);
    return std::any_of(info_.joints.begin(), info_.joints.end(), [&joint_name](const auto& joint) {
      return joint.name == joint_name && !joint.command_interfaces.empty();
    });
  };

  const auto started = static_cast<int>(
      std::count_if(start_interfaces.begin(), start_interfaces.end(), is_own_command_interface));
  const auto stopped = static_cast<int>(
      std::count_if(stop_interfaces.begin(), stop_interfaces.end(), is_own_command_interface));

  active_command_interfaces_ += started;
  active_command_interfaces_ -= stopped;
  active_command_interfaces_ = std::max(0, active_command_interfaces_);

  RCLCPP_INFO(rclcpp::get_logger("InspireRH56DFXHardwareInterface"),
              "Command mode switch: %d started, %d stopped -> %d active.", started, stopped,
              active_command_interfaces_);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
InspireRH56DFXHardwareInterface::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
  if (!state_received_) {
    return hardware_interface::return_type::OK;
  }

  unitree_go::msg::dds_::MotorStates_ state;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state = state_msg_;
  }

  if (state.states().size() < kMotorCount) {
    RCLCPP_ERROR(rclcpp::get_logger("InspireRH56DFXHardwareInterface"),
                 "Hand state on '%s' has %zu motors, expected at least %zu.", state_topic_.c_str(),
                 state.states().size(), kMotorCount);
    return hardware_interface::return_type::ERROR;
  }

  const double dt = period.seconds();
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const double previous_position = hw_positions_[i];
    const auto& cfg = joint_configs_[i];
    const double open_fraction = state.states().at(cfg.motor_index).q();
    hw_positions_[i] = open_fraction_to_joint_position(open_fraction, cfg);
    hw_velocities_[i] = dt > 1e-9 ? (hw_positions_[i] - previous_position) / dt : 0.0;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
InspireRH56DFXHardwareInterface::write(const rclcpp::Time& /*time*/,
                                       const rclcpp::Duration& /*period*/)
{
  if (!hand_command_publisher_ || active_command_interfaces_ == 0) {
    return hardware_interface::return_type::OK;
  }

  command_msg_.cmds().resize(kMotorCount);
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto& cfg = joint_configs_[i];
    command_msg_.cmds()
        .at(cfg.motor_index)
        .q(static_cast<float>(joint_position_to_open_fraction(hw_commands_[i], cfg)));
  }

  hand_command_publisher_->Write(command_msg_);
  return hardware_interface::return_type::OK;
}

} // namespace g1_hardware

PLUGINLIB_EXPORT_CLASS(g1_hardware::InspireRH56DFXHardwareInterface,
                       hardware_interface::SystemInterface)
