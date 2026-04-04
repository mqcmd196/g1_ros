#include "g1_hardware/g1_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <thread>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <unitree/robot/channel/channel_factory.hpp>

namespace g1_hardware
{

hardware_interface::CallbackReturn G1HardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // -- Parameters from <hardware><param> in the URDF ros2_control tag --
  try {
    network_interface_ = info.hardware_parameters.at("network_interface");
  } catch (const std::out_of_range &) {
    RCLCPP_FATAL(rclcpp::get_logger("G1HardwareInterface"),
      "Missing required parameter 'network_interface' in URDF hardware tag.");
    return CallbackReturn::ERROR;
  }

  if (info.hardware_parameters.count("kp")) {
    kp_ = std::stof(info.hardware_parameters.at("kp"));
  }
  if (info.hardware_parameters.count("kd")) {
    kd_ = std::stof(info.hardware_parameters.at("kd"));
  }
  if (info.hardware_parameters.count("waist_kp")) {
    waist_kp_ = std::stof(info.hardware_parameters.at("waist_kp"));
  }
  if (info.hardware_parameters.count("waist_kd")) {
    waist_kd_ = std::stof(info.hardware_parameters.at("waist_kd"));
  }
  if (info.hardware_parameters.count("weight_rate")) {
    weight_rate_ = std::stof(info.hardware_parameters.at("weight_rate"));
  }

  // -- Map joint names to SDK indices --
  const size_t n = info_.joints.size();
  sdk_indices_.resize(n, -1);
  hw_positions_.resize(n, 0.0);
  hw_velocities_.resize(n, 0.0);
  hw_commands_.resize(n, 0.0);

  for (size_t i = 0; i < n; ++i) {
    const std::string & name = info_.joints[i].name;
    auto it = kJointNameToSdkIndex.find(name);
    if (it == kJointNameToSdkIndex.end()) {
      RCLCPP_FATAL(rclcpp::get_logger("G1HardwareInterface"),
        "Unknown joint '%s' — no SDK index mapping defined.", name.c_str());
      return CallbackReturn::ERROR;
    }
    sdk_indices_[i] = it->second;
    RCLCPP_INFO(rclcpp::get_logger("G1HardwareInterface"),
      "Joint '%s' → SDK index %d", name.c_str(), sdk_indices_[i]);
  }

  RCLCPP_INFO(rclcpp::get_logger("G1HardwareInterface"),
    "Initialized: network_interface=%s, kp=%.1f, kd=%.2f, waist_kp=%.1f, waist_kd=%.2f",
    network_interface_.c_str(), kp_, kd_, waist_kp_, waist_kd_);
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
G1HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    si.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION,  &hw_positions_[i]);
    si.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,  &hw_velocities_[i]);
  }
  return si;
}

std::vector<hardware_interface::CommandInterface>
G1HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    if (info_.joints[i].command_interfaces.empty()) {
      continue;  // state-only joints (legs, waist) — no command interface
    }
    ci.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }
  return ci;
}

hardware_interface::CallbackReturn G1HardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("G1HardwareInterface"),
    "Activating... connecting to robot on interface '%s'", network_interface_.c_str());

  // Initialize Unitree DDS (singleton — safe to call once)
  unitree::robot::ChannelFactory::Instance()->Init(0, network_interface_);

  // Subscribe to robot state
  state_received_ = false;
  low_state_subscriber_.reset(
    new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(kTopicLowState));
  low_state_subscriber_->InitChannel([this](const void * msg) {
    auto s = reinterpret_cast<const unitree_hg::msg::dds_::LowState_ *>(msg);
    std::lock_guard<std::mutex> lock(state_mutex_);
    memcpy(&state_msg_, s, sizeof(unitree_hg::msg::dds_::LowState_));
    state_received_ = true;
  }, 1);

  // Wait up to 3 s for the first state message
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!state_received_ && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (!state_received_) {
    RCLCPP_FATAL(rclcpp::get_logger("G1HardwareInterface"),
      "No state received from robot within 3 s. Is the robot connected?");
    return CallbackReturn::ERROR;
  }

  // Seed commands from current hardware state so the robot doesn't jump
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      hw_positions_[i] = state_msg_.motor_state().at(sdk_indices_[i]).q();
      hw_velocities_[i] = state_msg_.motor_state().at(sdk_indices_[i]).dq();
      hw_commands_[i] = hw_positions_[i];  // hold current pose
    }
  }

  // Create publisher
  arm_sdk_publisher_.reset(
    new unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(kTopicArmSdk));
  arm_sdk_publisher_->InitChannel();

  // Start with weight = 0; ramp up only when controllers become active
  weight_ = 0.0f;
  deactivating_ = false;
  active_command_interfaces_ = 0;

  RCLCPP_INFO(rclcpp::get_logger("G1HardwareInterface"),
    "Activated. Weight will ramp up to 1.0 over %.1f s.",
    1.0f / weight_rate_);
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn G1HardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("G1HardwareInterface"),
    "Deactivating... ramping down control weight.");
  deactivating_ = true;

  // Blocking ramp-down: keep sending last commanded positions while weight → 0
  constexpr float dt = 0.02f;
  const float delta = weight_rate_ * dt;
  while (weight_ > 0.0f) {
    weight_ = std::max(0.0f, weight_ - delta);

    unitree_hg::msg::dds_::LowCmd_ cmd{};
    cmd.motor_cmd().at(kWeightJoint).q(weight_);
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      if (info_.joints[i].command_interfaces.empty()) continue;
      auto & mc = cmd.motor_cmd().at(sdk_indices_[i]);
      mc.q(static_cast<float>(hw_commands_[i]));
      mc.dq(0.0f);
      const bool is_waist = (sdk_indices_[i] == kWaistYaw ||
                             sdk_indices_[i] == kWaistRoll ||
                             sdk_indices_[i] == kWaistPitch);
      mc.kp(is_waist ? waist_kp_ : kp_);
      mc.kd(is_waist ? waist_kd_ : kd_);
      mc.tau(0.0f);
    }
    arm_sdk_publisher_->Write(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
  }

  arm_sdk_publisher_.reset();
  low_state_subscriber_.reset();
  RCLCPP_INFO(rclcpp::get_logger("G1HardwareInterface"), "Deactivated.");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type G1HardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!state_received_) {
    return hardware_interface::return_type::OK;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    hw_positions_[i]  = state_msg_.motor_state().at(sdk_indices_[i]).q();
    hw_velocities_[i] = state_msg_.motor_state().at(sdk_indices_[i]).dq();
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type G1HardwareInterface::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  active_command_interfaces_ += static_cast<int>(start_interfaces.size());
  active_command_interfaces_ -= static_cast<int>(stop_interfaces.size());
  active_command_interfaces_ = std::max(0, active_command_interfaces_);

  RCLCPP_INFO(rclcpp::get_logger("G1HardwareInterface"),
    "Command mode switch: %zu started, %zu stopped → %d active. Weight will %s.",
    start_interfaces.size(), stop_interfaces.size(), active_command_interfaces_,
    active_command_interfaces_ > 0 ? "ramp UP" : "ramp DOWN");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type G1HardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!arm_sdk_publisher_ || deactivating_) {
    return hardware_interface::return_type::OK;
  }

  // Ramp weight up when controllers are active, down when all are inactive
  const float dt = static_cast<float>(period.seconds());
  if (active_command_interfaces_ > 0) {
    weight_ = std::min(1.0f, weight_ + weight_rate_ * dt);
  } else {
    weight_ = std::max(0.0f, weight_ - weight_rate_ * dt);
  }

  unitree_hg::msg::dds_::LowCmd_ cmd{};
  cmd.motor_cmd().at(kWeightJoint).q(weight_);

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    if (info_.joints[i].command_interfaces.empty()) {
      continue;  // state-only joints (legs, waist) — skip
    }
    auto & mc = cmd.motor_cmd().at(sdk_indices_[i]);
    mc.q(static_cast<float>(hw_commands_[i]));
    mc.dq(0.0f);
    const bool is_waist = (sdk_indices_[i] == kWaistYaw ||
                           sdk_indices_[i] == kWaistRoll ||
                           sdk_indices_[i] == kWaistPitch);
    mc.kp(is_waist ? waist_kp_ : kp_);
    mc.kd(is_waist ? waist_kd_ : kd_);
    mc.tau(0.0f);
  }

  arm_sdk_publisher_->Write(cmd);
  return hardware_interface::return_type::OK;
}

}  // namespace g1_hardware

PLUGINLIB_EXPORT_CLASS(g1_hardware::G1HardwareInterface, hardware_interface::SystemInterface)
