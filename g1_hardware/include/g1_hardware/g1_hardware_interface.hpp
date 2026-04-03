#pragma once

#include <array>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace g1_hardware
{

// SDK joint indices for the G1 29DoF (legs, waist, arms)
// Legs
static constexpr int kLeftHipPitch       =  0;
static constexpr int kLeftHipRoll        =  1;
static constexpr int kLeftHipYaw         =  2;
static constexpr int kLeftKnee           =  3;
static constexpr int kLeftAnklePitch     =  4;
static constexpr int kLeftAnkleRoll      =  5;
static constexpr int kRightHipPitch      =  6;
static constexpr int kRightHipRoll       =  7;
static constexpr int kRightHipYaw        =  8;
static constexpr int kRightKnee          =  9;
static constexpr int kRightAnklePitch    = 10;
static constexpr int kRightAnkleRoll     = 11;
// Waist
static constexpr int kWaistYaw           = 12;
static constexpr int kWaistRoll          = 13;
static constexpr int kWaistPitch         = 14;
// Arms
static constexpr int kLeftShoulderPitch  = 15;
static constexpr int kLeftShoulderRoll   = 16;
static constexpr int kLeftShoulderYaw    = 17;
static constexpr int kLeftElbow          = 18;
static constexpr int kLeftWristRoll      = 19;
static constexpr int kLeftWristPitch     = 20;
static constexpr int kLeftWristYaw       = 21;
static constexpr int kRightShoulderPitch = 22;
static constexpr int kRightShoulderRoll  = 23;
static constexpr int kRightShoulderYaw   = 24;
static constexpr int kRightElbow         = 25;
static constexpr int kRightWristRoll     = 26;
static constexpr int kRightWristPitch    = 27;
static constexpr int kRightWristYaw      = 28;
// Weight register: writing to this joint's q field sets the SDK control weight (0→1)
static constexpr int kWeightJoint        = 29;

static const std::unordered_map<std::string, int> kJointNameToSdkIndex = {
  // Legs (state-only)
  {"left_hip_pitch_joint",       kLeftHipPitch},
  {"left_hip_roll_joint",        kLeftHipRoll},
  {"left_hip_yaw_joint",         kLeftHipYaw},
  {"left_knee_joint",            kLeftKnee},
  {"left_ankle_pitch_joint",     kLeftAnklePitch},
  {"left_ankle_roll_joint",      kLeftAnkleRoll},
  {"right_hip_pitch_joint",      kRightHipPitch},
  {"right_hip_roll_joint",       kRightHipRoll},
  {"right_hip_yaw_joint",        kRightHipYaw},
  {"right_knee_joint",           kRightKnee},
  {"right_ankle_pitch_joint",    kRightAnklePitch},
  {"right_ankle_roll_joint",     kRightAnkleRoll},
  // Waist (state-only)
  {"waist_yaw_joint",            kWaistYaw},
  {"waist_roll_joint",           kWaistRoll},
  {"waist_pitch_joint",          kWaistPitch},
  // Arms (state + command)
  {"left_shoulder_pitch_joint",  kLeftShoulderPitch},
  {"left_shoulder_roll_joint",   kLeftShoulderRoll},
  {"left_shoulder_yaw_joint",    kLeftShoulderYaw},
  {"left_elbow_joint",           kLeftElbow},
  {"left_wrist_roll_joint",      kLeftWristRoll},
  {"left_wrist_pitch_joint",     kLeftWristPitch},
  {"left_wrist_yaw_joint",       kLeftWristYaw},
  {"right_shoulder_pitch_joint", kRightShoulderPitch},
  {"right_shoulder_roll_joint",  kRightShoulderRoll},
  {"right_shoulder_yaw_joint",   kRightShoulderYaw},
  {"right_elbow_joint",          kRightElbow},
  {"right_wrist_roll_joint",     kRightWristRoll},
  {"right_wrist_pitch_joint",    kRightWristPitch},
  {"right_wrist_yaw_joint",      kRightWristYaw},
};

class G1HardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(G1HardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Hardware parameters
  std::string network_interface_;
  float kp_{60.0f};
  float kd_{1.5f};
  float weight_rate_{0.2f};   // weight units per second

  // Per-joint data (indexed by info_.joints order)
  std::vector<int>    sdk_indices_;   // SDK motor array index for each joint
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;

  // Weight ramp state
  float weight_{0.0f};
  bool  deactivating_{false};
  int   active_command_interfaces_{0};  // incremented/decremented by perform_command_mode_switch

  // DDS
  unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_>   arm_sdk_publisher_;
  unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> low_state_subscriber_;
  unitree_hg::msg::dds_::LowState_ state_msg_;
  std::mutex state_mutex_;
  bool state_received_{false};

  static constexpr const char * kTopicArmSdk  = "rt/arm_sdk";
  static constexpr const char * kTopicLowState = "rt/lowstate";
};

}  // namespace g1_hardware
