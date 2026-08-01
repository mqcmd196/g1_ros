// Copyright 2026 Takuma Hiraoka
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

#include "g1_hardware/g1_odom_interface.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace g1_hardware
{
G1OdomInterfaceNode::G1OdomInterfaceNode(const rclcpp::NodeOptions& options)
: Node("g1_odom_interface", options)
{
  const std::string network_interface = declare_parameter<std::string>("network_interface");

  const rclcpp::QoS sensor_qos(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", sensor_qos);

  unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
  // The G1 odometer service publishes a SportModeState-compatible sample on
  // this DDS topic. At present only position, velocity and quaternion are
  // populated by the service; leave every other Odometry field zero/empty.
  odom_sub_ = unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_>(
      new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(
          "rt/lf/odommodestate"));

  odom_sub_->InitChannel(
      [this](const void* msg) {
        const auto* odom_msg = static_cast<const unitree_go::msg::dds_::SportModeState_*>(msg);
        nav_msgs::msg::Odometry ros_msg{};

        const auto& position = odom_msg->position();
        ros_msg.pose.pose.position.x = position[0];
        ros_msg.pose.pose.position.y = position[1];
        ros_msg.pose.pose.position.z = position[2];

        // Unitree quaternion order is [w, x, y, z], whereas ROS uses x/y/z/w.
        const auto& quaternion = odom_msg->imu_state().quaternion();
        ros_msg.pose.pose.orientation.x = quaternion[1];
        ros_msg.pose.pose.orientation.y = quaternion[2];
        ros_msg.pose.pose.orientation.z = quaternion[3];
        ros_msg.pose.pose.orientation.w = quaternion[0];

        const auto& velocity = odom_msg->velocity();
        ros_msg.twist.twist.linear.x = velocity[0];
        ros_msg.twist.twist.linear.y = velocity[1];
        ros_msg.twist.twist.linear.z = velocity[2];

        odom_pub_->publish(ros_msg);
      },
      1);
}

} // namespace g1_hardware

RCLCPP_COMPONENTS_REGISTER_NODE(g1_hardware::G1OdomInterfaceNode)
