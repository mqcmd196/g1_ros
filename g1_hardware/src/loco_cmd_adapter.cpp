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

#include "g1_hardware/loco_cmd_adapter.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace loco_cmd_adapter
{
LocoCmdAdapterNode::LocoCmdAdapterNode(const rclcpp::NodeOptions options)
: Node("loco_cmd_adapter", options)
{
  declare_parameter<std::string>("network_interface");
  network_interface_ = get_parameter("network_interface").as_string();

  RCLCPP_INFO(get_logger(), "Activating... connecting to robot on interface '%s'",
              network_interface_.c_str());

  unitree::robot::ChannelFactory::Instance()->Init(0, network_interface_);
  loco_client_ = std::make_unique<unitree::robot::g1::LocoClient>();
  loco_client_->Init();

  RCLCPP_INFO(get_logger(), "Connection activated");

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [this](const geometry_msgs::msg::Twist& msg) {
        loco_client_->Move(msg.linear.x, msg.linear.y, msg.angular.z);
      });
}
} // namespace loco_cmd_adapter

RCLCPP_COMPONENTS_REGISTER_NODE(loco_cmd_adapter::LocoCmdAdapterNode);
