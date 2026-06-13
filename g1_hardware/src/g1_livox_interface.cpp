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

#include "g1_hardware/g1_livox_interface.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace g1_hardware
{
G1LivoxInterfaceNode::G1LivoxInterfaceNode(const rclcpp::NodeOptions& options)
: Node("g1_livox_interface", options)
{
  const std::string network_interface = declare_parameter<std::string>("network_interface");

  const rclcpp::QoS sensor_qos(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);
  lidar_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("livox/lidar", sensor_qos);
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("livox/imu", sensor_qos);

  // initialize DDS
  unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
  lidar_sub_ = unitree::robot::ChannelSubscriberPtr<sensor_msgs::msg::dds_::PointCloud2_>(
      new unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_>(
          "rt/utlidar/cloud_livox_mid360"));
  imu_sub_ = unitree::robot::ChannelSubscriberPtr<sensor_msgs::msg::dds_::Imu_>(
      new unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::Imu_>(
          "rt/utlidar/imu_livox_mid360"));

  lidar_sub_->InitChannel(
      [this](const void* msg) {
        const sensor_msgs::msg::dds_::PointCloud2_* lidar_msg =
            static_cast<const sensor_msgs::msg::dds_::PointCloud2_*>(msg);
        frame_id_ = lidar_msg->header().frame_id();
        const auto ros_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        ros_msg->header.stamp.sec = lidar_msg->header().stamp().sec();
        ros_msg->header.stamp.nanosec = lidar_msg->header().stamp().nanosec();
        ros_msg->header.frame_id = frame_id_;
        ros_msg->height = lidar_msg->height();
        ros_msg->width = lidar_msg->width();
        const auto& fields = lidar_msg->fields();
        ros_msg->fields.reserve(fields.size());
        for (const auto& field : fields) {
          sensor_msgs::msg::PointField ros_field;
          ros_field.name = field.name();
          ros_field.offset = field.offset();
          ros_field.datatype = field.datatype();
          ros_field.count = field.count();
          ros_msg->fields.push_back(ros_field);
        }
        ros_msg->is_bigendian = lidar_msg->is_bigendian();
        ros_msg->point_step = lidar_msg->point_step();
        ros_msg->row_step = lidar_msg->row_step();
        ros_msg->data = lidar_msg->data();
        ros_msg->is_dense = lidar_msg->is_dense();
        lidar_pub_->publish(*ros_msg);
      },
      1);

  imu_sub_->InitChannel(
      [this](const void* msg) {
        if (frame_id_.empty()) {
          return;
        }
        const sensor_msgs::msg::dds_::Imu_* imu_msg =
            static_cast<const sensor_msgs::msg::dds_::Imu_*>(msg);
        const auto ros_msg = std::make_shared<sensor_msgs::msg::Imu>();
        ros_msg->header.stamp.sec = imu_msg->header().stamp().sec();
        ros_msg->header.stamp.nanosec = imu_msg->header().stamp().nanosec();
        ros_msg->header.frame_id = imu_msg->header().frame_id();
        ros_msg->orientation.x = imu_msg->orientation().x();
        ros_msg->orientation.y = imu_msg->orientation().y();
        ros_msg->orientation.z = imu_msg->orientation().z();
        ros_msg->orientation.w = imu_msg->orientation().w();
        ros_msg->orientation_covariance = imu_msg->orientation_covariance();
        ros_msg->angular_velocity.x = imu_msg->angular_velocity().x();
        ros_msg->angular_velocity.y = imu_msg->angular_velocity().y();
        ros_msg->angular_velocity.z = imu_msg->angular_velocity().z();
        ros_msg->angular_velocity_covariance = imu_msg->angular_velocity_covariance();
        ros_msg->linear_acceleration.x = imu_msg->linear_acceleration().x();
        ros_msg->linear_acceleration.y = imu_msg->linear_acceleration().y();
        ros_msg->linear_acceleration.z = imu_msg->linear_acceleration().z();
        ros_msg->linear_acceleration_covariance = imu_msg->linear_acceleration_covariance();
        imu_pub_->publish(*ros_msg);
      },
      1);
}

} // namespace g1_hardware

RCLCPP_COMPONENTS_REGISTER_NODE(g1_hardware::G1LivoxInterfaceNode);
