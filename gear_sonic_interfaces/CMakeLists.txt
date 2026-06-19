cmake_minimum_required(VERSION 3.22)
project(gear_sonic_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_auto_generate_interfaces()

ament_package()
