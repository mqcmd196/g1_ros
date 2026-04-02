cmake_minimum_required(VERSION 3.8)
project(g1_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# unitree_sdk2 uses a namespaced target, not auto-detectable via ament_auto
find_package(unitree_sdk2 REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/g1_hardware_interface.cpp
)

target_link_libraries(${PROJECT_NAME}
  unitree_sdk2
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "PLUGINLIB_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(hardware_interface g1_hardware_plugin.xml)

install(FILES g1_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME})

ament_auto_package(
  INSTALL_TO_SHARE config launch
)
