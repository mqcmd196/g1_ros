cmake_minimum_required(VERSION 3.22)
project(g1_bringup)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

install(PROGRAMS
  scripts/inspire_dfx_hand_open_close_demo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME inspire_dfx_hand_open_close_demo
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
)
