cmake_minimum_required(VERSION 3.8)
project(xarm_controller_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(HIDAPI REQUIRED IMPORTED_TARGET hidapi-hidraw)

add_library(${PROJECT_NAME} SHARED
  src/xarm_hardware.cpp
  src/xarm_sdk.cpp
)

target_link_libraries(${PROJECT_NAME}
  PkgConfig::HIDAPI
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  include
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

pluginlib_export_plugin_description_file(
  hardware_interface
  xarm_hardware.xml
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY test config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES xarm_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()