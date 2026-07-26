cmake_minimum_required(VERSION 3.16)
project(mycobot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(mycobot_hardware SHARED
  src/mycobot_system.cpp
)
target_include_directories(mycobot_hardware PRIVATE include)
ament_target_dependencies(mycobot_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface mycobot_hardware.xml)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS mycobot_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  PROGRAMS
    scripts/mycobot_bridge.py
    # Diagnostic only, never launched -- run by hand with the bridge stopped.
    scripts/serial_rate_probe.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(mycobot_hardware)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)

ament_package()
