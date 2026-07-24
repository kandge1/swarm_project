cmake_minimum_required(VERSION 3.16)
project(mycobot_280pi_camera_moveit2)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

# Alias share directory + ament_index marker under "firefighter_moveit_config".
# MoveItConfigsBuilder("firefighter") resolves the config package via the
# convention {robot_name}_moveit_config -- some distros' moveit_configs_utils
# (e.g. ROS2 Galactic, which the physical robot runs) don't support the
# package_name= override kwarg to redirect that lookup to our actual package
# name (mycobot_280pi_camera_moveit2), so this alias lets the DEFAULT
# convention resolve correctly on every distro instead. Purely additive --
# does not rename or replace the real package, which is still what you
# `colcon build`/`ros2 launch` by name.
install(DIRECTORY launch DESTINATION share/firefighter_moveit_config
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/firefighter_moveit_config)
install(DIRECTORY worlds DESTINATION share/firefighter_moveit_config)
install(FILES .setup_assistant DESTINATION share/firefighter_moveit_config)

file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/firefighter_moveit_config_marker" "")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/firefighter_moveit_config_marker"
  DESTINATION share/ament_index/resource_index/packages
  RENAME firefighter_moveit_config)
