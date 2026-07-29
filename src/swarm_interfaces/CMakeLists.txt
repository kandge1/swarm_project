cmake_minimum_required(VERSION 3.8)
project(swarm_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# NOTE this package must build on BOTH machines (Galactic on the Pi, Jazzy on
# mars) from identical source, or the service type won't match across the DDS
# link. It has no dependencies beyond the rosidl generators precisely so there
# is nothing distro-specific to go wrong -- the .srv uses only builtin types.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BlockDetection.msg"
  "srv/DetectBlock.srv"
)

ament_package()
