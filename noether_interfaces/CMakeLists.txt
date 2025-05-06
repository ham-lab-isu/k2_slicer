cmake_minimum_required(VERSION 3.8)
project(noether_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate the service interface
rosidl_generate_interfaces(noether_interfaces
  "msg/ToolPath.msg"
  "msg/ToolPathPose.msg"
  "srv/PlanSurface.srv"
  DEPENDENCIES geometry_msgs
)

ament_package()
