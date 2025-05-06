cmake_minimum_required(VERSION 3.5)
project(khi2cpp_hw_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch ros2_control common
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY cx110l/urdf cx110l/srdf cx110l/rviz cx110l/meshes
  DESTINATION share/${PROJECT_NAME}/cx110l
)

ament_package()
