cmake_minimum_required(VERSION 3.8)
project(swarm_localization)
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  set_initial_poses.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME set_initial_poses
)
ament_package()
