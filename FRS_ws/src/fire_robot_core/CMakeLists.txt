cmake_minimum_required(VERSION 3.0.2)
project(fire_robot_core)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/fire_detection_publisher.py
  scripts/rviz_simulation_controller.py
  scripts/real_robot_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
