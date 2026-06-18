cmake_minimum_required(VERSION 2.8.3)
project(navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  duckietown_msgs
  path_planning
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs duckietown_msgs path_planning
)

install(PROGRAMS
  nodes/navigation_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)