cmake_minimum_required(VERSION 2.8.3)
project(path_planning)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  PathPlan.msg
)

add_service_files(
  FILES
  PlanPath.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

install(PROGRAMS
  nodes/path_planner_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
