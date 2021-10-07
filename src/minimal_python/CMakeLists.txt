cmake_minimum_required(VERSION 3.0.2)
project(minimal_python)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  minimal_example
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs minimal_example
)

catkin_install_python(PROGRAMS
  nodes/publisher
  nodes/subscriber
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  launch/*.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

