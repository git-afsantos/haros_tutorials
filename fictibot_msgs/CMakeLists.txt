cmake_minimum_required(VERSION 2.8.3)
project(fictibot_msgs)

find_package(catkin REQUIRED COMPONENTS roscpp rospy geometry_msgs message_generation)

########################
## Message generation ##
########################

add_message_files(
  FILES
  Custom.msg
)

generate_messages(
   DEPENDENCIES
   geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp rospy geometry_msgs message_runtime
)
