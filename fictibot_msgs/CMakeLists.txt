cmake_minimum_required(VERSION 2.8.3)
project(fictibot_msgs)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
)

########################
## Message generation ##
########################

add_message_files(
  FILES
  BumperEvent.msg
  WheelDropEvent.msg
  VelocityCommand.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)
