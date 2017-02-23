cmake_minimum_required(VERSION 2.8.3)
project(trinbot_core)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs sensor_msgs opencv)

add_message_files(
  DIRECTORY msg
  FILES
  #ballcoords.msg
  #command.msg
  )

catkin_python_setup()

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(	CATKIN_DEPENDS message_runtime std_msgs)
