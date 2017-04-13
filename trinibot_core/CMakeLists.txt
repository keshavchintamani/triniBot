cmake_minimum_required(VERSION 2.8.3)
project(trinibot_core)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs tf actionlib_msgs)

add_action_files(
	DIRECTORY action
	FILES move_trinibot.action
)
catkin_python_setup()
generate_messages(DEPENDENCIES std_msgs actionlib_msgs)
catkin_package(	CATKIN_DEPENDS std_msgs message_runtime tf actionlib_msgs	)



