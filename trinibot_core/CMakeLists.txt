cmake_minimum_required(VERSION 2.8.3)
project(trinibot_core)

find_package(catkin REQUIRED COMPONENTS rospy roscpp message_generation std_msgs tf2 tf2_ros actionlib_msgs)

#find_package(ompl)

#catkin_python_setup()

add_action_files(
	DIRECTORY action
	FILES move_trinibot.action
)
generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

catkin_package(	CATKIN_DEPENDS rospy roscpp std_msgs message_runtime tf2 tf2_ros actionlib_msgs)

#install(DIRECTORY launch
#	    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#s	    )

#catkin_install_python(PROGRAMS
#			nodes/tbmotioncontroller.py
#			nodes/tbplatformtransform.py
#			nodes/tbsensehat.py
#			nodes/tbteleopgui.py
#		      	scripts/tbmotorcontroller.py
 #                     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
###############################################################################
# INCLUDES
#set(trinibot_INCLUDE_DIRS src/include)


# Specify locations of header files.
#include_directories(include ${catkin_INCLUDE_DIRS})
#include_directories(include	${trinibot_INCLUDE_DIRS})
	    
###############################################################################

###############################################################################
# EXECUTABLE
#add_executable(
#  tbpathplanner_node src/tbpathplanner_node.cpp
#)

#target_link_libraries(tbpathplanner_node ${catkin_LIBRARIES})
#install(TARGETS tbpathplanner_node 
