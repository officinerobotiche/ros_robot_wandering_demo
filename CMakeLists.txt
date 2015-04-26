cmake_minimum_required(VERSION 2.8.3)
project(ros_robot_wandering_demo)

set(ROS_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  	tf
)

# Declare our catkin package.
catkin_package(
   CATKIN_DEPENDS geometry_msgs message_runtime std_msgs std_srvs
)

#########################################################
# SOURCES
set(${PROJECT_NAME}_SRC
	src/robot_wandering_node.cpp
)

#########################################################
# EXECUTABLE
add_executable( 
        robot_wandering_node
	${${PROJECT_NAME}_SRC}
)

target_link_libraries(
        robot_wandering_node
	${catkin_LIBRARIES}	
)


