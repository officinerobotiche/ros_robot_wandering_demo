cmake_minimum_required(VERSION 2.8.3)
project(ros_robot_wandering_demo)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

#########################################################
## Options
option( DEBUG_ACTIVE "Enable Debug build" ON )

if(DEBUG_ACTIVE)
    MESSAGE( "Debug compilation active" )
    set(ROS_BUILD_TYPE Debug)
    set(CMAKE_BUILD_TYPE Debug)
else()
    MESSAGE( "Release compilation active" )
    set(ROS_BUILD_TYPE Release)
    set(CMAKE_BUILD_TYPE Release)
endif()
#########################################################

find_package(catkin REQUIRED 
		COMPONENTS
		  roscpp
		  std_msgs
		  sensor_msgs
		  geometry_msgs
		  dynamic_reconfigure
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/wandering_dyn_params.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    CATKIN_DEPENDS
    geometry_msgs
    std_msgs
    std_srvs
    dynamic_reconfigure
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Declare a cpp executable
add_executable(robot_wandering_node src/robot_wandering_node.cpp)
add_dependencies(robot_wandering_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(robot_wandering_node ${catkin_LIBRARIES})


