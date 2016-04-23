cmake_minimum_required(VERSION 2.8.3)
project(ros_robot_wandering_demo)

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

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  dynamic_reconfigure
)

#########################################################
# CFG FILES
# Adding CFG files to project source, without compiling them
file(GLOB_RECURSE CFG . *.cfg)
set_source_files_properties(${CFG} PROPERTIES HEADER_FILE_ONLY TRUE)

generate_dynamic_reconfigure_options(
    ${CFG}
)
#########################################################

# Declare our catkin package.
catkin_package(
   CATKIN_DEPENDS
   geometry_msgs
   std_msgs
   std_srvs
   dynamic_reconfigure
)

#########################################################
# SOURCES
file(GLOB_RECURSE ${PROJECT_NAME}_SRC *.cpp)
# HEADERS
file(GLOB_RECURSE ${PROJECT_NAME}_HDR *.h)
#########################################################

#########################################################
# LAUNCH FILES
# Adding LAUNCH files to project source, without compiling them
file(GLOB_RECURSE LAUNCH . *.launch)
set_source_files_properties(${LAUNCH} PROPERTIES HEADER_FILE_ONLY TRUE)
#########################################################

#########################################################
# EXECUTABLE
add_executable( 
    robot_wandering_node
    ${${PROJECT_NAME}_SRC}
    ${${PROJECT_NAME}_HDR}
    ${LAUNCH}
    ${CFG}
)

target_link_libraries(
    robot_wandering_node
    ${catkin_LIBRARIES}
)


