################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(map_tag_handler)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
#  explore_labyrinth
)

################################################################################
# Setup for python modules and scripts
################################################################################
catkin_python_setup()

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs 
)

################################################################################
# Build
################################################################################
#include_directories(
#  ${catkin_INCLUDE_DIRS}
#  /home/christoph/catkin_ws/src/map_tag_handler
#)
################################################################################
# Install
################################################################################
catkin_install_python(PROGRAMS
  nodes/map_tag_handler.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#add_dependencies(explore_labyrinth ${${PROJECT_NAME}_EXPORTED_TARGETS})

################################################################################
# Test
################################################################################
