cmake_minimum_required(VERSION 2.8.3)
project(Project_refactoring)

set(CATKIN_TOPLEVEL TRUE)

find_package(catkin QUIET)

if(catkin_FOUND)
  include(${catkin_EXTRAS_DIR}/toplevel.cmake)
else()
  message(FATAL_ERROR
    "catkin was not found. Source your ROS environment before building, "
    "for example: source /opt/ros/<distro>/setup.bash")
endif()
