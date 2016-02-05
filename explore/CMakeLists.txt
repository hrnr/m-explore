cmake_minimum_required(VERSION 2.8.3)
project(explore_lite)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  move_base_msgs
  visualization_msgs
  geometry_msgs
  map_msgs
  nav_msgs
  actionlib_msgs
  tf
  costmap_2d
  actionlib
  navfn
  roslaunch
 )

find_package(Boost REQUIRED COMPONENTS thread)

################################################
## Declare ROS messages, services and actions ##
################################################
# we don't have any

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########
# c++11 support required
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
)

add_library(explore_lib
  src/explore_frontier.cpp
  src/loop_closure.cpp
  src/costmap_client.cpp)
set_target_properties(explore_lib
                      PROPERTIES OUTPUT_NAME "explore")

add_executable(explore src/explore.cpp)
target_link_libraries(explore ${catkin_LIBRARIES})
target_link_libraries(explore explore_lib)

#############
## Install ##
#############

# install roslaunch files
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

#############
## Testing ##
#############

# test all launch files
roslaunch_add_file_check(launch)
