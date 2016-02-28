cmake_minimum_required(VERSION 2.8.3)
project(multirobot_map_merge)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  tf
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

add_executable(map_merge
  src/map_merge.cpp
  src/combine_grids.cpp
  src/coordinate_conversions.cpp
)
target_link_libraries(map_merge ${catkin_LIBRARIES})

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
