cmake_minimum_required(VERSION 2.8.3)
project(map_merge)

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

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES p3dx_dpl
#  CATKIN_DEPENDS tf
#  DEPENDS system_lib
)

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

add_executable(map_merge src/map_merging.cpp)
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
