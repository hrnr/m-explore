cmake_minimum_required(VERSION 2.8.3)
project(multirobot_map_merge)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  map_msgs
  tf
  roslaunch
)

find_package(Boost REQUIRED COMPONENTS thread)

# OpenCV is required for merging without initial positions
find_package(OpenCV 3 REQUIRED)
if(OpenCV_VERSION VERSION_LESS "3.0")
  message(FATAL_ERROR "This package needs OpenCV >= 3.0")
endif()

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
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)

# we want static linking for now
add_library(combine_grids STATIC
  src/combine_grids.cpp
  src/coordinate_conversions.cpp
  src/combine_grids/estimate_transform.cpp
  src/combine_grids/features_matcher.cpp
  src/combine_grids/estimate_rigid_transform.cpp
  src/combine_grids/transform_estimator.cpp
)
target_link_libraries(combine_grids ${OpenCV_LIBRARIES})

add_executable(map_merge
  src/map_merge.cpp
)
target_link_libraries(map_merge combine_grids ${catkin_LIBRARIES})

#############
## Install ##
#############

# install roslaunch files
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(combine_grid_tests
    test/test_grid_utils.cpp
  )
  target_link_libraries(combine_grid_tests combine_grids ${catkin_LIBRARIES})

  # test all launch files
  roslaunch_add_file_check(launch)
  roslaunch_add_file_check(launch/experiments)
endif()

