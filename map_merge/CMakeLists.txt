cmake_minimum_required(VERSION 2.8.3)
project(multirobot_map_merge)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  map_msgs
  tf2_geometry_msgs
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
  src/combine_grids/merging_pipeline.cpp
  src/combine_grids/grid_warper.cpp
  src/combine_grids/grid_compositor.cpp
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
  find_package(roslaunch REQUIRED)

  # download test data
  set(base_url https://raw.githubusercontent.com/hrnr/m-explore-extra/master/map_merge)
  catkin_download_test_data(${PROJECT_NAME}_map00.pgm ${base_url}/hector_maps/map00.pgm MD5 915609a85793ec1375f310d44f2daf87)
  catkin_download_test_data(${PROJECT_NAME}_map05.pgm ${base_url}/hector_maps/map05.pgm MD5 cb9154c9fa3d97e5e992592daca9853a)
  catkin_download_test_data(${PROJECT_NAME}_2011-08-09-12-22-52.pgm ${base_url}/gmapping_maps/2011-08-09-12-22-52.pgm MD5 3c2c38e7dec2b7a67f41069ab58badaa)
  catkin_download_test_data(${PROJECT_NAME}_2012-01-28-11-12-01.pgm ${base_url}/gmapping_maps/2012-01-28-11-12-01.pgm MD5 681e704044889c95e47b0c3aadd81f1e)

  catkin_add_gtest(test_merging_pipeline test/test_merging_pipeline.cpp)
  # ensure that test data are downloaded before we run tests
  add_dependencies(test_merging_pipeline ${PROJECT_NAME}_map00.pgm ${PROJECT_NAME}_map05.pgm ${PROJECT_NAME}_2011-08-09-12-22-52.pgm ${PROJECT_NAME}_2012-01-28-11-12-01.pgm)
  target_link_libraries(test_merging_pipeline combine_grids ${catkin_LIBRARIES})

  # test all launch files
  roslaunch_add_file_check(launch)
  roslaunch_add_file_check(launch/experiments)
endif()
