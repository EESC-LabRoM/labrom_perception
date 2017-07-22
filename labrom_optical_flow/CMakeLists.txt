cmake_minimum_required(VERSION 2.8.3)
project(labrom_optical_flow)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  sensor_msgs
  nav_msgs
  roscpp
  rospy
  tf
)

find_package(OpenCV REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
   src/lucas_kanade.cpp
   src/reprojection.cpp
   src/continuous_homography.cpp
   src/continuous_homography_estimator.cpp
   src/continuous_homography_wrapper.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(continuous_homography_node nodes/continuous_homography_node.cpp)
target_link_libraries(continuous_homography_node ${catkin_LIBRARIES} ${PROJECT_NAME} )