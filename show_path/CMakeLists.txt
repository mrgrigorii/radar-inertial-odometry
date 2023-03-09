cmake_minimum_required(VERSION 3.0.2)
project(showpath)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Eigen3 REQUIRED)
include_directories(
${EIGEN3_INCLUDE_DIR}
)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf2_ros
  visualization_msgs
  geometry_msgs
  # Eigen3 REQUIRED
  message_generation
  message_filters
)


generate_messages(
   DEPENDENCIES
   geometry_msgs
   nav_msgs
   std_msgs  # Or other packages containing msgs
   sensor_msgs
)

catkin_package(
#  LIBRARIES onboard-sdk-ros
  CATKIN_DEPENDS
    message_runtime
    geometry_msgs
    nav_msgs
    std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_gps src/gps.cpp)


 target_link_libraries(${PROJECT_NAME}_gps
   ${catkin_LIBRARIES}
 )







