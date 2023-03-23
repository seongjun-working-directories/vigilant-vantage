cmake_minimum_required(VERSION 3.0.2)
project(aruco_generator)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  laser_geometry
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES utilities_aruco_generator
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport laser_geometry pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   sensor_msgs#   std_msgs
# )

find_package(OpenCV REQUIRED)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(aruco_generation src/aruco_generation.cpp)
target_link_libraries(aruco_generation ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})