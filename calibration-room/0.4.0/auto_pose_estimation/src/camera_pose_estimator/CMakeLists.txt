cmake_minimum_required(VERSION 3.0.2)
project(camera_pose_estimator)
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

find_package(OpenCV REQUIRED)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES camera_pose_estimator
 CATKIN_DEPENDS cv_bridge geometry_msgs image_transport laser_geometry pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(camera_pose_estimator src/camera_pose_estimator.cpp src/CameraPoseEstimator.cpp)
target_link_libraries(camera_pose_estimator ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
