cmake_minimum_required(VERSION 3.0.2)
project(lidar_pose_estimator)
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
find_package(PCL 1.10 REQUIRED)
find_package(Boost REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES lidar_pose_estimator
 CATKIN_DEPENDS cv_bridge geometry_msgs image_transport laser_geometry pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(lidar_pose_estimator src/lidar_pose_estimator.cpp src/LidarPoseEstimator.cpp src/LidarPoseEstimatorCallbacks.cpp src/ICP.cpp src/kdtree.cpp)
target_link_libraries(lidar_pose_estimator ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})