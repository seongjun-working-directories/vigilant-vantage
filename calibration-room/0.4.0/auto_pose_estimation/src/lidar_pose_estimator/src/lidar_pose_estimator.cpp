#include "lidar_pose_estimator/LidarPoseEstimator.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_pose_estimator");
  LidarPoseEstimator lidar_pose_estimator;
  ros::spin();
}