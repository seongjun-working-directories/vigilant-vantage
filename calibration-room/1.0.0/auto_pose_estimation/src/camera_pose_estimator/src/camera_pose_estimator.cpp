#include "camera_pose_estimator/CameraPoseEstimator.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_pose_estimator");
  CameraPoseEstimator camera_pose_estimator;
  ros::spin();

  return 0;
}