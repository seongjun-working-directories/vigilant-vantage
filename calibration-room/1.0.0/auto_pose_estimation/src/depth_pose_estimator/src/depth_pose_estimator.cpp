#include "depth_pose_estimator/DepthPoseEstimator.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_pose_estimator");
  ros::NodeHandle node_handle;

  DepthPoseEstimator depth_pose_estimator = DepthPoseEstimator(node_handle);

  ros::spin();

  return 0;
}