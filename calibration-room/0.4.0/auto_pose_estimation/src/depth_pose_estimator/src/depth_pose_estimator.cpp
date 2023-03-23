#include "depth_pose_estimator/DepthPoseEstimator.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_pose_estimator");
  ros::NodeHandle node_handle;

  float _chessboard_edge_size;
  int _chessboard_width, _chessboard_height, _frames_per_seconds;

  node_handle.getParam("/depth_pose_estimator/CHESSBOARD_EDGE_SIZE", _chessboard_edge_size);
  node_handle.getParam("/depth_pose_estimator/CHESSBOARD_WIDTH", _chessboard_width);
  node_handle.getParam("/depth_pose_estimator/CHESSBOARD_HEIGHT", _chessboard_height);
  node_handle.getParam("/depth_pose_estimator/FRAMES_PER_SECONDS", _frames_per_seconds);

  DepthPoseEstimator depth_pose_estimator =
    DepthPoseEstimator(node_handle, _chessboard_edge_size, _chessboard_width, _chessboard_height, _frames_per_seconds);

  ros::spin();

  return 0;
}