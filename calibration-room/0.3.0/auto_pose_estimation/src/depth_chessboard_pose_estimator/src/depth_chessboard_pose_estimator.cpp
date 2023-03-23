#include "depth_chessboard_pose_estimator/DepthChessboardPoseEstimator.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_chessboard_pose_estimator");
  ros::NodeHandle node_handle;

  float _chessboard_edge_size;
  int _chessboard_width, _chessboard_height, _frames_per_seconds;

  node_handle.getParam("/depth_chessboard_pose_estimator/chessboard_edge_size", _chessboard_edge_size);
  node_handle.getParam("/depth_chessboard_pose_estimator/chessboard_width", _chessboard_width);
  node_handle.getParam("/depth_chessboard_pose_estimator/chessboard_height", _chessboard_height);
  node_handle.getParam("/depth_chessboard_pose_estimator/frames_per_seconds", _frames_per_seconds);

  DepthChessboardPoseEstimator depth_chessboard_pose_estimator =
    DepthChessboardPoseEstimator(node_handle, _chessboard_edge_size, _chessboard_width, _chessboard_height, _frames_per_seconds);

  ros::spin();

  return 0;
}