#include "depth_pose_estimator/DepthPoseEstimator.hpp"

static inline void aruco_generator(int width = 5, int height = 7, float marker_size = 0.04f, float marker_seperation_size = 0.004f)
{
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  // cv::aruco::GridBoard::create(y, x, ...);
  cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(width, height, marker_size, marker_seperation_size, dictionary);
  cv::Mat arucoboard;
  board->draw(cv::Size(600, 500), arucoboard, 10, 1);
  cv::imwrite("arucoboard.jpg", arucoboard);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_pose_estimator");
  ros::NodeHandle node_handle;

  aruco_generator();

  float _marker_size, _marker_seperation_size;
  int _arucoboard_height, _arucoboard_width, _frames_per_seconds;
  node_handle.getParam("/depth_pose_estimator/marker_size", _marker_size);              // 단위: meter
  node_handle.getParam("/depth_pose_estimator/marker_seperation_size", _marker_seperation_size);  // 단위: meter
  node_handle.getParam("/depth_pose_estimator/arucoboard_width", _arucoboard_width);    // y
  node_handle.getParam("/depth_pose_estimator/arucoboard_height", _arucoboard_height);  // x
  node_handle.getParam("/depth_pose_estimator/frames_per_seconds", _frames_per_seconds);  // 초당 프레임

  DepthPoseEstimator depth_pose_estimator =
    // DepthPoseEstimator(..., y, x, ...);
    DepthPoseEstimator(node_handle, _marker_size, _marker_seperation_size, _arucoboard_width, _arucoboard_height, _frames_per_seconds);

  ros::spin();

  return 0;
}