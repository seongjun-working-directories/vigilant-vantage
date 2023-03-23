#include "lidar_pose_estimator/LidarPoseEstimator.hpp"

LidarPoseEstimator::LidarPoseEstimator()
{
  lidar_subscriber = node_handle.subscribe("/scan", 10, &LidarPoseEstimator::laser_scan_callback, this);
  mode_write = node_handle.subscribe("/lidar_reference", 1, &LidarPoseEstimator::reference_callback, this);
  mode_read = node_handle.subscribe("/lidar_current", 1, &LidarPoseEstimator::current_callback, this);

  value = 0.0f;
  angle = 0.0f;
  angle_increment = 0.0f;
  angle_min = 0.0f;

  is_reference_mode = false;
  is_current_mode = false;
}

LidarPoseEstimator::~LidarPoseEstimator()
{
  // YET
}

void LidarPoseEstimator::lidar_check()
{
  float R[4] = {1.0f, 0.0f, 0.0f, 1.0f};
  float T[2] = {0.0f, 0.0f};

  CvMat r = cvMat(2, 2, CV_32F, R);
  CvMat t = cvMat(2, 1, CV_32F, T);

  dock_ICP dock_icp;
  float lidar_err;

  // [TEST] std::cout << current.size() << "," << ref1.size() << std::endl;

  if (current.size() != 0 && ref1.size() != 0)
  {
    lidar_err = dock_icp.icp(&current[0], current.size(), &ref1[0], ref1.size(), &r, &t, cvTermCriteria(CV_TERMCRIT_ITER, 100, 0.1));
  }

  // [TEST]
  std::cout << "Checking Error " << lidar_err << std::endl;
}