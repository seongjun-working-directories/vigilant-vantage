#include "lidar_pose_estimator/LidarPoseEstimator.hpp"

LidarPoseEstimator::LidarPoseEstimator()
{
  is_reference_mode = false;
  is_current_mode = false;

  parameter_initializer();

  lidar_subscriber = node_handle.subscribe(lidar_topic, 10, &LidarPoseEstimator::laser_scan_callback, this);
  reference_subscriber = node_handle.subscribe(lidar_reference_topic, 1, &LidarPoseEstimator::reference_callback, this);
  current_subscriber = node_handle.subscribe(lidar_current_topic, 1, &LidarPoseEstimator::current_callback, this);

  value = 0.0f;
  angle = 0.0f;
  angle_increment = 0.0f;
  angle_min = 0.0f;
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

  dock_ICP dock_icp = dock_ICP(lidar_image_topic_1, lidar_image_topic_2, lidar_tf_topic);
  float lidar_err;

  // [TEST] std::cout << current.size() << "," << ref1.size() << std::endl;

  if (current.size() != 0 && ref1.size() != 0)
  {
    lidar_err = dock_icp.icp(&current[0], current.size(), &ref1[0], ref1.size(), &r, &t, cvTermCriteria(CV_TERMCRIT_ITER, 100, 0.1));
  }

  // [TEST] std::cout << "Checking Error " << lidar_err << std::endl;
}

void LidarPoseEstimator::parameter_initializer()
{
  ros::NodeHandle node_handle("~");

  std::string lidar_topic_ = "/invalid_scan";
  std::string lidar_image_topic_1_ = "/invalid_lidar_image_message_1";
  std::string lidar_image_topic_2_ = "/invalid_lidar_image_message_2";
  std::string lidar_reference_topic_ = "/invalid_lidar_reference";
  std::string lidar_current_topic_ = "/invalid_lidar_current";
  std::string lidar_tf_topic_ = "/invalid_lidar_tf";

  std::string node_name = ros::this_node::getName();
  // [TEST] std::cout << ros::this_node::getName() << std::endl;

  node_handle.param(node_name + "/LIDAR_TOPIC", lidar_topic, lidar_topic_);
  node_handle.param(node_name + "/LIDAR_IMAGE_1", lidar_image_topic_1, lidar_image_topic_1_);
  node_handle.param(node_name + "/LIDAR_IMAGE_2", lidar_image_topic_2, lidar_image_topic_2_);
  node_handle.param(node_name + "/LIDAR_REFERENCE", lidar_reference_topic, lidar_reference_topic_);
  node_handle.param(node_name + "/LIDAR_CURRENT", lidar_current_topic, lidar_current_topic_);
  node_handle.param(node_name + "/LIDAR_TF", lidar_tf_topic, lidar_tf_topic_);

  /* [TEST]
  std::cout << lidar_topic << " " 
    << lidar_image_topic_1 << " " 
    << lidar_image_topic_2 << " " 
    << lidar_reference_topic << " " 
    << lidar_current_topic << " " 
    << lidar_tf_topic << std::endl;
  */
}