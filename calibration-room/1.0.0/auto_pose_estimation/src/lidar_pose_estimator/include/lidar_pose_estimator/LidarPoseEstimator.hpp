#ifndef LIDAR_POSE_ESTIMATOR_HPP
#define LIDAR_POSE_ESTIMATOR_HPP

#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "laser_geometry/laser_geometry.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/common/angles.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <thread>

#include "lidar_pose_estimator/ICP.hpp"

class LidarPoseEstimator
{
private:
  // *** variables *** //
  ros::NodeHandle node_handle;
  ros::Subscriber lidar_subscriber;
  ros::Subscriber reference_subscriber, current_subscriber;

  std::string lidar_topic;
  std::string lidar_image_topic_1, lidar_image_topic_2;
  std::string lidar_reference_topic, lidar_current_topic;
  std::string lidar_tf_topic;

  bool is_reference_mode, is_current_mode;

  std::vector<cv::Point2f> current;
  std::vector<cv::Point2f> ref1;

  float value, angle, angle_increment, angle_min;
  int count, mode;
  bool wait, check, trigger;

public:
  // *** constructor && destructor *** //
  LidarPoseEstimator();
  ~LidarPoseEstimator();

  // *** callback methods *** //
  void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_messages);
  void reference_callback(const std_msgs::Bool::ConstPtr &reference_bool);
  void current_callback(const std_msgs::Bool::ConstPtr &current_bool);

  // *** core methods *** //
  void lidar_check();

  // *** other methods *** //
  void parameter_initializer();
};

#endif