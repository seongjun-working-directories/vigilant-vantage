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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/angles.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

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
  ros::Subscriber mode_write, mode_read;

  float value, angle, angle_increment, angle_min;
  int count, mode;
  bool wait, check, trigger;

  bool is_reference_mode, is_current_mode;

  std::vector<cv::Point2f> current;
  std::vector<cv::Point2f> ref1;
  dock_ICP dock_icp;

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
};

#endif