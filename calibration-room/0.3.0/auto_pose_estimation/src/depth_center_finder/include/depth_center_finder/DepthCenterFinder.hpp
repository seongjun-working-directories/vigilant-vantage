#ifndef DEPTH_CENTER_FINDER_HPP
#define DEPTH_CENTER_FINDER_HPP

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "image_transport/image_transport.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/angles.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <tuple>
#include <math.h>

#include <Eigen/Core>
#include <Eigen/Sparse>

class DepthCenterFinder
{
private:
  // *** variables *** //
  image_transport::Subscriber color_image_subscriber;
  image_transport::Subscriber ir_image_subscriber;
  ros::Subscriber store_image_command_subscriber;
  ros::Subscriber run_finder_command_subscriber;

  cv::Mat color_image;
  cv::Mat ir_image;

  std::vector<std::vector<cv::Point>> color_square_corners;
  std::vector<std::vector<cv::Point>> ir_square_corners;

  bool store_image_command_color;
  bool store_image_command_ir;

  double box_width;
  double box_height;

public:
  // *** constructor && destructor *** //
  DepthCenterFinder(ros::NodeHandle& node_handle, double _box_width, double _box_height);
  ~DepthCenterFinder();

  // *** callbacks *** //
  void color_image_callback(const sensor_msgs::Image::ConstPtr& messages);
  void ir_image_callback(const sensor_msgs::Image::ConstPtr& messages);
  void store_image_command_callback(const std_msgs::Bool::ConstPtr& messages);
  void run_finder_command_callback(const std_msgs::Bool::ConstPtr& messages);

  // *** core methods ***//
  std::vector<std::vector<cv::Point>> color_square_detection(cv::Mat image);  // 3 channels
  std::vector<std::vector<cv::Point>> ir_square_detection(cv::Mat image);     // 1 channel
  void draw_squares(std::string window_name, cv::Mat& image, const std::vector<std::vector<cv::Point> >& squares);
  void rigid_transformation();

  // *** other methods *** //
  double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);
  int encoding_mat_type(const std::string & encoding);
};

#endif