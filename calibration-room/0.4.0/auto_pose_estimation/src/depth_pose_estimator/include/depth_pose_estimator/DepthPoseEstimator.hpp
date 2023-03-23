#ifndef DEPTH_POSE_ESTIMATOR_HPP
#define DEPTH_POSE_ESTIMATOR_HPP

#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

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
#include "opencv2/core/eigen.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <thread>
#include <vector>
#include <set>
#include <tuple>

#include <Eigen/Core>
#include <Eigen/Sparse>

class DepthPoseEstimator
{
private:
  // *** variables *** //
  image_transport::Subscriber image_subscriber;
  ros::Subscriber depth_subscriber;
  ros::Publisher world_corners_publisher, camera_corners_publisher;
  ros::Publisher verification_publisher;
  ros::Publisher compressed_image_publisher_1, compressed_image_publisher_2;

  std::string depth_color_topic, depth_registered_topic, depth_tf;

  int frames_per_seconds;
  cv::Size chessboard_dimensions;
  float chessboard_edge_size;

  std::vector<cv::Point3f> object_points;
  pcl::PointCloud<pcl::PointXYZ> world_corners_messages;

  double* intrinsic_parameters;
  double* radial_tangential_distortion;

  cv::Mat camera_martix, distortion_coefficients;
  cv::Mat rotation_vector, translation_vector;

  std::vector<cv::Point2f> detected_corners;

public:
  // *** Constructor && Destrcutor *** //
  DepthPoseEstimator(
    ros::NodeHandle& node_handle, float _chessboard_edge_size,
    int _chessboard_width, int _chessboard_height, int _frames_per_seconds
  );
  ~DepthPoseEstimator();

  // *** callbacks *** //
  void image_callback(const sensor_msgs::Image::ConstPtr& image_messages);
  void depth_callback(const sensor_msgs::PointCloud2::ConstPtr& depth_messages);

  // *** core methods *** //
  void chessboard_detection(cv::Mat& copied_frame);
  std::tuple<Eigen::Matrix3f, Eigen::Vector3f> rigid_transformation(std::vector<cv::Point3f> corner_information_vector);
  void pose_verification(std::tuple<Eigen::Matrix3f, Eigen::Vector3f> Rt);

  // *** other methods *** //
  void create_world_coordinate_system();
  int encoding_mat_type(const std::string& encoding);
  pcl::PointCloud<pcl::PointXYZ> sensor_to_pcl(sensor_msgs::PointCloud2 sensor_messages);
  sensor_msgs::PointCloud2 pcl_to_sensor(pcl::PointCloud<pcl::PointXYZ> pcl_messages);
  void parameter_initializer(ros::NodeHandle& node_handle);
};

#endif