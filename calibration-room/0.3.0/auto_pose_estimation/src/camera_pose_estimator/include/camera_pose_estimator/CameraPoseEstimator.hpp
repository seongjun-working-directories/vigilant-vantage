#ifndef CAMERA_POSE_ESTIMATOR_HPP
#define CAMERA_POSE_ESTIMATOR_HPP

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "opencv2/opencv.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/calib3d/calib3d_c.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/aruco.hpp"

#include <iostream>
#include <vector>

#define BOARD_LENGTH 0.033
#define GAP 0.007

class CameraPoseEstimator
{
private:
  ros::NodeHandle node_handle;
  image_transport::Subscriber image_subscriber;
  ros::Publisher compressed_image_publisher_1, compressed_image_publisher_2;

  cv::Mat image, copied_image;

  std::ostringstream vector_to_marker;
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;

  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(5, 7, BOARD_LENGTH, GAP, dictionary);

  cv::Mat intrinsic_parameter = (
    cv::Mat1d(3,3) << 950.3955649641379, 0, 617.4085543989751,
                      0, 949.664898671666, 384.055444194634,
                      0, 0, 1
  );
  cv::Mat distortion_coefficient = (
    cv::Mat1d(1,5) << -0.4179201177519848,
                      0.2459347422574937,
                      0.000295633578362335,
                      -0.0002136899031111,
                      -0.1050203120225495
  );

public:
  CameraPoseEstimator();
  ~CameraPoseEstimator();

  void camera_callback(const sensor_msgs::Image::ConstPtr& msg);
  void estimate_pose();
  void get_eular_angles(cv::Mat &rotation_camera_matrix, cv::Vec3d& euler_angles);
};

#endif