#ifndef DOCK_ICP_HPP
#define DOCK_ICP_HPP

#include "sensor_msgs/CompressedImage.h"

#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/core/core_c.h"
#include <iostream>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "lidar_pose_estimator/kdtree.hpp"

class dock_ICP
{
public:
	ros::NodeHandle node_handle;
  ros::Publisher compressed_image_publisher_1
		= node_handle.advertise<sensor_msgs::CompressedImage>("lidar_image_message_1", 1);
	ros::Publisher compressed_image_publisher_2
		= node_handle.advertise<sensor_msgs::CompressedImage>("lidar_image_message_2", 1);

	float icp(const cv::Point2f* new_points, int nb_point_new, const cv::Point2f* ref_points, int nb_point_ref, CvMat * r_final, CvMat *t_final, CvTermCriteria criteria);
	float inlierratio = 0.0;
	bool getCovICP(const cv::Point2f* r, int nb_point_ref, const cv::Point2f* m, int nb_ref, float *dxx, float *dxy, float *dyx, float *dyy);

	void getRTMatrixSVD(const cv::Point2f* a, const cv::Point2f* b, int count, CvMat *r, CvMat *t);
	bool getRTMatrixSVD_filter(const cv::Point2f* r, const cv::Point2f* m, int nb_point_ref, int count, CvMat * r_final, CvMat *t_final, float *inlierratio);
	float dist_sq(float *a1, float*a2, int dims);
	cv::Mat drawDistribution(cv::Mat *input, float normalized = 0.0f);

	std::vector<cv::Point2f> RT_ref_data;

	CvMat test_r1;
	CvMat test_t1;

	dock_ICP(){}
	~dock_ICP(){}
};

#endif