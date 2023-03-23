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
  ros::Publisher compressed_image_publisher_1;
	ros::Publisher compressed_image_publisher_2;

	std::string lidar_tf_topic;
	float inlierratio = 0.0;
	CvMat test_r1, test_t1;
	std::vector<cv::Point2f> RT_ref_data;

	dock_ICP(std::string _lidar_image_topic_1, std::string _lidar_image_topic_2, std::string _lidar_tf_topic);
	~dock_ICP();

	float icp(const cv::Point2f* new_points, int nb_point_new, const cv::Point2f* ref_points, int nb_point_ref, CvMat * r_final, CvMat *t_final, CvTermCriteria criteria);
	bool getCovICP(const cv::Point2f* r, int nb_point_ref, const cv::Point2f* m, int nb_ref, float *dxx, float *dxy, float *dyx, float *dyy);

	void getRTMatrixSVD(const cv::Point2f* a, const cv::Point2f* b, int count, CvMat *r, CvMat *t);
	bool getRTMatrixSVD_filter(const cv::Point2f* r, const cv::Point2f* m, int nb_point_ref, int count, CvMat * r_final, CvMat *t_final, float *inlierratio);
	float dist_sq(float *a1, float*a2, int dims);
	cv::Mat drawDistribution(cv::Mat *input, float normalized = 0.0f);
};

#endif