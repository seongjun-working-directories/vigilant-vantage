#include "depth_chessboard_pose_estimator/DepthChessboardPoseEstimator.hpp"

// *** 생성자 함수 *** //
DepthChessboardPoseEstimator::DepthChessboardPoseEstimator(
  ros::NodeHandle& node_handle, float _chessboard_edge_size,
  int _chessboard_width, int _chessboard_height, int _frames_per_seconds
) {
  // subscriber
  image_subscriber = image_transport::ImageTransport(node_handle).subscribe("/camera/color/image_raw", 1, &DepthChessboardPoseEstimator::image_callback, this);
  depth_subscriber = node_handle.subscribe("/camera/depth_registered/points", 1, &DepthChessboardPoseEstimator::depth_callback, this);

  // publisher
  world_corners_publisher = node_handle.advertise<sensor_msgs::PointCloud2>("world_corners_messages", 1);
  camera_corners_publisher = node_handle.advertise<sensor_msgs::PointCloud2>("camera_corners_messages", 1);
  verification_publisher = node_handle.advertise<sensor_msgs::PointCloud2>("verification_messages", 1);

  // image_publisher
  compressed_image_publisher_1 = node_handle.advertise<sensor_msgs::CompressedImage>("depth_chessboard_image_message_1", 1);
  compressed_image_publisher_2 = node_handle.advertise<sensor_msgs::CompressedImage>("depth_chessboard_image_message_2", 1);

  // 초당 프레임
  frames_per_seconds = _frames_per_seconds;

  // chessboard 관련 변수
  chessboard_dimensions = cv::Size(_chessboard_width - 1, _chessboard_height - 1);  // y x 순
  chessboard_edge_size = _chessboard_edge_size;

  create_world_coordinate_system();

  // 내부 파라미터값 1
  double _intrinsic_parameters[] = {453.7543640, 0.0, 323.54632568, 0.0, 453.7543640, 241.9261780, 0.0, 0.0, 1.0};
  intrinsic_parameters = _intrinsic_parameters;
  double _radial_tangential_distortion[] = {0.0, 0.0, 0.0, 0.0};
  radial_tangential_distortion = _radial_tangential_distortion;

  // 내부 파라미터값 2
  camera_martix = cv::Mat(3, 3, CV_64FC1, intrinsic_parameters);
  distortion_coefficients = cv::Mat(4, 1, CV_64FC1, radial_tangential_distortion);

  cv::namedWindow("default");
  cv::namedWindow("found");
}

// *** 소멸자 함수 *** //
DepthChessboardPoseEstimator::~DepthChessboardPoseEstimator()
{
  cv::destroyWindow("default");
  cv::destroyWindow("found");
}

// *** Chessboard를 찾아내는 함수 *** //
void DepthChessboardPoseEstimator::chessboard_detection(cv::Mat& copied_frame)
{
  cv::Mat frame;
  copied_frame.copyTo(frame);


  std::vector<cv::Point2f> chessboard_corners;
  bool found = cv::findChessboardCorners(
    frame, chessboard_dimensions, chessboard_corners,
    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK
  );

  if (found)
  {
    detected_corners.clear();
    detected_corners = chessboard_corners;

    cv::drawChessboardCorners(frame, chessboard_dimensions, chessboard_corners, found);
    cv::imshow("found", frame);

    std::vector<uchar> data;
    sensor_msgs::CompressedImage compressed_image;
    compressed_image.header.stamp = ros::Time::now();
    compressed_image.format = "jpeg";
    cv::imencode(".jpg", frame, data);
    compressed_image.data = data;
    compressed_image_publisher_2.publish(compressed_image);
  }
  else
  {
    detected_corners.clear();
  }
}

// *** rigid_transformation 함수 *** //
std::tuple<Eigen::Matrix3f, Eigen::Vector3f> DepthChessboardPoseEstimator::rigid_transformation(std::vector<cv::Point3f> corner_information_vector)
{
  Eigen::Matrix<float, 3, Eigen::Dynamic> camera_coordinate_points, world_coordinate_points;
  Eigen::Matrix<float, 3, Eigen::Dynamic> camera_substract_mean, world_substract_mean;

  int corner_information_vector_size = corner_information_vector.size();
  // [TEST] std::cout << corner_information_vector_size << std::endl;

  camera_coordinate_points.resize(3, corner_information_vector_size);
  world_coordinate_points.resize(3, corner_information_vector_size);
  camera_substract_mean.resize(3, corner_information_vector_size);
  world_substract_mean.resize(3, corner_information_vector_size);

  int count = 0;
  for (auto &corner_information : corner_information_vector)
  {
    camera_coordinate_points(0, count) = corner_information.x;
    camera_coordinate_points(1, count) = corner_information.y;
    camera_coordinate_points(2, count) = corner_information.z;
    count++;
  }
  
  count = 0;
  for (auto &object_point : object_points)
  {
    world_coordinate_points(0 ,count) = object_point.x;
    world_coordinate_points(1, count) = object_point.y;
    world_coordinate_points(2, count) = object_point.z;
    count++;
  }

  const Eigen::Vector3f centroid_camera = camera_coordinate_points.rowwise().mean();
  const Eigen::Vector3f centroid_world = world_coordinate_points.rowwise().mean();

  camera_substract_mean = camera_coordinate_points.colwise() - centroid_camera;
  world_substract_mean = world_coordinate_points.colwise() - centroid_world;

  Eigen::Matrix<float, 3, Eigen::Dynamic> H = camera_substract_mean * world_substract_mean.transpose();
  Eigen::JacobiSVD<Eigen::Matrix3Xf> svd = H.jacobiSvd(Eigen::DecompositionOptions::ComputeFullU | Eigen::DecompositionOptions::ComputeFullV);

  const Eigen::Matrix3f& U = svd.matrixU();
  Eigen::MatrixXf V = svd.matrixV();
  Eigen::Matrix3f R = V * U.transpose();

  if (R.determinant() < 0.0f)
	{
		V.col(2) *= -1.0f;
		R = V * U.transpose();
	}

  const Eigen::Vector3f t = -R * centroid_camera + centroid_world;

  return std::make_tuple(R, t);
}

// *** 계산된 Rt 값을 기반으로 verification 하는 함수 *** //
void DepthChessboardPoseEstimator::pose_verification(std::tuple<Eigen::Matrix3f, Eigen::Vector3f> Rt)
{
  Eigen::Matrix<float, 4, 4> Rt_4x4;

  Rt_4x4(0, 0) = std::get<0>(Rt)(0, 0);
  Rt_4x4(0, 1) = std::get<0>(Rt)(0, 1);
  Rt_4x4(0, 2) = std::get<0>(Rt)(0, 2);
  Rt_4x4(0, 3) = 0;

  Rt_4x4(1, 0) = std::get<0>(Rt)(1, 0);
  Rt_4x4(1, 1) = std::get<0>(Rt)(1, 1);
  Rt_4x4(1, 2) = std::get<0>(Rt)(1, 2);
  Rt_4x4(1, 3) = 0;

  Rt_4x4(2, 0) = std::get<0>(Rt)(2, 0);
  Rt_4x4(2, 1) = std::get<0>(Rt)(2, 1);
  Rt_4x4(2, 2) = std::get<0>(Rt)(2, 2);
  Rt_4x4(2, 3) = 0;

  Rt_4x4(3, 0) = std::get<1>(Rt)(0);
  Rt_4x4(3, 1) = std::get<1>(Rt)(1);
  Rt_4x4(3, 2) = std::get<1>(Rt)(2);
  Rt_4x4(3, 3) = 1;

  // NOT IMPLEMENTED YET
  // [CODE] ...

  // 검증 과정에서 연산된 pointcloud가 여기서 publish됨
  // [YET] verification_publisher.publish();
}

// *** 체크보드의 월드좌표계를 저장하는 함수 *** //
void DepthChessboardPoseEstimator::create_world_coordinate_system()
{
  object_points.clear();

  for (int i = 0; i < chessboard_dimensions.height; i++)
  {
    for (int j = 0; j < chessboard_dimensions.width; j++)
    {
      cv::Point3f corner(i * (chessboard_edge_size), j * (chessboard_edge_size), 0.0f);
      pcl::PointXYZ corner_point(corner.x, corner.y, corner.z);
      object_points.push_back(corner);
      world_corners_messages.push_back(corner_point);
    }
  }
}

// *** 인코딩 방식을 상수화하는 함수 *** //
int DepthChessboardPoseEstimator::encoding_mat_type(const std::string & encoding)
{
  if (encoding == "mono8")
  {
    return CV_8UC1;
  }
  else if (encoding == "bgr8")
  {
    return CV_8UC3;
  }
  else if (encoding == "mono16")
  {
    return CV_16SC1;
  }
  else if (encoding == "rgba8")
  {
    return CV_8UC4;
  }
  else if (encoding == "bgra8")
  {
    return CV_8UC4;
  }
  else if (encoding == "32FC1")
  {
    return CV_32FC1;
  }
  else if (encoding == "rgb8")
  {
    return CV_8UC3;   // color의 image_raw 인코딩 형식
  }
  else if (encoding == "16UC1")
  {
    return CV_16UC1;  // depth의 image_raw 인코딩 형식
  }
  else {
    throw std::runtime_error("Unsupported encoding type");
  }
}

// *** sensor_msgs::PointCloud2에서 pcl::PointCloud<pcl::PointXYZ>로 변환하는 함수 *** //
pcl::PointCloud<pcl::PointXYZ> DepthChessboardPoseEstimator::sensor_to_pcl(sensor_msgs::PointCloud2 sensor_messages)
{
  pcl::PointCloud<pcl::PointXYZ> pcl_messages;
  pcl::fromROSMsg(sensor_messages, pcl_messages);
  return pcl_messages;
}

// *** pcl::PointCloud<pcl::PointXYZ>에서 sensor_msgs::PointCloud2로 변환하는 함수 *** //
sensor_msgs::PointCloud2 DepthChessboardPoseEstimator::pcl_to_sensor(pcl::PointCloud<pcl::PointXYZ> pcl_messages)
{
  sensor_msgs::PointCloud2 sensor_messages;
  pcl::toROSMsg(pcl_messages, sensor_messages);
  sensor_messages.header.frame_id = "base_link";
  return sensor_messages;
}