#include "depth_pose_estimator/DepthPoseEstimator.hpp"

// *** 생성자 함수 *** //
DepthPoseEstimator::DepthPoseEstimator(
  ros::NodeHandle &node_handle, float _marker_size, float _marker_seperation_size,
  int _arucoboard_width, int _arucoboard_height, int _frames_per_seconds
) {
  // subscriber
  image_subscriber = image_transport::ImageTransport(node_handle).subscribe("/camera/color/image_raw", 1, &DepthPoseEstimator::image_callback, this);
  depth_subscriber = node_handle.subscribe("/camera/depth_registered/points", 1, &DepthPoseEstimator::depth_callback, this);

  // publisher
  world_corners_publisher = node_handle.advertise<sensor_msgs::PointCloud2>("world_corners_messages", 1);
  camera_corners_publisher = node_handle.advertise<sensor_msgs::PointCloud2>("camera_corners_messages", 1);
  verification_publisher = node_handle.advertise<sensor_msgs::PointCloud2>("verification_messages", 1);

  // image publisher
  compressed_image_publisher_1 = node_handle.advertise<sensor_msgs::CompressedImage>("depth_image_message_1", 1);
  compressed_image_publisher_2 = node_handle.advertise<sensor_msgs::CompressedImage>("depth_image_message_2", 1);

  // 초당 프레임
  frames_per_seconds = _frames_per_seconds;

  // arucoboard 관련 변수 1
  marker_size = _marker_size;
  marker_seperation_size = _marker_seperation_size;
  arucoboard_dimensions = cv::Size(_arucoboard_width, _arucoboard_height);  // y x 순

  // arucoboard 관련 변수 2
  dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  board = cv::aruco::GridBoard::create(
    // y x 순
    _arucoboard_width, _arucoboard_height, _marker_size, _marker_seperation_size, dictionary
  );
  params = cv::aruco::DetectorParameters::create();

  // 기준 월드좌표계 코너값
  create_world_coordinate_system();

  // 내부 파라미터값 1
  double _intrinsic_parameters[] = {453.7543640, 0.0, 323.54632568, 0.0, 453.7543640, 241.9261780, 0.0, 0.0, 1.0};
  intrinsic_parameters = _intrinsic_parameters;
  double _radial_tangential_distortion[] = {0.0, 0.0, 0.0, 0.0};
  radial_tangential_distortion = _radial_tangential_distortion;

  // 내부 파라미터값 2
  camera_matrix = cv::Mat(3, 3, CV_64FC1, intrinsic_parameters);
  distortion_coefficients = cv::Mat(4, 1, CV_64FC1, radial_tangential_distortion);

  // 이미지 화면
  cv::namedWindow("default");
  cv::namedWindow("found");
}

// *** 소멸자 함수 *** //
DepthPoseEstimator::~DepthPoseEstimator()
{
  cv::destroyWindow("default");
  cv::destroyWindow("found");
}

// *** Arucoboard를 찾아내는 함수 *** //
void DepthPoseEstimator::arucoboard_detection(cv::Mat &copied_frame)
{
  cv::Mat frame;
  copied_frame.copyTo(frame);

  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;
  // [CANDIDATE] std::vector<std::vector<cv::Point2f>> rejected_candidates;
  cv::aruco::detectMarkers(frame, board->dictionary, marker_corners, marker_ids);
  // [CANDIDATE] cv::aruco::refineDetectedMarkers(copied_frame, board, marker_corners, marker_ids, rejected_candidates);

  // marker가 4개 이상 발견된 경우
  if (marker_ids.size() > 3)
  {
    cv::aruco::drawDetectedMarkers(copied_frame, marker_corners, marker_ids);
    cv::Vec3d rvec, tvec;
    int valid = cv::aruco::estimatePoseBoard(marker_corners, marker_ids, board, camera_matrix, distortion_coefficients, rvec, tvec);

    if (valid > 0)
    {
      cv::drawFrameAxes(copied_frame, camera_matrix, distortion_coefficients, rvec, tvec, 0.1);

      int count = 0;
      std::vector<std::tuple<int, std::vector<cv::Point2f>>> _detected_markers;
      for (auto &marker : marker_corners)
      {
        std::tuple<int, std::vector<cv::Point2f>> marker_information;
        if (marker.size() == 4)
        {
          marker_information = std::make_tuple(marker_ids[count], marker);
          _detected_markers.push_back(marker_information);
        }
        count++;
      }

      detected_markers = _detected_markers;
    }

    cv::imshow("found", copied_frame);
    std::vector<uchar> data;
    sensor_msgs::CompressedImage compressed_image;
    compressed_image.header.stamp = ros::Time::now();
    compressed_image.format = "jpeg";
    cv::imencode(".jpg", copied_frame, data);
    compressed_image.data = data;
    compressed_image_publisher_1.publish(compressed_image);
  }
  else
  {
    detected_markers.clear();
    cv::imshow("default", copied_frame);
    std::vector<uchar> data;
    sensor_msgs::CompressedImage compressed_image;
    compressed_image.header.stamp = ros::Time::now();
    compressed_image.format = "jpeg";
    cv::imencode(".jpg", copied_frame, data);
    compressed_image.data = data;
    compressed_image_publisher_2.publish(compressed_image);
  }
}

// *** rigid_transformation 함수 *** //
std::tuple<Eigen::Matrix3f, Eigen::Vector3f> DepthPoseEstimator::rigid_transformation(
    std::vector<std::tuple<int, std::vector<cv::Point3f>>> marker_information_vector
) {
  Eigen::Matrix<float, 3, Eigen::Dynamic> camera_coordinate_points, world_coordinate_points;
  Eigen::Matrix<float, 3, Eigen::Dynamic> camera_substract_mean, world_substract_mean;

  int marker_information_vector_size = marker_information_vector.size();
  // [TEST]
  std::cout << marker_information_vector_size << std::endl;

  camera_coordinate_points.resize(3, marker_information_vector_size * 4);
  world_coordinate_points.resize(3, marker_information_vector_size * 4);
  camera_substract_mean.resize(3, marker_information_vector_size * 4);
  world_substract_mean.resize(3, marker_information_vector_size * 4);

  int count = 0;
  for (auto &marker_information : marker_information_vector)
  {
    // [TEST] std::cout << "===== camera_coordinate_points recording started =====" << std::endl;
    // std::cout << "ID :: " << std::get<0>(marker_information) << std::endl;
    for (auto &marker_corners : std::get<1>(marker_information))
    {
      camera_coordinate_points(0, count) = marker_corners.x;
      camera_coordinate_points(1, count) = marker_corners.y;
      camera_coordinate_points(2, count) = marker_corners.z;

      // [TEST] std::cout << marker_corners.x << " " << marker_corners.y << " " << marker_corners.z << std::endl;
      count++;
    }
    // [TEST] std::cout << "===== camera_coordinate_points recording started =====\n" << std::endl;
  }
  // [TEST] std::cout << (count == marker_information_vector.size() * 4) << std::endl;

  count = 0;
  for (auto &marker_information : marker_information_vector)
  {
    int index = std::get<0>(marker_information);

    world_coordinate_points(0, count) = object_points[index].x;
    world_coordinate_points(1, count) = object_points[index].y;
    world_coordinate_points(2, count) = +0.0f;

    world_coordinate_points(0, count+1) = object_points[index].x;
    world_coordinate_points(1, count+1) = object_points[index].y + marker_size;
    world_coordinate_points(2, count+1) = +0.0f;

    world_coordinate_points(0, count+2) = object_points[index].x + marker_size;
    world_coordinate_points(0, count+2) = object_points[index].y + marker_size;
    world_coordinate_points(0, count+2) = +0.0f;

    world_coordinate_points(0, count+3) = object_points[index].x + marker_size;
    world_coordinate_points(0, count+3) = object_points[index].y;
    world_coordinate_points(0, count+3) = +0.0f;

    count += 4;
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

// *** 검출된 점들로부터 평면을 도출하는 함수 *** //
Eigen::Vector4f DepthPoseEstimator::plane_detection(pcl::PointCloud<pcl::PointXYZ> data)
{
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;
  pcl::SACSegmentation<pcl::PointXYZ> segmentation;

  segmentation.setOptimizeCoefficients(true);
  segmentation.setModelType(pcl::SACMODEL_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setDistanceThreshold(0.01);
  segmentation.setInputCloud(data.makeShared());
  segmentation.segment(inliers, coefficients);

  /* [TEST]
  std::cout << "DATA SIZE" << data.size() << std::endl;
  std::cout << "INLIER SIZE" << inliers.indices.size() << std::endl;

  std::cout << "\nMODEL COEFFICIENTS>>> " << std::endl;
  for (auto &coefficient : coefficients.values)
  {
    std::cout << coefficient << std::endl;
  }
  */

  return Eigen::Vector4f(coefficients.values[0], coefficients.values[1], coefficients.values[2], coefficients.values[3]);
}

// *** 평면으로부터 점까지의 거리가 일정 범위를 넘어서는지 검사하는 함수 *** //
bool DepthPoseEstimator::distance_from_plane(Eigen::Vector4f _plane_description, std::vector<cv::Point3f> _points)
{
  double a = _plane_description(0);
  double b = _plane_description(1);
  double c = _plane_description(2);
  double d = _plane_description(3);

  for (auto &point : _points)
  {
    double distance = std::abs(a*point.x + b*point.y + c*point.z + d) / std::sqrt(std::pow(a, 2) + std::pow(b, 2) + std::pow(c, 2));
    // [TEST] std::cout << distance << std::endl;

    if(distance > 0.015) {
      return false;
    }
  }

  return true;
}

// *** 계산된 Rt 값을 기반으로 verification 하는 함수 *** //
void DepthPoseEstimator::pose_verification(std::tuple<Eigen::Matrix3f, Eigen::Vector3f> Rt)
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

// *** Arucoboard의 월드좌표계를 저장하는 함수 *** //
void DepthPoseEstimator::create_world_coordinate_system()
{
  object_points.clear();

  for (int i = 0; i < arucoboard_dimensions.height; i++)  // x
  {
    for (int j = 0; j < arucoboard_dimensions.width; j++) // y
    {
      cv::Point3f top_left_corner(i*(marker_size+marker_seperation_size), j*(marker_size+marker_seperation_size), 0.0f);
      pcl::PointXYZ top_left_corner_point(top_left_corner.x, top_left_corner.y, top_left_corner.z);
      object_points.push_back(top_left_corner);
      world_corners_messages.push_back(top_left_corner_point);
    }
  }
}

// *** 인코딩 방식을 상수화하는 함수 *** //
int DepthPoseEstimator::encoding_mat_type(const std::string & encoding)
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
pcl::PointCloud<pcl::PointXYZ> DepthPoseEstimator::sensor_to_pcl(sensor_msgs::PointCloud2 sensor_messages)
{
  pcl::PointCloud<pcl::PointXYZ> pcl_messages;
  pcl::fromROSMsg(sensor_messages, pcl_messages);
  return pcl_messages;
}

// *** pcl::PointCloud<pcl::PointXYZ>에서 sensor_msgs::PointCloud2로 변환하는 함수 *** //
sensor_msgs::PointCloud2 DepthPoseEstimator::pcl_to_sensor(pcl::PointCloud<pcl::PointXYZ> pcl_messages)
{
  sensor_msgs::PointCloud2 sensor_messages;
  pcl::toROSMsg(pcl_messages, sensor_messages);
  sensor_messages.header.frame_id = "base_link";
  return sensor_messages;
}
