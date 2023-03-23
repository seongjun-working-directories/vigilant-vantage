#include "camera_pose_estimator/CameraPoseEstimator.hpp"

CameraPoseEstimator::CameraPoseEstimator()
{
  parameter_initializer();

  image_subscriber = node_handle.subscribe(camera_topic, 1, &CameraPoseEstimator::camera_callback, this);
  compressed_image_publisher_1 = node_handle.advertise<sensor_msgs::CompressedImage>(camera_image_01, 1);
  compressed_image_publisher_2 = node_handle.advertise<sensor_msgs::CompressedImage>(camera_image_02, 1);

  // [TEST] cv::namedWindow("default");
  // [TEST] cv::namedWindow("image");

  dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  board = cv::aruco::GridBoard::create(board_x, board_y, aruco_edge_size, aruco_gap_size, dictionary);

  /* [CANDIDATE]
  for (auto &obj_point: board->objPoints)
  {
    for (auto &obj_corner: obj_point)
    {
      // // [ALTERNATIVE]
      // float tmp_x = obj_corner.x;
      // float tmp_y = obj_corner.y;
      // // float tmp_z = obj_corner.z;

      // obj_corner.x = 1.36;
      // obj_corner.y = tmp_x - 0.45;
      // obj_corner.z = tmp_y + 0.8;


      obj_corner.x = obj_corner.x - 0.45;
      obj_corner.y = obj_corner.y + 0.8;
      obj_corner.z = 1.36;
    }
  }

  for (auto &obj_point: board->objPoints)
  {
    for (auto &obj_corner: obj_point)
    {
      std::cout << obj_corner.x << " " << obj_corner.y << " " << obj_corner.z << std::endl;
    }
  }
  */
}

CameraPoseEstimator::~CameraPoseEstimator()
{
  // [TEST] cv::destroyWindow("default");
  // [TEST] cv::destroyWindow("image");
}

void CameraPoseEstimator::camera_callback(const sensor_msgs::CompressedImage::ConstPtr& image_messages)
{
  // [TEST] std::cout << (image_messages->data).size() << std::endl;

  cv::Mat input_frame = cv::imdecode(cv::Mat(image_messages->data), 1);
  input_frame.copyTo(copied_image);
  cv::aruco::detectMarkers(copied_image, dictionary, corners, ids);
  // [TEST] cv::imshow("default", copied_image);

  std::vector<uchar> data;
  sensor_msgs::CompressedImage compressed_image;
  compressed_image.header.stamp = ros::Time::now();
  compressed_image.format = "jpeg";
  cv::imencode(".jpg", copied_image, data);
  compressed_image.data = data;
  compressed_image_publisher_1.publish(compressed_image);

  estimate_pose();
  cv::waitKey(frames_per_seconds);
}

void CameraPoseEstimator::estimate_pose()
{
  if (ids.size() > 0) {
    cv::aruco::drawDetectedMarkers(copied_image, corners, ids);
    cv::Vec3d rvec, tvec;
    int valid = cv::aruco::estimatePoseBoard(corners, ids, board, intrinsic_parameter, distortion_coefficient, rvec, tvec);

    if(valid > 0)
      cv::drawFrameAxes(copied_image, intrinsic_parameter, distortion_coefficient, rvec, tvec, 3.5);

    cv::Mat R;
    cv::Rodrigues(rvec,R);
    cv::Mat R_inv = R.inv();

    cv::Mat P = -R_inv * tvec;
    double*p = (double*)P.data;
    // [TEST] std::cout << "x = " << p[0] << ", y = " << p[1] << ", z = " << p[2] << std::endl;

    double x = 1.36 - p[2];
    double y = p[0] - 0.45;
    double z = p[1] + 0.8;

    cv::Vec3d euler_angles;
    get_eular_angles(R, euler_angles);

    double pitch   = euler_angles[0] * (M_PI/180);
    double yaw = euler_angles[1] * (M_PI/180) - 3.141;
    double roll  = euler_angles[2] * (M_PI/180);

    // [TEST] std::cout << "roll = " << roll << ", pitch = " << pitch << ", yaw = " << yaw << std::endl;

    vector_to_marker.str(std::string());
    vector_to_marker << std::setprecision(4)<< "x: " << std::setw(8) << x << " ";
    vector_to_marker << std::setprecision(4)<< "y: " << std::setw(8) << y << " ";
    vector_to_marker << std::setprecision(4)<< "z: " << std::setw(8) << z << " ";

    cv::putText(copied_image, vector_to_marker.str(),
    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 252, 124), 2, CV_AVX);

    // [TEST] cv::imshow("found", copied_image);

    std::vector<uchar> data;
    sensor_msgs::CompressedImage compressed_image;
    compressed_image.header.stamp = ros::Time::now();
    compressed_image.format = "jpeg";
    cv::imencode(".jpg", copied_image, data);
    compressed_image.data = data;

    compressed_image_publisher_2.publish(compressed_image);

    static tf::TransformBroadcaster tf_broadcaster;
    tf::Transform tf_transform;
    tf::Quaternion tf_quaternion;

    tf_transform.setOrigin(tf::Vector3(x, y, z));
    tf_quaternion.setRPY(roll, pitch, yaw);
    tf_transform.setRotation(tf_quaternion);

    tf_broadcaster.sendTransform(tf::StampedTransform(tf_transform, ros::Time::now(), "base_link", camera_tf));

    ids.clear();
    corners.clear();
  }
}

void CameraPoseEstimator::get_eular_angles(cv::Mat &rotation_camera_matrix, cv::Vec3d& euler_angles)
{
  cv::Mat camera_matrix, rotation_matrix, translation_vector, rotation_matrix_X, rotation_matrix_Y, rotation_matrix_Z;
  double* _r = rotation_camera_matrix.ptr<double>();
  double projection_matrix[12] = {
    _r[0],_r[1],_r[2],0,
    _r[3],_r[4],_r[5],0,
    _r[6],_r[7],_r[8],0
  };

  cv::decomposeProjectionMatrix(
    cv::Mat(3, 4, CV_64FC1, projection_matrix),
    camera_matrix, rotation_matrix, translation_vector,
    rotation_matrix_X, rotation_matrix_Y, rotation_matrix_Z,
    euler_angles
  );
}

void CameraPoseEstimator::parameter_initializer()
{
  node_handle.getParam("/camera_pose_estimator/CAMERA_TOPIC", camera_topic);
  
  node_handle.getParam("/camera_pose_estimator/CAMERA_IMAGE_01_TOPIC", camera_image_01);
  node_handle.getParam("/camera_pose_estimator/CAMERA_IMAGE_02_TOPIC", camera_image_02);
  
  node_handle.getParam("/camera_pose_estimator/CAMERA_TF_TOPIC", camera_tf);
  
  node_handle.getParam("/camera_pose_estimator/ARUCO_EDGE_SIZE", aruco_edge_size);
  node_handle.getParam("/camera_pose_estimator/ARUCO_GAP_SIZE", aruco_gap_size);
  node_handle.getParam("/camera_pose_estimator/ARUCO_BOARD_X", board_x);
  node_handle.getParam("/camera_pose_estimator/ARUCO_BOARD_Y", board_y);

  node_handle.getParam("/camera_pose_estimator/FRAMES_PER_SECONDS", frames_per_seconds);
}