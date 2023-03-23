#include "camera_pose_estimator/CameraPoseEstimator.hpp"

CameraPoseEstimator::CameraPoseEstimator()
{
  parameter_initializer();

  image_subscriber = node_handle.subscribe(camera_topic, 1, &CameraPoseEstimator::camera_callback, this);
  compressed_image_publisher_1 = node_handle.advertise<sensor_msgs::CompressedImage>("camera_image_message_1", 1);
  compressed_image_publisher_2 = node_handle.advertise<sensor_msgs::CompressedImage>("camera_image_message_2", 1);

  cv::namedWindow("default");
  cv::namedWindow("image");
}

CameraPoseEstimator::~CameraPoseEstimator()
{
  cv::destroyWindow("default");
  cv::destroyWindow("image");
}

void CameraPoseEstimator::camera_callback(const sensor_msgs::CompressedImage::ConstPtr& image_messages)
{
  // [TEST] std::cout << (image_messages->data).size() << std::endl;

  cv::Mat input_frame = cv::imdecode(cv::Mat(image_messages->data), 1);
  input_frame.copyTo(copied_image);
  cv::aruco::detectMarkers(copied_image, dictionary, corners, ids);
  cv::imshow("default", copied_image);

  std::vector<uchar> data;
  sensor_msgs::CompressedImage compressed_image;
  compressed_image.header.stamp = ros::Time::now();
  compressed_image.format = "jpeg";
  cv::imencode(".jpg", copied_image, data);
  compressed_image.data = data;
  compressed_image_publisher_1.publish(compressed_image);

  estimate_pose();
  cv::waitKey(1);
}

void CameraPoseEstimator::estimate_pose()
{
  if (ids.size() > 0) {
    cv::aruco::drawDetectedMarkers(copied_image, corners, ids);
    cv::Vec3d rvec, tvec;
    int valid = cv::aruco::estimatePoseBoard(corners, ids, board, intrinsic_parameter, distortion_coefficient, rvec, tvec);

    if(valid > 0)
      cv::drawFrameAxes(copied_image, intrinsic_parameter, distortion_coefficient, rvec, tvec, 3.5);


    std::cout << "Translation = " << tvec[0] << std::endl;
    std::cout << "Rotation = " << rvec[0] << std::endl;

    cv::Mat R;
    cv::Rodrigues(rvec,R);
    cv::Mat R_inv = R.inv();

    cv::Mat P = -R_inv * tvec;
    double*p = (double*)P.data;
    std::cout << "x = " << p[0] << ", y = " << p[1] << ", z = " << p[2] << std::endl;

    cv::Vec3d euler_angles;
    get_eular_angles(R, euler_angles);

    double yaw   = euler_angles[0];
    double pitch = euler_angles[1];
    double roll  = euler_angles[2];

    std::cout << "roll = " << roll << ", pitch = " << pitch << ", yaw = " << yaw << std::endl;

    vector_to_marker.str(std::string());
    vector_to_marker << std::setprecision(4)<< "x: " << std::setw(8) << p[0];
    vector_to_marker << std::setprecision(4)<< "y: " << std::setw(8) << p[1];
    vector_to_marker << std::setprecision(4)<< "z: " << std::setw(8) << p[2];

    cv::putText(copied_image, vector_to_marker.str(),
    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 252, 124), 2, CV_AVX);

    cv::imshow("image", copied_image);

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

    tf_transform.setOrigin(tf::Vector3(p[0], p[1], p[2]));
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
  node_handle.getParam("/camera_pose_estimator/CAMERA_TF", camera_tf);
}