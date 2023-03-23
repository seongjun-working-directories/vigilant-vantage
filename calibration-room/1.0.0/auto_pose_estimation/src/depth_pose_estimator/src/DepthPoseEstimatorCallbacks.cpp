#include "depth_pose_estimator/DepthPoseEstimator.hpp"

void DepthPoseEstimator::image_callback(const sensor_msgs::Image::ConstPtr& image_messages)
{
  cv::Mat frame(
    image_messages->height - 80,  // 480px - 80px
    image_messages->width,        // 640px
    encoding_mat_type(image_messages->encoding),
    const_cast<unsigned char*>(image_messages->data.data()),
    image_messages->step
  );

  cv::Mat copied_frame;
  if (image_messages->encoding == "rgb8")
  {
    cv::Mat temporary_frame;
    cv::cvtColor(frame, temporary_frame, cv::COLOR_RGB2BGR);
    copied_frame = temporary_frame;
  }
  else
  {
    copied_frame = frame;
  }

  std::vector<uchar> data;
  sensor_msgs::CompressedImage compressed_image;
  compressed_image.header.stamp = ros::Time::now();
  compressed_image.format = "jpeg";
  cv::imencode(".jpg", copied_frame, data);
  compressed_image.data = data;
  compressed_image_publisher_1.publish(compressed_image);

  // [TEST] cv::imshow("DEFAULT", copied_frame);

  chessboard_detection(copied_frame);

  world_corners_publisher.publish(pcl_to_sensor(world_corners_messages));
  cv::waitKey(1000/frames_per_seconds);
}

void DepthPoseEstimator::depth_callback(const sensor_msgs::PointCloud2::ConstPtr& depth_messages)
{
  if (detected_corners.size() != (chessboard_dimensions.width * chessboard_dimensions.height)) return;

  // /camera/depth(_registered)/points 메시지를 pcl::PointCloud<pcl::PointXYZ> 형태로 변환
  sensor_msgs::PointCloud2 depth_points = *depth_messages;
  pcl::PointCloud<pcl::PointXYZ> pcl_depth_points;
  pcl_depth_points = sensor_to_pcl(depth_points);

  std::vector<cv::Point3f> chessboard_information_vector;
  pcl::PointCloud<pcl::PointXYZ> camera_corners_messages;

  for (auto &detected_corner : detected_corners)
  {
    // IR 영상 및 COLOR 영상 간 좌표축 차이 반영
    // [ORIGINAL]
    pcl::PointXYZ _corner_point_xyz = pcl_depth_points.at(detected_corner.x, detected_corner.y);
    // [ALTERNATIVE] pcl::PointXYZ _corner_point_xyz = pcl_depth_points.at(detected_corner.x - 3, detected_corner.y + 50);

    // 오른손 법칙에 맞게 좌표축을 변경 내용 반영
    // [ORIGINAL] cv::Point3f _corner(_corner_point_xyz.x, _corner_point_xyz.y, _corner_point_xyz.z);
    // [ALTERNATIVE]
    cv::Point3f _corner(_corner_point_xyz.z, _corner_point_xyz.x * (-1), _corner_point_xyz.y * (-1));


    chessboard_information_vector.push_back(_corner);
    camera_corners_messages.push_back(_corner_point_xyz);

    // [TEST] std::cout << _corner.x << " " << _corner.y << " " << _corner.z << std::endl;
  }

  std::tuple<Eigen::Matrix3f, Eigen::Vector3f> Rt = rigid_transformation(chessboard_information_vector);  // [CORE METHOD]

  Eigen::Matrix3f eigen_rotation_matrix = std::get<0>(Rt);
  Eigen::Vector3f eigen_translation_vector = std::get<1>(Rt);

  cv::eigen2cv(eigen_rotation_matrix, rotation_vector);
  cv::eigen2cv(eigen_translation_vector, translation_vector);

  double x, y, z, roll, pitch, yaw;
  x = eigen_translation_vector(0);
  y = eigen_translation_vector(1);
  z = eigen_translation_vector(2);
  roll = std::atan2(eigen_rotation_matrix(2, 1), eigen_rotation_matrix(2, 2));
  pitch = std::atan2(
    -eigen_rotation_matrix(2, 0),
    std::sqrt(eigen_rotation_matrix(2, 1) * eigen_rotation_matrix(2, 1) + eigen_rotation_matrix(2, 2) * eigen_rotation_matrix(2, 2))
  );
  yaw = std::atan2(eigen_rotation_matrix(1, 0), eigen_rotation_matrix(0, 0));

  if (!(std::isnan(x) + std::isnan(y) + std::isnan(z)))
  {
    // [TEST] std::cout <<"x, y, z >>> " << x << ", " << y << ", " << z << std::endl;
    // [TEST] std::cout <<"roll, pitch, yaw >>> " << roll << ", " << pitch << ", " << yaw << std::endl;
    
    static tf::TransformBroadcaster tf_broadcaster;
    tf::Transform tf_transform;
    tf::Quaternion tf_quaternion;

    tf_transform.setOrigin(tf::Vector3(x, y, z));
    tf_quaternion.setRPY(roll, pitch, yaw);
    tf_transform.setRotation(tf_quaternion);
    tf_broadcaster.sendTransform(tf::StampedTransform(tf_transform, ros::Time::now(), "base_link", depth_tf));
  }

  // pose_verification(Rt);

  camera_corners_publisher.publish(pcl_to_sensor(camera_corners_messages));
}