#include "depth_pose_estimator/DepthPoseEstimator.hpp"

// *** /camera/color/image_raw에 대한 콜백 함수 *** //
void DepthPoseEstimator::image_callback(const sensor_msgs::Image::ConstPtr &image_messages)
{
  cv::Mat frame(
    (image_messages->height) - 80,  // 480px - 80px
    image_messages->width,          // 640px
    encoding_mat_type(image_messages->encoding),
    const_cast<unsigned char*>(image_messages->data.data()),
    image_messages->step
  );

  cv::Mat copied_frame;
  if (image_messages->encoding == "rgb8")
  {
    cv::Mat temporary_frame;
    cv::cvtColor(frame, temporary_frame, cv::COLOR_RGB2BGR);
    temporary_frame.copyTo(copied_frame);
  }
  else
  {
    frame.copyTo(copied_frame);
  }

  arucoboard_detection(copied_frame);

  // [ALTERNATIVE] cv::waitKey(1000/frames_per_seconds);
  // [ALTERNATIVE]
  cv::waitKey(10);
}

// *** /camera/depth/points에 대한 콜백 함수 *** //
void DepthPoseEstimator::depth_callback(const sensor_msgs::PointCloud2::ConstPtr &depth_messages)
{
  if (detected_markers.size() < 4) return;

  // /camera/depth(_registered)/points 메시지를 pcl::PointCloud<pcl::PointXYZ> 형태로 변환
  sensor_msgs::PointCloud2 depth_points = *depth_messages;
  pcl::PointCloud<pcl::PointXYZ> pcl_depth_points;
  pcl_depth_points = sensor_to_pcl(depth_points);

  std::vector<std::tuple<int, std::vector<cv::Point3f>>> marker_information_vector;
  pcl::PointCloud<pcl::PointXYZ> camera_corners_messages;

  // [TEST] ROS_INFO("POINT RECORDER START");
  for (auto &detected_marker : detected_markers)
  {
    int index = std::get<0>(detected_marker);
    std::vector<cv::Point2f> detected_marker_corners = std::get<1>(detected_marker);
    std::tuple<int, std::vector<cv::Point3f>> processed_marker;

    bool hasNan = false;
    std::vector<cv::Point3f> processed_3d_corners;
    for (auto &detected_marker_corner : detected_marker_corners)
    {
      // [ALTERNATIVE] pcl::PointXYZ _corner_point_xyz = pcl_depth_points.at(detected_marker_corner.x - 3, detected_marker_corner.y + 50);
      pcl::PointXYZ _corner_point_xyz = pcl_depth_points.at(detected_marker_corner.x, detected_marker_corner.y);
      camera_corners_messages.push_back(_corner_point_xyz);
      cv::Point3f _corner_point_3f(_corner_point_xyz.x, _corner_point_xyz.y, _corner_point_xyz.z);
      processed_3d_corners.push_back(_corner_point_3f);

      if (std::isnan(_corner_point_3f.x) + std::isnan(_corner_point_3f.y) + std::isnan(_corner_point_3f.z) != 0)
      {
        hasNan = true;
      }
    }

    if (!hasNan)
    {
      processed_marker = std::make_tuple(index, processed_3d_corners);
      marker_information_vector.push_back(processed_marker);
    }
  }
  // [TEST] ROS_INFO("POINT RECORDER END");

  if (marker_information_vector.size() > 0)
  {
    Eigen::Vector4f plane_description = plane_detection(camera_corners_messages);

    std::vector<std::tuple<int, std::vector<cv::Point3f>>> outlier_eliminated_marker_information_vector;

    for (auto &marker_information : marker_information_vector)
    {
      if (distance_from_plane(plane_description, std::get<1>(marker_information)))
      {
        outlier_eliminated_marker_information_vector.push_back(marker_information);
      }
      else
      {
        // [TEST] std::cout << "ID `" << std::get<0>(marker_information) << "` 의 마커가 등록되지 않았습니다." << std::endl;
      }
    }

    // [CORE METHOD]
    std::tuple<Eigen::Matrix3f, Eigen::Vector3f> Rt = rigid_transformation(outlier_eliminated_marker_information_vector);

    Eigen::Matrix3f eigen_rotation_matrix = std::get<0>(Rt);
    Eigen::Vector3f eigen_translation_vector = std::get<1>(Rt);

    cv::eigen2cv(eigen_rotation_matrix, rotation_vector);
    cv::eigen2cv(eigen_translation_vector, translation_vector);

    // [TEST] std::cout << "ROTATION MATRIX >>> " << eigen_rotation_matrix << std::endl;
    // [TEST] std::cout << "TRANSLATION VECTOR >>> " << eigen_translation_vector << std::endl;

    double x, y, z;
    x = round(eigen_translation_vector(0) * 1000) / 1000;
    y = round(eigen_translation_vector(1) * 1000) / 1000;
    z = round(eigen_translation_vector(2) * 1000) / 1000;

    double roll, pitch, yaw;
    roll = std::atan2(eigen_rotation_matrix(2, 1), eigen_rotation_matrix(2, 2));
    pitch = std::atan2(
      -eigen_rotation_matrix(2, 0),
      std::sqrt(
        eigen_rotation_matrix(2, 1)*eigen_rotation_matrix(2, 1) + eigen_rotation_matrix(2, 2)*eigen_rotation_matrix(2, 2)
      )
    );
    yaw = std::atan2(eigen_rotation_matrix(1, 0), eigen_rotation_matrix(0, 0));

    if (
      (std::isnan(x) + std::isnan(y) + std::isnan(z) + std::isnan(roll) + std::isnan(pitch) + std::isnan(yaw) == 0)
      && ( x > 0.0001 || x < -0.0001) && ( y > 0.0001 || y < -0.0001) && ( z > 0.0001 || z < -0.0001)
      && ( x < 100 && x > -100) && (y < 100 && y > -100) && (z < 100 && z > -100)
    ) {
      /* [TEST]
      */
      std::cout << "x, y, z >>> " << x << ", " << y << ", " << z << std::endl;
      std::cout << "roll, pitch, yaw >>> " << roll << ", " << pitch << ", " << yaw << std::endl;

      static tf::TransformBroadcaster tf_broadcaster;
      tf::Transform tf_transform;
      tf::Quaternion tf_quaternion;

      tf_transform.setOrigin(tf::Vector3(x, y, z));
      tf_quaternion.setRPY(roll, pitch, yaw);
      tf_transform.setRotation(tf_quaternion);
      tf_broadcaster.sendTransform(tf::StampedTransform(tf_transform, ros::Time::now(), "base_link", "depth_tf"));
    }

    // pose_verification(Rt);
  }


  world_corners_publisher.publish(pcl_to_sensor(world_corners_messages));
  camera_corners_publisher.publish(pcl_to_sensor(camera_corners_messages));

  /* [TEST]
  Eigen::Vector4f plane_vector = plane_detection(camera_corners_messages);
  std::cout << plane_vector(0) << "x + " << plane_vector(1) << "y + "
    << plane_vector(2) << "z + " << plane_vector(3) << std::endl;
  */

  // [ALTERNATIVE] cv::waitKey(1000/frames_per_seconds);
  // [ALTERNATIVE]
  cv::waitKey(10);
}