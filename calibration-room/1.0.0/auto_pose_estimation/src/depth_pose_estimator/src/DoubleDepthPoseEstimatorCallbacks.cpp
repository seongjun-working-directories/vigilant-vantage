#include "depth_pose_estimator/DoubleDepthPoseEstimator.hpp"

void DoubleDepthPoseEstimator::image_callback(const sensor_msgs::Image::ConstPtr& image_messages)
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

  // [TEST]
  cv::imshow("DEFAULT", copied_frame);

  aruco_marker_detection(copied_frame);

  int count = 0;
  for (auto &marker_corner : marker_corners)
  {
    // [TEST] std::cout << marker_ids[count] << ": ";
    // [TEST] std::cout << marker_corner.size() << std::endl;

    if (marker_corner.size() != 4)
    {
      // [TEST]
      std::cout << marker_ids[count] << "의 요소 수가 4개 미만이므로 관련 데이터를 삭제합니다." << std::endl;

      marker_ids.erase(marker_ids.begin() + count);
      marker_corners.erase(marker_corners.begin() + count);
    }

    count++;
  }

  world_corners_publisher.publish(pcl_to_sensor(world_corners_messages));
  cv::waitKey(1000/frames_per_seconds);
}

void DoubleDepthPoseEstimator::depth_callback(const sensor_msgs::PointCloud2::ConstPtr& depth_messages)
{
  if (!(marker_ids.size() > 0)) return;

  // /camera/depth(_registered)/points 메시지를 pcl::PointCloud<pcl::PointXYZ> 형태로 변환
  sensor_msgs::PointCloud2 depth_points = *depth_messages;
  pcl::PointCloud<pcl::PointXYZ> pcl_depth_points;
  pcl_depth_points = sensor_to_pcl(depth_points);

  std::vector<cv::Point3f> aruco_marker_information_vector;
  pcl::PointCloud<pcl::PointXYZ> camera_corners_messages;

  for(auto &id : marker_ids)
  {
    pcl::PointXYZ _corner_point_xyz_lt, _corner_point_xyz_rt, _corner_point_xyz_rb, _corner_point_xyz_lb;

    try {
      // IR 영상 및 COLOR 영상 간 좌표축 차이 반영
      /* [ORIGINAL]
      */
      _corner_point_xyz_lt = pcl_depth_points.at(marker_corners[id][0].x, marker_corners[id][0].y);
      _corner_point_xyz_rt = pcl_depth_points.at(marker_corners[id][1].x, marker_corners[id][1].y);
      _corner_point_xyz_rb = pcl_depth_points.at(marker_corners[id][2].x, marker_corners[id][2].y);
      _corner_point_xyz_lb = pcl_depth_points.at(marker_corners[id][3].x, marker_corners[id][3].y);
      
      /* [ALTERNATIVE]
      _corner_point_xyz_lt = pcl_depth_points.at(marker_corners[id][0].x - 3, marker_corners[id][0].y + 50);
      _corner_point_xyz_rt = pcl_depth_points.at(marker_corners[id][1].x - 3, marker_corners[id][1].y + 50);
      _corner_point_xyz_rb = pcl_depth_points.at(marker_corners[id][2].x - 3, marker_corners[id][2].y + 50);
      _corner_point_xyz_lb = pcl_depth_points.at(marker_corners[id][3].x - 3, marker_corners[id][3].y + 50);
      */
    }
    catch (int e) {
      continue;
    }

    // 오른손 법칙에 맞게 좌표축을 변경 내용 반영
    /* [ORIGINAL]
    cv::Point3f _corner_lt(_corner_point_xyz_lt.x, _corner_point_xyz_lt.y, _corner_point_xyz_lt.z);
    cv::Point3f _corner_rt(_corner_point_xyz_rt.x, _corner_point_xyz_rt.y, _corner_point_xyz_rt.z);
    cv::Point3f _corner_rb(_corner_point_xyz_rb.x, _corner_point_xyz_rb.y, _corner_point_xyz_rb.z);
    cv::Point3f _corner_lb(_corner_point_xyz_lb.x, _corner_point_xyz_lb.y, _corner_point_xyz_lb.z);
    */
    // [ALTERNATIVE]
    cv::Point3f _corner_lt(_corner_point_xyz_lt.z, _corner_point_xyz_lt.x * (-1), _corner_point_xyz_lt.y * (-1));
    cv::Point3f _corner_rt(_corner_point_xyz_rt.z, _corner_point_xyz_rt.x * (-1), _corner_point_xyz_rt.y * (-1));
    cv::Point3f _corner_rb(_corner_point_xyz_rb.z, _corner_point_xyz_rb.x * (-1), _corner_point_xyz_rb.y * (-1));
    cv::Point3f _corner_lb(_corner_point_xyz_lb.z, _corner_point_xyz_lb.x * (-1), _corner_point_xyz_lb.y * (-1));

    std::cout << "LT & LB" << std::endl;
    std::cout << _corner_lt.x << " " << _corner_lt.y << " " << _corner_lt.z << std::endl;
    std::cout << _corner_lb.x << " " << _corner_lb.y << " " << _corner_lb.z << std::endl;


    aruco_marker_information_vector.push_back(_corner_lt);
    aruco_marker_information_vector.push_back(_corner_rt);
    aruco_marker_information_vector.push_back(_corner_rb);
    aruco_marker_information_vector.push_back(_corner_lb);

    camera_corners_messages.push_back(_corner_point_xyz_lt);
    camera_corners_messages.push_back(_corner_point_xyz_rt);
    camera_corners_messages.push_back(_corner_point_xyz_rb);
    camera_corners_messages.push_back(_corner_point_xyz_lb);

    /* [TEST]
    std::cout << _corner_lt.x << " " << _corner_lt.y << " " << _corner_lt.z << std::endl;
    std::cout << _corner_rt.x << " " << _corner_rt.y << " " << _corner_rt.z << std::endl;
    std::cout << _corner_rb.x << " " << _corner_rb.y << " " << _corner_rb.z << std::endl;
    std::cout << _corner_lb.x << " " << _corner_lb.y << " " << _corner_lb.z << std::endl;
    */
  }

  std::tuple<Eigen::Matrix3f, Eigen::Vector3f> Rt = rigid_transformation(aruco_marker_information_vector);  // [CORE METHOD]

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
    // [TEST]
    std::cout << "[" << ros::this_node::getName() << "] x, y, z >>> " << x << ", " << y << ", " << z << std::endl;
    // [TEST]
    std::cout << "[" << ros::this_node::getName() << "] roll, pitch, yaw >>> " << roll << ", " << pitch << ", " << yaw << std::endl;
    
    static tf::TransformBroadcaster tf_broadcaster;
    tf::Transform tf_transform;
    tf::Quaternion tf_quaternion;

    tf_transform.setOrigin(tf::Vector3(x, y, z));
    tf_quaternion.setRPY(roll, pitch, yaw);
    tf_transform.setRotation(tf_quaternion);
    tf_broadcaster.sendTransform(tf::StampedTransform(tf_transform, ros::Time::now(), "base_link", depth_tf));
  }

  camera_corners_publisher.publish(pcl_to_sensor(camera_corners_messages));
}