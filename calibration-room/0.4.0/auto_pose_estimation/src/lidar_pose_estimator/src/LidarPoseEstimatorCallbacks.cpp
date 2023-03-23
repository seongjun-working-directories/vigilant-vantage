#include "lidar_pose_estimator/LidarPoseEstimator.hpp"

void LidarPoseEstimator::laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_messages)
{
  cv::Point2f poi;
  cv::Point2f poi_;

  angle_increment = scan_messages->angle_increment;

  if (is_reference_mode)
  {
    ref1.clear();

    for (int i=0; i<scan_messages->ranges.size(); i++)
    {
      value = scan_messages->ranges[i];
      angle = angle_min + angle_increment * i;
      poi_.x = value * cos(angle)*1000.0f;
      poi_.y = value * sin(angle)*1000.0f;

      if (std::isinf(poi_.x) || std::isinf(poi_.y)) continue;
      ref1.push_back(poi_);
    }

    // lidar reference data를 텍스트파일로 저장

    std::string temporary_filename = lidar_reference_topic;
    temporary_filename.erase(temporary_filename.begin());
    std::string filename = temporary_filename + ".txt";
    // [RESULT] `lidar_front_reference.txt` or `lidar_rear_reference.txt`
    std::ofstream write_file(filename.data());
    if (write_file.is_open())
    {
      for (auto &element : ref1)
      {
        write_file << element.x << " " << element.y << "\n";
      }
      write_file.close();
    }


    // [TEST] std::cout << "REFERENCE COMPLETE" <<std::endl;

    ref1.clear();
    is_reference_mode = false;
  }

  if (is_current_mode && (ref1.size() != 0))
  {
    current.clear();

    for (int i = 0; i < scan_messages->ranges.size(); i++)
    {
      value = scan_messages->ranges[i];
      angle = angle_min + angle_increment * i;
      poi.x = value * cos(angle)*1000.0f;
      poi.y = value * sin(angle)*1000.0f;

      if (std::isinf(poi.x) || std::isinf(poi.y)) continue;
      current.push_back(poi);
    }

    lidar_check();
    std::cout <<"CURRENT COMPLETE" <<std::endl;

    is_current_mode = false;
    ref1.clear();
  }
}

void LidarPoseEstimator::reference_callback(const std_msgs::Bool::ConstPtr &reference_bool)
{
  // [TEST]
  ROS_INFO("start: [%d]", reference_bool->data);

  is_reference_mode = reference_bool->data;
  if (reference_bool->data == true) is_current_mode = false;
}

void LidarPoseEstimator::current_callback(const std_msgs::Bool::ConstPtr &current_bool)
{
  // [TEST]
  ROS_INFO("check: [%d]", current_bool->data);

  is_current_mode = current_bool->data;
  ref1.clear();

  std::string temporary_filename = lidar_reference_topic;
  temporary_filename.erase(temporary_filename.begin());
  std::string filename = temporary_filename + ".txt";
  std::ifstream open_file(filename.data());
  if (open_file.is_open())
  {
    std::string line;
    while(std::getline(open_file, line))
    {
      std::stringstream sstream(line);
      std::string word;

      std::vector<double> x_and_y;
      while (std::getline(sstream, word, ' '))
      {
        x_and_y.push_back(std::stod(word));
      }

      ref1.push_back(cv::Point2f(x_and_y[0], x_and_y[1]));
    }
    open_file.close();
  }

  /* [TEST]
  for (auto &element : ref1)
  {
    std::cout << element.x << " " << element.y << std::endl;
  }
  */
}