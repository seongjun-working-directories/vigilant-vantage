#ifndef UTILITIES_HPP
#define UTILITIES_HPP

#include <ctime>
#include <cstdlib>
#include <chrono>
#include <cmath>
#include <pcl/point_types.h>

typedef pcl::PointXYZI PointType;

class Utilities
{
private:
  std::chrono::time_point<std::chrono::system_clock> start, end;

public:
  Utilities();
  ~Utilities();

  void tic();
  double toc();

  double rad2deg(double radians);
  double deg2rad(double degrees);
};

#endif