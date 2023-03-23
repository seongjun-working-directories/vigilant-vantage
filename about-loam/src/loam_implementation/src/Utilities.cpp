#include "loam_implementation/Utilities.hpp"

Utilities::Utilities()
{
  // YET
}

Utilities::~Utilities()
{
  // YET
}

void Utilities::tic()
{
  start = std::chrono::system_clock::now();
}

double Utilities::toc()
{
  end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  return elapsed_seconds.count() * 1000;
}

inline double Utilities::rad2deg(double radians)
{
  return radians * 180.0 / M_PI;
}

inline double Utilities::deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}