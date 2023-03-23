#include "depth_center_finder/DepthCenterFinder.hpp"

// *** 생성자 함수 *** //
DepthCenterFinder::DepthCenterFinder(ros::NodeHandle& node_handle, double _box_width, double _box_height)
{
  color_image_subscriber = image_transport::ImageTransport(node_handle).subscribe("/camera/color/image_raw", 1, &DepthCenterFinder::color_image_callback, this);
  ir_image_subscriber = image_transport::ImageTransport(node_handle).subscribe("/camera/ir/image_raw", 1, &DepthCenterFinder::ir_image_callback, this);
  store_image_command_subscriber = node_handle.subscribe("/store_image", 1, &DepthCenterFinder::store_image_command_callback, this);
  run_finder_command_subscriber = node_handle.subscribe("/process_image", 1, &DepthCenterFinder::run_finder_command_callback, this);

  store_image_command_color = false;
  store_image_command_ir = false;

  box_width = box_width;
  box_height = box_height;

  cv::namedWindow("default color");
  cv::namedWindow("square detected color");
  cv::namedWindow("color");

  cv::namedWindow("default ir");
  cv::namedWindow("square detected ir");
  cv::namedWindow("ir");
}

// *** 소멸자 함수 *** //
DepthCenterFinder::~DepthCenterFinder()
{
  cv::destroyWindow("default color");
  cv::destroyWindow("square detected color");
  cv::destroyWindow("color");

  cv::destroyWindow("default ir");
  cv::destroyWindow("sqaure detected ir");
  cv::destroyWindow("ir");
}

// *** color 이미지에서 사각형을 찾아 각 선분을 vector로 반환하는 함수 *** //
std::vector<std::vector<cv::Point>> DepthCenterFinder::color_square_detection(cv::Mat image)
{
  std::vector<std::vector<cv::Point>> squares, contours;

  const int threshold = 50;
  const int max_threshold_level = 2;

  cv::Mat copied_image, temporary_image, processed_image, result_image;
  image.copyTo(copied_image);

  cv::dilate(copied_image, copied_image, cv::Mat(), cv::Point(-1, -1));
  cv::medianBlur(copied_image, copied_image, 5);

  cv::pyrDown(copied_image, temporary_image, cv::Size(copied_image.cols/2, copied_image.rows/2));
  cv::pyrUp(temporary_image, processed_image, copied_image.size());

  // [TEST] std::cout << "PROCESSED_IMAGE BEFORE >>> " << processed_image.channels() << std::endl; // 3

  cv::cvtColor(processed_image, processed_image, cv::COLOR_BGR2GRAY);

  // [TEST] std::cout << "PROCESSED_IMAGE AFTER >>> " << processed_image.channels() << std::endl; // 1

  for (int level=0; level<max_threshold_level; level++)
  {
    if (level == 0)
    {
      cv::Canny(processed_image, result_image, 0, threshold, 5);
      cv::dilate(result_image, result_image, cv::Mat(), cv::Point(-1, -1));
    }
    else
    {
      result_image = processed_image >= (level+1)*255 / max_threshold_level;
    }

    cv::findContours(result_image, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point> approximate;

    for (size_t i=0; i<contours.size(); i++)
    {
      cv::approxPolyDP(cv::Mat(contours[i]), approximate, cv::arcLength(cv::Mat(contours[i]), true) * 0.02, true);

      if (
        approximate.size() == 4
        && std::fabs(cv::contourArea(cv::Mat(approximate))) > 100
        && cv::isContourConvex(cv::Mat(approximate))
      ) {
        double max_cosine = 0;

        for (int j=2; j<5; j++)
        {
          double cosine = std::fabs(angle(approximate[j%4], approximate[j-2], approximate[j-1]));
          max_cosine = std::max(max_cosine, cosine);
        }
        if (max_cosine < 0.3) squares.push_back(approximate);
      }
    }
  }

  return squares;
}

// *** ir 이미지에서 사각형을 찾아 각 선분을 vector로 반환하는 함수 *** //
std::vector<std::vector<cv::Point>> DepthCenterFinder::ir_square_detection(cv::Mat image)
{
  std::vector<std::vector<cv::Point>> squares, contours;

  const int threshold = 50;
  const int max_threshold_level = 2;

  cv::Mat copied_image, temporary_image, processed_image, result_image;
  image.copyTo(copied_image);

  cv::dilate(copied_image, copied_image, cv::Mat(), cv::Point(-1, -1));
  cv::medianBlur(copied_image, copied_image, 5);

  cv::pyrDown(copied_image, temporary_image, cv::Size(copied_image.cols/2, copied_image.rows/2));
  cv::pyrUp(temporary_image, processed_image, copied_image.size());

  // [TEST] std::cout << "PROCESSED_IMAGE >>> " << processed_image.channels() << std::endl; // 1

  processed_image.convertTo(processed_image, CV_8U);

  for (int level=0; level<max_threshold_level; level++)
  {
    if (level == 0)
    {
      cv::Canny(processed_image, result_image, 0, threshold, 5);
      cv::dilate(result_image, result_image, cv::Mat(), cv::Point(-1, -1));
    }
    else
    {
      result_image = processed_image >= (level+1)*255 / max_threshold_level;
    }

    cv::findContours(result_image, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point> approximate;

    for (size_t i=0; i<contours.size(); i++)
    {
      cv::approxPolyDP(cv::Mat(contours[i]), approximate, cv::arcLength(cv::Mat(contours[i]), true) * 0.02, true);

      if (
        approximate.size() == 4
        && std::fabs(cv::contourArea(cv::Mat(approximate))) > 200
        && std::fabs(cv::contourArea(cv::Mat(approximate))) < 15000
        && cv::isContourConvex(cv::Mat(approximate))
      ) {
        double max_cosine = 0;

        for (int j=2; j<5; j++)
        {
          double cosine = std::fabs(angle(approximate[j%4], approximate[j-2], approximate[j-1]));
          max_cosine = std::max(max_cosine, cosine);
        }
        if (max_cosine < 0.3) squares.push_back(approximate);
      }
    }
  }

  return squares;
}

// *** 발견한 선분들로 이미지에 사각형을 그리는 함수 *** //
void DepthCenterFinder::draw_squares(std::string window_name, cv::Mat& image, const std::vector<std::vector<cv::Point> >& squares)
{
  for( size_t i = 0; i < squares.size(); i++ )
  {
    const cv::Point* p = &squares[i][0];
    int n = (int)squares[i].size();
    polylines(image, &p, &n, 1, true, cv::Scalar(0,255,0), 3, cv::LINE_AA);
  }

  cv::imshow(window_name, image);
}

// *** rigid_transformation 함수 *** //
void DepthCenterFinder::rigid_transformation()
{
  // YET
}

// *** point0를 기준으로 point1, point2의 cosine 값을 계산하는 함수 **** //
double DepthCenterFinder::angle(cv::Point point1, cv::Point point2, cv::Point point0)
{
  double dx1 = point1.x - point0.x;
  double dy1 = point1.y - point0.y;
  double dx2 = point2.x - point0.x;
  double dy2 = point2.y - point0.y;

  return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

// *** 인코딩 방식을 상수화하는 함수 *** //
int DepthCenterFinder::encoding_mat_type(const std::string & encoding)
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
    return CV_16SC1;  // ir의 image_raw 인코딩 형식
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
  else
  {
    throw std::runtime_error("Unsupported encoding type");
  }
}