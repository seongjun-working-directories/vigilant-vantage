#include "depth_center_finder/DepthCenterFinder.hpp"

void DepthCenterFinder::color_image_callback(const sensor_msgs::Image::ConstPtr& messages)
{
  cv::Mat frame(
    messages->height - 80,  // 480px - 80px
    messages->width,        // 640px
    encoding_mat_type(messages->encoding),
    const_cast<unsigned char*>(messages->data.data()),
    messages->step
  );

  if (store_image_command_color)
  {
    frame.copyTo(color_image);

    std::cout << "[" << ros::Time::now() << "] /camera/color/image_raw의 1개 프레임이 저장되었습니다. " << std::endl;
    store_image_command_color = false;

    cv::imshow("color", color_image);
  }

  cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
  cv::imshow("default color", frame);
  cv::waitKey(1);
}

void DepthCenterFinder::ir_image_callback(const sensor_msgs::Image::ConstPtr& messages)
{
  cv::Mat frame(
    messages->height,   // 400px
    messages->width,    // 640px
    encoding_mat_type(messages->encoding),
    const_cast<unsigned char*>(messages->data.data()),
    messages->step
  );

  frame.convertTo(frame, CV_8U);

  if (store_image_command_ir)
  {
    frame.copyTo(ir_image);

    std::cout << "[" << ros::Time::now() << "] /camera/ir/image_raw의 1개 프레임이 저장되었습니다. " << std::endl;
    store_image_command_ir = false;

    cv::imshow("ir", ir_image);
  }

  cv::imshow("default ir", frame);
  cv::waitKey(10);
}

void DepthCenterFinder::store_image_command_callback(const std_msgs::Bool::ConstPtr& messages)
{
  store_image_command_color = messages->data;
  store_image_command_ir = messages->data;
  if (messages->data) std::cout << "STORING COLOR & IR IMAGES" << std::endl;
}

void DepthCenterFinder::run_finder_command_callback(const std_msgs::Bool::ConstPtr& messages)
{
  if (
    (messages->data)            // 조건 1 : process_image 토픽 값이 true 이어야 함
    && (!(color_image.empty())) // 조건 2 : color image 가 저장되어 있어야 함
    && (!(ir_image.empty()))    // 조건 3 : ir_image 가 저장되어 있어야 함
  ) {
    color_square_corners.clear();
    ir_square_corners.clear();

    color_square_corners = color_square_detection(color_image);
    ir_square_corners = ir_square_detection(ir_image);

    draw_squares("square detected color", color_image, color_square_corners);
    draw_squares("square detected ir", ir_image, ir_square_corners);

    /* [TEST]
    std::cout << "COLOR_SQUARE_CORNERS.SIZE() >>> " << color_square_corners.size() << std::endl;
    std::cout << "IR_SQUARE_CORNERS.SIZE() >>> " << ir_square_corners.size() << std::endl;
    */

    if (
      (color_square_corners.size() == 2)
      && (ir_square_corners.size() == 2)
    ) {
      std::vector<cv::Point> color_corner_vector, ir_corner_vector;

      for (auto &color_square_corner : color_square_corners)
      {
        for (auto &element : color_square_corner)
        {
          color_corner_vector.push_back(element);
        }
      }

      for (auto &ir_square_corner : ir_square_corners)
      {
        for (auto &element : ir_square_corner)
        {
          ir_corner_vector.push_back(element);
        }
      }

      cv::Mat result = cv::estimateAffine2D(ir_corner_vector, color_corner_vector);

      /* [TEST]
      for (int i=0; i<result.rows; i++)
      {
        for (int j=0; j<result.cols; j++)
        {
          std::cout << result.at<double>(i, j) << " ";
        }
        std::cout << std::endl;
      }
      */

      /* [RESULT]
      COLOR_SQUARE_CORNERS.SIZE() >>> 2
      IR_SQUARE_CORNERS.SIZE() >>> 2
      // [a11 a12 b1; a21 a22 b2]
      0.992245 -0.0399901 -2.87683
      -0.0296827 0.999392 51.3332
      */
    }

    cv::waitKey(1);
  }
}