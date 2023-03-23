#include "depth_center_finder/DepthCenterFinder.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_center_finder");
  ros::NodeHandle node_handle;

  /* [EXAMPLE]
  */
  double _box_width, _box_height;
  node_handle.getParam("/depth_center_finder/box_width", _box_width);
  node_handle.getParam("/depth_center_finder/box_width", _box_height);

  DepthCenterFinder depth_center_finder = DepthCenterFinder(node_handle, _box_width, _box_height);

  ros::spin();

  return 0;
}