#include <ros/ros.h>
#include "exportbag/export_bag.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "export_bag");
  ros::NodeHandle nh("~");

  try {
    exportbag::ExportBag node(nh);
    node.initialize();
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
  }
  return 0;
}
