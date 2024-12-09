#include "apriltag_ros/tagfloor_server.h"
#include <ros/ros.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "tag_floor_detector");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Create the TagFloorDetector object, initializing service and timer
  apriltag_ros::TagFloorDetector tag_floor_detector(nh, pnh);

  ROS_INFO("TagFloorDetector node is running...");

  ros::spin();

  return 0;
}

