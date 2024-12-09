#include "apriltag_ros/camera_detection_stream.h"
#include <ros/ros.h>
#include <string>



int main(int argc, char **argv) {
  ros::init(argc, argv, "apriltag_ros_detector_stream");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  apriltag_ros::DetectorStream detector;
  detector.initialize(nh, pnh);

  ros::spin();
  return 0;
}
