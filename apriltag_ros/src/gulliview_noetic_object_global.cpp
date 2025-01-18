#include "ros/node_handle.h"
#include "ros/publisher.h"
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/Tag2CamMsg.h>
#include <geometry_msgs/PoseStamped.h>
#include <map>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "object_pose_estimation.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "object_pose_estimator");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ObjectPoseEstimator estimator(nh, pnh);
  ros::spin();

  return 0;
}
