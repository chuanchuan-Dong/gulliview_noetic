#ifndef OBJECT_POSE_ESTIMATOR_H
#define OBJECT_POSE_ESTIMATOR_H

#include "ros/node_handle.h"
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/Tag2CamMsg.h>
#include <array>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <map>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

class ObjectPoseEstimator {
private:
  ros::Subscriber tag_detections_sub_;
  ros::Subscriber cam2tag_sub_;
  ros::Publisher global_pose_pub_;
  ros::Publisher gloabl_odometry_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  std::map<int, geometry_msgs::PoseWithCovarianceStamped> camera_to_tag_poses_;
  // std::map<int, std::array<double, 3>> global_tag_positions;

  std::map<int, std::array<double, 3>> global_tag_positions = {
      {8, {0, 0, 0}}, {9, {4.265, 0, 0}}, {6, {0, 2, 0}}, {7, {4.268, 2, 0}},
      {4, {0, 4, 0}}, {5, {4.268, 4, 0}}, {2, {0, 6, 0}}, {3, {4.295, 6, 0}},
      {0, {0, 8, 0}}, {1, {4.3, 8, 0}}};

  bool camera_pose_received_ = false;
  int camera_index, cam2tag_index;
  void tagDetectionsCallback(
      const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);
  void cam2TagCallback(const apriltag_ros::Tag2CamMsg::ConstPtr &msg);
  void publishGlobalOdometry(const tf2::Transform &object_global_mean,
                             int object_tag_id);
  void broadcastGlobalTagTransforms();

public:
  ObjectPoseEstimator(ros::NodeHandle &nh, ros::NodeHandle &pnh);
};

#endif // OBJECT_POSE_ESTIMATOR_H
