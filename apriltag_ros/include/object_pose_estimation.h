#ifndef OBJECT_POSE_ESTIMATOR_H
#define OBJECT_POSE_ESTIMATOR_H

#include "ros/node_handle.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/Tag2CamMsg.h>
#include <map>
#include <array>

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
    {8, {0, 0, 0}}, {9, {4.28, 0, 0}}, {6, {0, 2, 0}}, {7, {4.28, 2, 0}},
    {4, {0, 4, 0}}, {5, {4.28, 4, 0}}, {2, {0, 6, 0}}, {3, {4.28, 6, 0}},
    {0, {0, 8, 0}}, {1, {4.28, 8, 0}}};

  bool camera_pose_received_ = false;

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
