

#ifndef APRILTAG_ROS_TAGFLOOR_DETECTOR_H
#define APRILTAG_ROS_TAGFLOOR_DETECTOR_H

#include "apriltag_ros/AprilTagDetectionArray.h"
#include "apriltag_ros/Tag2CamMsg.h"
#include "apriltag_ros/common_functions.h"
#include <apriltag_ros/tagfloor.h>
#include <ros/node_handle.h>
#include <ros/timer.h>
#include <vector>

namespace apriltag_ros {

class TagFloorDetector {
  // once trigger the service, will continuously publish cam2tag pose
private:
  TagDetector tag_detector_;
  ros::ServiceServer tag_floor_service_;

  ros::Publisher tag_floor_detection_pub_cam0_, tag_floor_detection_pub_cam1_,
      tag_floor_detection_pub_cam2_, tag_floor_detection_pub_cam3_;

  std::vector<AprilTagDetectionArray> tagfloor_2cam_v_;
  std::vector<Tag2CamMsg> cam_2tagfloor_v_;
  std::map<int, ros::Publisher *> publishers = {
      {0, &tag_floor_detection_pub_cam0_},
      {1, &tag_floor_detection_pub_cam1_},
      {2, &tag_floor_detection_pub_cam2_},
      {3, &tag_floor_detection_pub_cam3_}};

  ros::Timer publish_timer_;
  bool publishing_ = false;
  tf2_ros::TransformBroadcaster broadcaster_;

public:
  TagFloorDetector(ros::NodeHandle &nh, ros::NodeHandle &pnh);

  bool analyzeImage(apriltag_ros::tagfloor::Request &request,
                    apriltag_ros::tagfloor::Response &response);

  void publishDetections(const ros::TimerEvent &); // publish the pose and TF

  void addStaticTransform(const std::string &parent_frame,
                          const std::string &child_frame,
                          const Eigen::Vector3d &translation);
  void tf_publisher();
};

} // namespace apriltag_ros

#endif