#include "apriltag_ros/tagfloor_server.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "apriltag_ros/Tag2CamMsg.h"
#include "apriltag_ros/common_functions.h"
#include "apriltag_ros/tagfloor.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/core/persistence.hpp"
#include "opencv2/imgcodecs.hpp"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/console.h>
#include <ros/init.h>
#include <ros/publisher.h>
#include <ros/rate.h>
#include <std_msgs/Header.h>
#include <string>
#include <tf2_ros/transform_broadcaster.h>

namespace apriltag_ros {
// constructor
TagFloorDetector::TagFloorDetector(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    : tag_detector_(pnh) {
  tag_floor_service_ = nh.advertiseService(
      "tagfloor_service", &TagFloorDetector::analyzeImage, this);
  tag_floor_detection_pub_cam0_ = nh.advertise<Tag2CamMsg>("cam2tag/cam0", 1);
  tag_floor_detection_pub_cam1_ = nh.advertise<Tag2CamMsg>("cam2tag/cam1", 1);
  tag_floor_detection_pub_cam2_ = nh.advertise<Tag2CamMsg>("cam2tag/cam2", 1);
  tag_floor_detection_pub_cam3_ = nh.advertise<Tag2CamMsg>("cam2tag/cam3", 1);

  tagfloor_2cam_v_.resize(4); // as return to client.
  cam_2tagfloor_v_.resize(4); // as publish information.

  publish_timer_ =
      nh.createTimer(ros::Duration(0.1), &TagFloorDetector::publishDetections,
                     this, false, false);

  ROS_INFO_STREAM("TagFloorDetector initialized. Waiting for service call...");
}

bool TagFloorDetector::analyzeImage(
    apriltag_ros::tagfloor::Request &request,
    apriltag_ros::tagfloor::Response &response) {
  response.tag_detections.clear();
  response.tag_detections.resize(4);
  // map topic with publisher
  for (const auto &entry : publishers) {
    int camera_id = entry.first;
    ros::Publisher *publisher = entry.second;

    std::string cam_path = request.full_path_to_file_load + "cam" +
                           std::to_string(camera_id) + ".jpg";

    cv::Mat image = cv::imread(cam_path, cv::IMREAD_COLOR);
    if (image.empty()) {
      ROS_ERROR_STREAM("Could not read image: " << cam_path);
      continue;
    }

    ROS_INFO_STREAM("Read image from " << cam_path);

    cv_bridge::CvImagePtr loaded_image(
        new cv_bridge::CvImage(std_msgs::Header(), "bgr8", image));
    loaded_image->header.frame_id = "cam" + std::to_string(camera_id);

    // Get estimated tag pose in the camera frame.
    //   - camera frame: looking from behind the camera (like a
    //     photographer), x is right, y is down and z is straight
    //     ahead
    //   - tag frame: looking straight at the tag (oriented correctly),
    //     x is right, y is up and z is towards you (out of the tag).

    apriltag_ros::AprilTagDetectionArray detections_tag2cam =
        tag_detector_.detectTags(
            loaded_image,
            sensor_msgs::CameraInfoConstPtr(
                new sensor_msgs::CameraInfo(request.camera_info[camera_id])));
    response.tag_detections[camera_id] = detections_tag2cam;
    tagfloor_2cam_v_[camera_id] = detections_tag2cam; 

    // convert to cam2tag
    apriltag_ros::Tag2CamMsg detections_cam2tag;
    detections_cam2tag.camera_id = std::to_string(camera_id);

    for (const auto &detection :
         detections_tag2cam.detections) { // for each detection, for each detection, only one tag included.
      geometry_msgs::PoseWithCovarianceStamped pose_tag2cam = detection.pose;

      Eigen::Quaterniond q_tag2cam(pose_tag2cam.pose.pose.orientation.w,
                                   pose_tag2cam.pose.pose.orientation.x,
                                   pose_tag2cam.pose.pose.orientation.y,
                                   pose_tag2cam.pose.pose.orientation.z);

      Eigen::Vector3d t_tag2cam(pose_tag2cam.pose.pose.position.x,
                                pose_tag2cam.pose.pose.position.y,
                                pose_tag2cam.pose.pose.position.z);

      Eigen::Quaterniond q_cam2tag = q_tag2cam.inverse(); // cam2tag
      Eigen::Vector3d t_cam2tag = -(q_cam2tag * t_tag2cam);

      // fill the message
      apriltag_ros::AprilTagDetection detection_cam2tag;
      geometry_msgs::PoseWithCovarianceStamped pose_cam2tag;

      pose_cam2tag.pose.pose.position.x = t_cam2tag.x();
      pose_cam2tag.pose.pose.position.y = t_cam2tag.y();
      pose_cam2tag.pose.pose.position.z = t_cam2tag.z();
      pose_cam2tag.pose.pose.orientation.w = q_cam2tag.w();
      pose_cam2tag.pose.pose.orientation.x = q_cam2tag.x();
      pose_cam2tag.pose.pose.orientation.y = q_cam2tag.y();
      pose_cam2tag.pose.pose.orientation.z = q_cam2tag.z();

      detection_cam2tag.id.push_back(detection.id[0]);
      detection_cam2tag.size.push_back(detection.size[0]);
      detection_cam2tag.pose = pose_cam2tag;

      detections_cam2tag.detections.push_back(detection_cam2tag);
    }
    cam_2tagfloor_v_[camera_id] = detections_cam2tag; // 

  }

  publishing_ = true;     // Start publishing
  publish_timer_.start(); // Start the timer

  return true;
}

void TagFloorDetector::publishDetections(const ros::TimerEvent &) {
  if (!publishing_) {
    return;
  }

  ROS_INFO_STREAM("Camera to Floor Tag Pose is available now :)\n");
  // publish for each tag2camera pose
  ROS_INFO("Publishing tag floor detections...");
  for (const auto &entry : publishers) {
    int camera_id = entry.first;
    ros::Publisher *publisher = entry.second;
    publisher->publish(cam_2tagfloor_v_[camera_id]); // publish camera2tag

    // AprilTagDetectionArray floortag2cam = tagfloor_global_v_[camera_id];

    // for (unsigned int i = 0; i < floortag2cam.detections.size(); i++) {
    //   geometry_msgs::PoseStamped pose;
    //   pose.pose = floortag2cam.detections[i].pose.pose.pose;
    //   pose.header = floortag2cam.detections[i].pose.header;

    //   geometry_msgs::TransformStamped tag_transform;

    //   tag_transform.header = pose.header;
    //   tag_transform.header.stamp = ros::Time::now();
    //   tag_transform.header.frame_id = "camera" + std::to_string(camera_id) +
    //   std::to_string(floortag2cam.detections[i].id[0]);
    //   tag_transform.child_frame_id =
    //       "tag_" + std::to_string(floortag2cam.detections[i].id[0]);

    //   tag_transform.transform.translation.x = pose.pose.position.x;
    //   tag_transform.transform.translation.y = pose.pose.position.y;
    //   tag_transform.transform.translation.z = pose.pose.position.z;
    //   tag_transform.transform.rotation = pose.pose.orientation;

    //   // Broadcast the transform
    //   broadcaster_.sendTransform(tag_transform);
    // }

    // publish tf for tagfloor 2 global
    //
    // tf_publisher();
  }
}

//   void TagFloorDetector::tf_publisher() {

//     std::unordered_map<int, Eigen::Vector3d> tag_positions = {
//         {0, Eigen::Vector3d(0.0, 0.0, 0.0)},
//         {1, Eigen::Vector3d(4.25, 0.0, 0.0)},
//         {2, Eigen::Vector3d(0.0, 2.0, 0.0)},
//         {3, Eigen::Vector3d(4.25, 2.0, 0.0)},
//         {4, Eigen::Vector3d(0.0, 4.0, 0.0)},
//         {5, Eigen::Vector3d(4.28, 4.0, 0.0)},
//         {6, Eigen::Vector3d(0.0, 6.0, 0.0)},
//         {7, Eigen::Vector3d(4.28, 6.0, 0.0)},
//         {8, Eigen::Vector3d(0, 8.0, 0.0)},
//         {9, Eigen::Vector3d(4.28, 8.0, 0.0)}};

//     for (const auto &tag_pair : tag_positions) {
//       int tag_id = tag_pair.first;
//       const Eigen::Vector3d &position = tag_pair.second;

//       std::string parent_frame = "global";
//       std::string child_frame = "tag_" + std::to_string(tag_id);

//       // Publish the transform from tag 0 to the current tag
//       addStaticTransform(parent_frame, child_frame, position);
//     }
//   }

//   void TagFloorDetector::addStaticTransform(
//       const std::string &parent_frame, const std::string &child_frame,
//       const Eigen::Vector3d &translation) {
//     geometry_msgs::TransformStamped transformStamped;

//     transformStamped.header.stamp = ros::Time::now();
//     transformStamped.header.frame_id = parent_frame;
//     transformStamped.child_frame_id = child_frame;

//     transformStamped.transform.translation.x = translation.x();
//     transformStamped.transform.translation.y = translation.y();
//     transformStamped.transform.translation.z = translation.z();
//     transformStamped.transform.rotation.x = 0.0;
//     transformStamped.transform.rotation.y = 0.0;
//     transformStamped.transform.rotation.z = 0.0;
//     transformStamped.transform.rotation.w = 1.0;

//     broadcaster_.sendTransform(transformStamped);
//   }

} // namespace apriltag_ros

// Eigen::Quaterniond q_tag2cam(pose_tag2cam.pose.pose.orientation.w,
//                              pose_tag2cam.pose.pose.orientation.x,
//                              pose_tag2cam.pose.pose.orientation.y,
//                              pose_tag2cam.pose.pose.orientation.z);

// // Convert quaternion to rotation matrix (R_tag2cam)
// Eigen::Matrix3d R_tag2cam = q_tag2cam.toRotationMatrix();

// Eigen::Vector3d t_tag2cam(pose_tag2cam.pose.pose.position.x,
//                           pose_tag2cam.pose.pose.position.y,
//                           pose_tag2cam.pose.pose.position.z);

// Eigen::Matrix3d R_cam2tag =
//     R_tag2cam.transpose(); // Transpose of R_tag2cam
// Eigen::Vector3d t_cam2tag = -R_cam2tag * t_tag2cam;