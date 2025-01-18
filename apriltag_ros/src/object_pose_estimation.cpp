#include "object_pose_estimation.h"
#include "ros/node_handle.h"
#include <apriltag_ros/ObjGlobalPose.h>
#include <string>

ObjectPoseEstimator::ObjectPoseEstimator(ros::NodeHandle &nh,
                                         ros::NodeHandle &pnh) {

  pnh.param<int>("camera_index", camera_index, 999);
  pnh.param<int>("cam2tag_index", cam2tag_index, 999);

  // Subscribe to topics
  tag_detections_sub_ =
      nh.subscribe("/camera" + std::to_string(camera_index) + "/tag_detections",
                   1, &ObjectPoseEstimator::tagDetectionsCallback, this);
  cam2tag_sub_ = nh.subscribe("/cam2tag/cam" + std::to_string(cam2tag_index), 1,
                              &ObjectPoseEstimator::cam2TagCallback, this);

  // Publisher for the global pose of the object
  global_pose_pub_ =
      nh.advertise<apriltag_ros::ObjGlobalPose>( // the msg type contain id
                                                 // number + geometry pose msg
          "/camera" + std::to_string(camera_index) + "/global_object_pose", 10);

  gloabl_odometry_pub_ = nh.advertise<nav_msgs::Odometry>(
      "/camera" + std::to_string(camera_index) + "/odom_gv", 10);

  // broadcastGlobalTagTransforms();
}

void ObjectPoseEstimator::cam2TagCallback(
    const apriltag_ros::Tag2CamMsg::ConstPtr &msg) {
  camera_to_tag_poses_.clear();
  for (const auto &detection : msg->detections) {
    camera_to_tag_poses_[detection.id[0]] = detection.pose;
  }
  if (camera_to_tag_poses_.size() != 4) {
    ROS_ERROR("Camera to floor tag pose error. check whether there is 4 "
              "floortag pose publish by server node");
  }
  camera_pose_received_ = !camera_to_tag_poses_.empty();
}
// detection send
void ObjectPoseEstimator::tagDetectionsCallback(
    const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {
  if (!camera_pose_received_) {
    ROS_WARN("Camera poses relative to floor tags not received yet.");
    return;
  }

  for (const auto &detection : msg->detections) {
    int object_tag_id = detection.id[0];
    geometry_msgs::PoseWithCovarianceStamped object_pose_cam = detection.pose;

    tf2::Quaternion mean_rotation(0.0, 0.0, 0.0, 1.0);
    tf2::Vector3 mean_translation(0.0, 0.0, 0.0);
    int count = 0;

    for (const auto &cam_pose :
         camera_to_tag_poses_) { // for each camera should have 4 floortag
                                 // detected
      count++;
      int floor_tag_id = cam_pose.first;
      geometry_msgs::PoseWithCovarianceStamped cam_pose_tag = cam_pose.second;

      if (global_tag_positions.find(floor_tag_id) ==
          global_tag_positions.end()) {
        ROS_WARN("Floor tag ID %d not found in global tag positions",
                 floor_tag_id);
        continue;
      }

      tf2::Transform tf_object_cam, tf_cam_tag;
      tf2::fromMsg(object_pose_cam.pose.pose, tf_object_cam);
      tf2::fromMsg(cam_pose_tag.pose.pose, tf_cam_tag);

      tf2::Transform tf_object_global = tf_cam_tag * tf_object_cam;

      std::array<double, 3> global_shift = global_tag_positions[floor_tag_id];
      tf_object_global.getOrigin() +=
          tf2::Vector3(global_shift[0], global_shift[1], global_shift[2]);

      mean_translation += tf_object_global.getOrigin();
      mean_rotation += tf_object_global.getRotation();

      // ROS_INFO_STREAM("<ObjectPoseEstimator>:Global Location Based on floor
      // tag id:"
      //                 << floor_tag_id
      //                 << "\t: " << tf2::toMsg(tf_object_global));
    }

    mean_translation /= count;
    tf2::Transform object_global_mean(mean_rotation.normalize(),
                                      mean_translation);
    ROS_INFO_STREAM("<ObjectPoseEstimator>: From Camera "
                    << camera_index << "\tGlobal Location(Mean):" << "\n: "
                    << tf2::toMsg(object_global_mean));

    // Publish global pose (to be used by the guard node)
    geometry_msgs::PoseStamped global_pose_geometry_msg;
    apriltag_ros::ObjGlobalPose global_pose_msg;
    global_pose_msg.ObjID = "object_" + std::to_string(object_tag_id);
    global_pose_geometry_msg.header.stamp = ros::Time::now();
    global_pose_geometry_msg.header.frame_id =
        "camera" + std::to_string(camera_index);
    global_pose_geometry_msg.pose.position.x =
        object_global_mean.getOrigin().x();
    global_pose_geometry_msg.pose.position.y =
        object_global_mean.getOrigin().y();
    global_pose_geometry_msg.pose.position.z =
        object_global_mean.getOrigin().z();
    global_pose_geometry_msg.pose.orientation =
        tf2::toMsg(object_global_mean.getRotation());
    global_pose_msg.pose = global_pose_geometry_msg;
    global_pose_pub_.publish(global_pose_msg);

    // publishGlobalOdometry(object_global_mean, object_tag_id);
    //
    //
    // TF Tree will be construct at guard node.

    // geometry_msgs::TransformStamped object_transform_stamped;
    // object_transform_stamped.header.stamp = ros::Time::now();
    // object_transform_stamped.header.frame_id = "global_frame";
    // object_transform_stamped.child_frame_id =
    //     "object_" + std::to_string(object_tag_id);

    // object_transform_stamped.transform.translation.x =
    //     object_global_mean.getOrigin().x();
    // object_transform_stamped.transform.translation.y =
    //     object_global_mean.getOrigin().y();
    // object_transform_stamped.transform.translation.z =
    //     object_global_mean.getOrigin().z();

    // object_transform_stamped.transform.rotation =
    //     tf2::toMsg(object_global_mean.getRotation());

    // tf_broadcaster_.sendTransform(object_transform_stamped);
  }
}

// void ObjectPoseEstimator::publishGlobalOdometry(
//     const tf2::Transform &object_global_mean, int object_tag_id) {
//   nav_msgs::Odometry odometry_msg;
//   odometry_msg.header.stamp = ros::Time::now();
//   odometry_msg.header.frame_id = "global_frame";
//   odometry_msg.child_frame_id = "object_" + std::to_string(object_tag_id);

//   odometry_msg.pose.pose.position.x = object_global_mean.getOrigin().x();
//   odometry_msg.pose.pose.position.y = object_global_mean.getOrigin().y();
//   odometry_msg.pose.pose.position.z = object_global_mean.getOrigin().z();

//   odometry_msg.pose.pose.orientation =
//       tf2::toMsg(object_global_mean.getRotation());

//   odometry_msg.twist.twist.linear.x = 0.0;
//   odometry_msg.twist.twist.linear.y = 0.0;
//   odometry_msg.twist.twist.linear.z = 0.0;

//   odometry_msg.twist.twist.angular.x = 0.0;
//   odometry_msg.twist.twist.angular.y = 0.0;
//   odometry_msg.twist.twist.angular.z = 0.0;

//   gloabl_odometry_pub_.publish(odometry_msg);
// }

// void ObjectPoseEstimator::broadcastGlobalTagTransforms() {
//   for (const auto &tag_pos : global_tag_positions) {
//     int tag_id = tag_pos.first;
//     const std::array<double, 3> &global_position = tag_pos.second;

//     geometry_msgs::TransformStamped transform_stamped;
//     transform_stamped.header.stamp = ros::Time::now();
//     transform_stamped.header.frame_id = "global_frame";
//     transform_stamped.child_frame_id = "floor_tag_" + std::to_string(tag_id);

//     transform_stamped.transform.translation.x = global_position[0];
//     transform_stamped.transform.translation.y = global_position[1];
//     transform_stamped.transform.translation.z = global_position[2];

//     transform_stamped.transform.rotation.w = 1.0;
//     transform_stamped.transform.rotation.x = 0.0;
//     transform_stamped.transform.rotation.y = 0.0;
//     transform_stamped.transform.rotation.z = 0.0;

//     tf_broadcaster_.sendTransform(transform_stamped);
//     ROS_INFO_STREAM("Broadcasting transform for floor tag " << tag_id);
//   }
// }
