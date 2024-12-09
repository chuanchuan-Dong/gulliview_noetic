#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/Tag2CamMsg.h>
#include <geometry_msgs/PoseStamped.h>
#include <map>
#include <ros/console.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
// Global tag positions (in meters)
std::map<int, std::array<double, 3>> global_tag_positions = {
    {8, {0, 0, 0}}, {9, {4.28, 0, 0}}, {6, {0, 2, 0}}, {7, {4.28, 2, 0}},
    {4, {0, 4, 0}}, {5, {4.28, 4, 0}}, {2, {0, 6, 0}}, {3, {4.28, 6, 0}},
    {0, {0, 8, 0}}, {1, {4.28, 8, 0}}};

class ObjectPoseEstimator {
private:
  ros::Subscriber tag_detections_sub_;
  ros::Subscriber cam2tag_sub_;
  ros::Publisher global_pose_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  std::map<int, geometry_msgs::PoseWithCovarianceStamped> camera_to_tag_poses_;
  bool camera_pose_received_ = false;

public:
  ObjectPoseEstimator(ros::NodeHandle &nh) {
    // Subscribe to topics
    tag_detections_sub_ =
        nh.subscribe("/camera2/tag_detections", 1,
                     &ObjectPoseEstimator::tagDetectionsCallback, this);
    cam2tag_sub_ = nh.subscribe("/cam2tag/cam1", 1,
                                &ObjectPoseEstimator::cam2TagCallback, this);

    // Publisher for the global pose of the object
    global_pose_pub_ =
        nh.advertise<geometry_msgs::PoseStamped>("global_object_pose", 10);
    broadcastGlobalTagTransforms();
  }

  void cam2TagCallback(const apriltag_ros::Tag2CamMsg::ConstPtr &msg) {
    // camera pose to each tag
    camera_to_tag_poses_.clear();
    for (const auto &detection : msg->detections) {
      camera_to_tag_poses_[detection.id[0]] = detection.pose;
    }

    camera_pose_received_ = !camera_to_tag_poses_.empty();
  }

  void tagDetectionsCallback(
      const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {
    if (!camera_pose_received_) {
      ROS_WARN("Camera poses relative to floor tags not received yet.");
      return;
    }

    for (const auto &detection : msg->detections) {
      int object_tag_id =
          detection.id[0]; // Object tag ID detected by the camera
      geometry_msgs::PoseWithCovarianceStamped object_pose_cam = detection.pose;

      tf2::Quaternion mean_rotation(0.0, 0.0, 0.0, 1.0);
      tf2::Vector3 mean_translation(0.0, 0.0, 0.0);
      int count = 0;
      // Transform the object pose to the global frame using each floor tag's
      // pose, and get the mean global location
      for (const auto &cam_pose : camera_to_tag_poses_) {
        count++;
        int floor_tag_id = cam_pose.first;
        geometry_msgs::PoseWithCovarianceStamped cam_pose_tag = cam_pose.second;

        if (global_tag_positions.find(floor_tag_id) ==
            global_tag_positions.end()) {
          ROS_WARN("Floor tag ID %d not found in global tag positions",
                   floor_tag_id);
          continue;
        }

        tf2::Transform tf_object_cam,
            tf_cam_tag; // Convert poses to tf2 for computation
        tf2::fromMsg(object_pose_cam.pose.pose, tf_object_cam);
        tf2::fromMsg(cam_pose_tag.pose.pose, tf_cam_tag);

        // Compute object pose in global frame: Global = (GlobalFloorTagPose +
        // (CameraToTag * TagToObject))
        tf2::Transform tf_object_global = tf_cam_tag * tf_object_cam;

        std::array<double, 3> global_shift = global_tag_positions[floor_tag_id];
        tf_object_global.getOrigin() +=
            tf2::Vector3(global_shift[0], global_shift[1],
                         global_shift[2]); // shift to the global

        mean_translation += tf_object_global.getOrigin();
        mean_rotation += tf_object_global.getRotation();

        ROS_INFO_STREAM("Global Location Based on floor tag id:"
                        << floor_tag_id
                        << "\t: " << tf2::toMsg(tf_object_global));
      }
      mean_translation /= count;
      tf2::Transform object_global_mean(mean_rotation.normalize(),
                                        mean_translation);
      ROS_INFO_STREAM(
          "Global Location(Mean):" << "\t: " << tf2::toMsg(object_global_mean));
      // TF tree build
      geometry_msgs::TransformStamped object_transform_stamped;
      object_transform_stamped.header.stamp = ros::Time::now();
      object_transform_stamped.header.frame_id = "global_frame";
      object_transform_stamped.child_frame_id =
          "object_" + std::to_string(object_tag_id);

      object_transform_stamped.transform.translation.x =
          object_global_mean.getOrigin().x();
      object_transform_stamped.transform.translation.y =
          object_global_mean.getOrigin().y();
      object_transform_stamped.transform.translation.z =
          object_global_mean.getOrigin().z();

      object_transform_stamped.transform.rotation =
          tf2::toMsg(object_global_mean.getRotation());

      tf_broadcaster_.sendTransform(object_transform_stamped);
    }
  }

  void broadcastGlobalTagTransforms() {
    // Broadcast transforms from each tag to the global frame
    for (const auto &tag_pos : global_tag_positions) {
      int tag_id = tag_pos.first;
      const std::array<double, 3> &global_position = tag_pos.second;

      geometry_msgs::TransformStamped transform_stamped;
      transform_stamped.header.stamp = ros::Time::now();
      transform_stamped.header.frame_id = "global_frame";
      transform_stamped.child_frame_id = "floor_tag_" + std::to_string(tag_id);

      // Define the translation (position of the tag in the global frame)
      transform_stamped.transform.translation.x = global_position[0];
      transform_stamped.transform.translation.y = global_position[1];
      transform_stamped.transform.translation.z = global_position[2];

      // Set rotation to identity (no rotation relative to the global frame)
      transform_stamped.transform.rotation.w = 1.0;
      transform_stamped.transform.rotation.x = 0.0;
      transform_stamped.transform.rotation.y = 0.0;
      transform_stamped.transform.rotation.z = 0.0;

      // Broadcast the transform
      tf_broadcaster_.sendTransform(transform_stamped);
      ROS_INFO_STREAM("Broadcasting transform for floor tag " << tag_id);
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "object_pose_estimator");
  ros::NodeHandle nh;

  ObjectPoseEstimator estimator(nh);
  ros::spin();

  return 0;
}
