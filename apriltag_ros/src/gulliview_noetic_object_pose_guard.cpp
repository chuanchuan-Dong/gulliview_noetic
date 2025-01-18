#include "apriltag_ros/ObjGlobalPose.h"
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <map>
#include <ros/console.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

class GuardNode {
public:
  GuardNode(ros::NodeHandle &nh) {
    for (int i = 0; i < 4; i++) {
      std::string topic = "/camera" + std::to_string(i) + "/global_object_pose";
      camera_subs_.emplace_back(
          nh.subscribe(topic, 10, &GuardNode::poseCallback, this));
    }

    global_pose_pub_ =
        nh.advertise<geometry_msgs::PoseStamped>("/global_object_pose", 10);
  }

private:
  ros::Publisher global_pose_pub_;
  std::vector<ros::Subscriber> camera_subs_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  std::map<int, std::pair<double, double>> camera_positions =
      { // to find object close to which area.
          {0, {2.1, 1}},
          {1, {2.1, 3}},
          {2, {2.1, 5}},
          {3, {2.1, 7}}};

  struct ObjectInfo {
    geometry_msgs::PoseStamped pose;
    int best_camera;
    double min_distance;

    // Constructor to initialize
    ObjectInfo() : best_camera(-1), min_distance(999) {}

    ObjectInfo(const geometry_msgs::PoseStamped &p, int cam, double dist)
        : pose(p), best_camera(cam), min_distance(dist) {}
  };

  std::map<int, ObjectInfo> object_tracking_;

  void poseCallback(const apriltag_ros::ObjGlobalPose::ConstPtr &msg) {
    if (msg->pose.header.frame_id.empty()) {
      ROS_ERROR("<guard>: Received empty frame_id");
      return;
    }

    if (msg->ObjID.empty()) {
      ROS_ERROR("<guard>: Received empty ObjID");
      return;
    }

    int camera_id = extractCameraID(msg->pose.header.frame_id);
    if (camera_id == -1)
      return;

    int object_id = extractObjectID(msg->ObjID);
    if (object_id == -1)
      return;

    double obj_x = msg->pose.pose.position.x;
    double obj_y = msg->pose.pose.position.y;

    double min_dist = std::numeric_limits<double>::max();
    int closest_camera = -1;

    for (const auto &[cam_id, cam_pos] : camera_positions) {
      double dist = std::hypot(cam_pos.first - obj_x, cam_pos.second - obj_y);
      if (dist < min_dist) {
        min_dist = dist;
        closest_camera = cam_id;
      }
    }

    if (closest_camera == -1)
      return;

    ROS_INFO_STREAM("<guard>: Camera " << closest_camera
                                       << " selected for object " << object_id);

    if (object_tracking_.find(object_id) == object_tracking_.end() ||
        min_dist < object_tracking_[object_id].min_distance) {
      object_tracking_[object_id] =
          ObjectInfo(msg->pose, closest_camera, min_dist);
    }

    global_pose_pub_.publish(object_tracking_[object_id].pose);

    // construct tf trees
    geometry_msgs::TransformStamped object_transform_stamped;
    object_transform_stamped.header.stamp = ros::Time::now();
    object_transform_stamped.header.frame_id = "global_frame";
    object_transform_stamped.child_frame_id =
        "object_" + std::to_string(object_id);

    object_transform_stamped.transform.translation.x =
        msg->pose.pose.position.x;
    object_transform_stamped.transform.translation.y =
        msg->pose.pose.position.y;
    object_transform_stamped.transform.translation.z =
        msg->pose.pose.position.z;

    object_transform_stamped.transform.rotation = msg->pose.pose.orientation;

    tf_broadcaster_.sendTransform(object_transform_stamped);
  }

  int extractCameraID(const std::string &frame_id) {
    if (frame_id.find("camera") == std::string::npos || frame_id.length() < 7) {
      ROS_ERROR("<guard>: Error extracting camera ID from frame_id: %s",
                frame_id.c_str());
      return -1;
    }
    return frame_id[6] - '0';
  }

  int extractObjectID(const std::string &ObjID) {
    if (ObjID.find("object_") == std::string::npos || ObjID.length() <= 7) {
      ROS_ERROR("<guard>: Error extracting object ID from ObjID: %s",
                ObjID.c_str());
      return -1;
    }
    try {
      return std::stoi(ObjID.substr(7));
    } catch (const std::exception &e) {
      ROS_ERROR("<guard>: Failed to convert ObjID to integer: %s",
                ObjID.c_str());
      return -1;
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "guard_node");
  ros::NodeHandle nh;
  GuardNode guard(nh);
  ros::spin();
}
