#include "apriltag_ros/common_functions.h"
#include <apriltag_ros/tagfloor.h>
#include <ros/ros.h>

bool getRosParameter(ros::NodeHandle &pnh, const std::string &name, double &param) {
  // Write parameter "name" from ROS Parameter Server into param
  // Return true if successful, false otherwise
  if (pnh.hasParam(name)) {
    pnh.getParam(name, param);
    ROS_INFO_STREAM("Set camera parameter " << name << " = " << param);
    return true;
  } else {
    ROS_ERROR_STREAM("Could not find parameter: " << name);
    return false;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "apriltag_ros_tagfloor_client"); // client node
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Create a service client for the tagfloor_service
  ros::ServiceClient client = nh.serviceClient<apriltag_ros::tagfloor>("tagfloor_service");

  // Prepare the service request
  apriltag_ros::tagfloor service;

  // Get the base directory where images are stored
  service.request.full_path_to_file_load = apriltag_ros::getAprilTagOption<std::string>(pnh, "image_base_path", "./config/data/");
  if (service.request.full_path_to_file_load.empty()) {
    ROS_ERROR("Image base path is empty!");
    return 1;
  }

  // Fill camera info for each camera (cam0, cam1, cam2, cam3)
  for (int camera_id = 0; camera_id < 4; ++camera_id) {
    sensor_msgs::CameraInfo camera_info;
    camera_info.distortion_model = "plumb_bob";

    double fx, fy, cx, cy;
    std::string prefix = "cam" + std::to_string(camera_id) + "_";

    if (!getRosParameter(pnh, prefix + "fx", fx)) return 1;
    if (!getRosParameter(pnh, prefix + "fy", fy)) return 1;
    if (!getRosParameter(pnh, prefix + "cx", cx)) return 1;
    if (!getRosParameter(pnh, prefix + "cy", cy)) return 1;

    // Intrinsic 
    camera_info.K[0] = fx;
    camera_info.K[2] = cx;
    camera_info.K[4] = fy;
    camera_info.K[5] = cy;
    camera_info.K[8] = 1.0;

    // Projection/camera matrix
    camera_info.P[0] = fx;
    camera_info.P[2] = cx;
    camera_info.P[5] = fy;
    camera_info.P[6] = cy;
    camera_info.P[10] = 1.0;

    service.request.camera_info.push_back(camera_info);
  }

  // Call the service (detect tags in images specified by image_base_path)
  if (client.call(service)) {
    // tag2cam
    for (size_t camera_id = 0; camera_id < service.response.tag_detections.size(); ++camera_id) {
      const auto &detections = service.response.tag_detections[camera_id].detections;
      ROS_INFO_STREAM("Camera " << camera_id << ": detected " << detections.size() << " tags.");

      for (const auto &detection : detections) {
        ROS_INFO_STREAM(" - Tag ID: " << detection.id[0]
                        << ", Position: [" << detection.pose.pose.pose.position.x << ", "
                        << detection.pose.pose.pose.position.y << ", "
                        << detection.pose.pose.pose.position.z << "]");
      }
    }
  } else {
    ROS_ERROR("Failed to call service tagfloor_service");
    return 1;
  }

  return 0; // Successful execution
}
