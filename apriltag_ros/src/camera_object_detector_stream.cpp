
#include "apriltag_ros/camera_detection_stream.h"
#include <ctime>
#include <ros/console.h>
#include <string>

namespace apriltag_ros {

void DetectorStream::initialize(ros::NodeHandle &nh, ros::NodeHandle &pnh) {
  tag_detector_ = std::make_shared<TagDetector>(pnh);

  draw_tag_detections_image_ =
      pnh.param<bool>("publish_tag_detections_image", false);
  it_ = std::make_shared<image_transport::ImageTransport>(nh);

  std::string transport_hint;
  pnh.param<std::string>("transport_hint", transport_hint, "raw");

  int camera_index;
  pnh.param<int>("camera_index", camera_index, 999);

  if (camera_index == 999) {
      ROS_ERROR("Set the camera correctly");
  }

  int queue_size;
  pnh.param<int>("queue_size", queue_size, 1);

  pnh.param<double>("cam" + std::to_string(camera_index) +"_fx", cam_fx_, 0.0);
  pnh.param<double>("cam" + std::to_string(camera_index) +"_fy", cam_fy_, 0.0);
  pnh.param<double>("cam" + std::to_string(camera_index) +"_cx", cam_cx_, 0.0);
  pnh.param<double>("cam" + std::to_string(camera_index) +"_cy", cam_cy_, 0.0);

  camera_info_.distortion_model = "plumb_bob";
  camera_info_.K = {cam_fx_, 0.0, cam_cx_, 0.0, cam_fy_,
                    cam_cy_, 0.0, 0.0,     1.0};
  camera_info_.P[0] = cam_fx_;
  camera_info_.P[2] = cam_cx_;
  camera_info_.P[5] = cam_fy_;
  camera_info_.P[6] = cam_cy_;
  camera_info_.P[10] = 1.0;

  tag_detections_publisher_ =
      nh.advertise<AprilTagDetectionArray>("camera" + std::to_string(camera_index) + "/tag_detections", 1);

  if (draw_tag_detections_image_) {
    tag_detections_image_publisher_ = it_->advertise("camera" + std::to_string(camera_index) + "/tag_detections", 1);
  }

  image_sub_ = it_->subscribe("image_rect", queue_size,
                              &DetectorStream::imageCallback, this);
}

void DetectorStream::imageCallback(
    const sensor_msgs::ImageConstPtr &image_rect) {
  std::scoped_lock<std::mutex> lock(detection_mutex_);
  // Lazy updates:
  // When there are no subscribers _and_ when tf is not published,
  // skip detection.
  // if (tag_detections_publisher_.getNumSubscribers() == 0 &&
  //     tag_detections_image_publisher_.getNumSubscribers() == 0 &&
  //     !tag_detector_->get_publish_tf()) {
  //   // ROS_INFO_STREAM("No subscribers and no tf publishing, skip
  //   processing."); return;
  // }

  // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
  // AprilTag 2 on the iamge
  try {
    cv_image_ = cv_bridge::toCvCopy(image_rect, image_rect->encoding);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Publish detected tags in the image by AprilTag 2
  clock_t time_1 = std::clock();
  tag_detections_publisher_.publish(tag_detector_->detectTags(
      cv_image_, sensor_msgs::CameraInfoConstPtr(
                     new sensor_msgs::CameraInfo(camera_info_))));
  clock_t time_2 = std::clock();
  double time_taken = double(time_2 - time_1) / double(CLOCKS_PER_SEC);
  ROS_INFO_STREAM("Detection takes" << time_taken << std::endl);
  // Publish the camera image overlaid by outlines of the detected tags and
  // their payload values
  if (draw_tag_detections_image_) {
    tag_detector_->drawDetections(cv_image_);
    tag_detections_image_publisher_.publish(cv_image_->toImageMsg());
  }
}

} // namespace apriltag_ros
