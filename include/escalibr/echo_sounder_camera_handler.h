/**
*
* \author     Monika Roznere <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#ifndef ESCALIBR_ECHO_SOUNDER_CAMERA_HANDLER_H
#define ESCALIBR_ECHO_SOUNDER_CAMERA_HANDLER_H

#include "escalibr/ellipse_math.h"

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <ping_nodelet/Ping.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace escalibr
{

class EchoSounderCameraHandler
{
public:
  EchoSounderCameraHandler(int calibration_method, std::string camera_topic,
    std::string echo_sounder_topic, float min_depth, float max_depth, float ellipse_depth_range);
  ~EchoSounderCameraHandler() {}

private:
  ros::NodeHandle nh_;
  EllipseMath math_handler;

  // Counter used to make sure that data points are not coming from random depth measurements
  int counter_;
  int FINAL_HOLD_COUNT_ = 50;

  // Information on acceptable data points and ellipses
  float min_depth_;
  float max_depth_;
  float ellipse_depth_range_;  // depth range for a singular ellipse

  // Previous image currently used for calculating data points
  bool prev_structure_detected_;
  cv::Mat prev_image_;

  message_filters::Subscriber<sensor_msgs::Image> img_sub_;
  message_filters::Subscriber<ping_nodelet::Ping> echo_sounder_sub_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, ping_nodelet::Ping> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;

  void coarseCallback(const sensor_msgs::ImageConstPtr& img_msg,
    const ping_nodelet::Ping::ConstPtr& echo_sounder_msg);

  void saveResult(std::vector<float> result);
};

}  // namespace escalibr

#endif  // ESCALIBR_ECHO_SOUNDER_CAMERA_HANDLER_H
