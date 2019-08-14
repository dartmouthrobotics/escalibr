/**
*
* \author     Monika Roznere <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#ifndef ESCALIBR_ECHOSOUNDERCAMERAHANDLER_H
#define ESCALIBR_ECHOSOUNDERCAMERAHANDLER_H

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

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
  EchoSounderCameraHandler(int calibration_method, std::string camera_topic, std::string echo_sounder_topic, float min, float max);
  ~EchoSounderCameraHandler() {};

private:
  ros::NodeHandle nh_;

  float min;  // minumum valid depth value
  float max;  // maximum valid depth value
  bool prev_structure_detected;
  std::vector<cv::Point3f> edge_point_data;

  message_filters::Subscriber<sensor_msgs::Image> img_sub_;
  message_filters::Subscriber<ping_nodelet::Ping> echo_sounder_sub_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, ping_nodelet::Ping> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  boost::shared_ptr<Sync> sync;

  void super_coarse_callback(const sensor_msgs::ImageConstPtr& img_msg,
    const ping_nodelet::Ping::ConstPtr& echo_sounder_msg);

  bool check_ellipse();
  void create_ellipse();
};

}  // namespace escalibr

#endif  // ESCALIBR_ECHOSOUNDERCAMERAHANDLER_H
