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
  float range;  // acceptable range for an ellipse
  bool prev_structure_detected;
  cv::Mat prev_image;
  std::vector<cv::Point3f> edge_point_data;
  std::vector< std::vector<float> > ellipse_data;

  message_filters::Subscriber<sensor_msgs::Image> img_sub_;
  message_filters::Subscriber<ping_nodelet::Ping> echo_sounder_sub_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, ping_nodelet::Ping> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  boost::shared_ptr<Sync> sync;

  void super_coarse_callback(const sensor_msgs::ImageConstPtr& img_msg,
    const ping_nodelet::Ping::ConstPtr& echo_sounder_msg);

  int check_ellipse();
  void create_ellipse(int pointer);
  std::vector<float> get_echo_sounder_tf();
  void save_result(std::vector<float> result);
};

}  // namespace escalibr

#endif  // ESCALIBR_ECHOSOUNDERCAMERAHANDLER_H
