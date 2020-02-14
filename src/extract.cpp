/**
*
* \author     Monika Roznere <monika.roznere.gr@dartmouth.edu> <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#include "escalibr/echo_sounder_camera_extractor.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <string>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "echo_sounder_camera_extraction");

  std::string camera_topic = "/camera/image_raw";
  std::string echo_sounder_topic = "/ping_nodelet/ping";
  std::string orb_slam_topic = "/orb_slam2/data";

  escalibr::EchoSounderCameraExtractor handler(camera_topic, echo_sounder_topic, orb_slam_topic);

  ros::spin();

  return 0;
}
