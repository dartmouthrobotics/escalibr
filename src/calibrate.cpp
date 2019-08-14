/**
*
* \author     Monika Roznere <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#include "escalibr/EchoSounderCameraHandler.h"

#include <ros/ros.h>
#include <iostream>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "echo_sounder_camera_calibration");

  // TO DO: retrieved from YAML file for ease
  std::string camera_topic = "/";
  std::string echo_sounder_topic = "/";
  int calibration_method = 0;  // 0: super coarse

  // TO DO: use min and max for possible depth range? to ignore far off distances
  float min = 0.40;
  float max = 0.70;

  escalibr::EchoSounderCameraHandler handler(calibration_method, camera_topic, echo_sounder_topic, min, max);

  ros::spin();
  return 0;
}
