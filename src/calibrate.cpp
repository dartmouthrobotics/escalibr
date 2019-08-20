/**
*
* \author     Monika Roznere <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#include "escalibr/echo_sounder_camera_handler.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "echo_sounder_camera_calibration");

  YAML::Node config = YAML::LoadFile(argv[1]);

  // ROS topics for imagery and echo sounder measurements
  std::string camera_topic = config["camera_topic"].as<std::string>();
  std::string echo_sounder_topic = config["echo_sounder_topic"].as<std::string>();

  int calibration_method = config["calibration_method"].as<int>();

  float min_depth = config["min_depth"].as<float>();
  float max_depth = config["max_depth"].as<float>();
  float ellipse_depth_range = config["ellipse_depth_range"].as<float>();

  escalibr::EchoSounderCameraHandler handler(calibration_method, camera_topic, echo_sounder_topic, min_depth, max_depth, ellipse_depth_range);

  ros::spin();
  return 0;
}
