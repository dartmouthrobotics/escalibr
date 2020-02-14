/**
*
* \author     Monika Roznere <monika.roznere.gr@dartmouth.edu> <mroznere@gmail.com>
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
  bool save_images = config["save_images"].as<bool>();

  float min_detect_depth = config["min_detect_depth"].as<float>();
  float max_detect_depth = config["max_detect_depth"].as<float>();
  float detect_depth_range = config["detect_depth_range"].as<float>();
  float min_echo_depth = config["min_echo_depth"].as<float>();
  float max_echo_depth = config["max_echo_depth"].as<float>();
  int min_confidence_level = config["min_confidence_level"].as<float>();

  escalibr::EchoSounderCameraHandler handler(calibration_method, camera_topic,
    echo_sounder_topic, min_detect_depth, max_detect_depth, detect_depth_range,
    min_echo_depth, max_echo_depth, min_confidence_level, save_images);

  ros::spin();

  return 0;
}
