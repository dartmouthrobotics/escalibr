/**
*
* \author     Monika Roznere <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#include "escalibr/EchoSounderCameraHandler.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


namespace escalibr
{

struct AppEllipse
{
  bool operator() (cv::Point3f pt1, cv::Point3f pt2) {return (pt1.z < pt2.z);}
} appEllipse;

EchoSounderCameraHandler::EchoSounderCameraHandler(int calibration_method, std::string camera_topic,
  std::string echo_sounder_topic, float min, float max)
{
  this->min = min;
  this->max = max;
  this->prev_structure_detected = false;

  this->img_sub_.subscribe(nh_, camera_topic, 1);
  this->echo_sounder_sub_.subscribe(nh_, echo_sounder_topic, 1);

  this->sync.reset(new Sync(SyncPolicy(1), this->img_sub_, this->echo_sounder_sub_));

  if(0 == calibration_method)
  {
    std::cout << "SUPER COARSE CALIBRATION" << std::endl;
    this->sync->registerCallback(boost::bind(&EchoSounderCameraHandler::super_coarse_callback, this, _1, _2));
  }
}


// callback to implement the super coarse calibration using the camera and echo sounder data
void EchoSounderCameraHandler::super_coarse_callback(const sensor_msgs::ImageConstPtr& img_msg,
  const ping_nodelet::Ping::ConstPtr& echo_sounder_msg)
{

  // conver image message to CV image
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // detect edge point
  // if no structure detected and structure was detected beforehand
  if (this->min < echo_sounder_msg->distance < this->max && prev_structure_detected)
  {
    // grab edge point

    // well imagine that we got the pixel x and y coordinate for the center of the structure.
    int x = 20;
    int y = 50;

    cv::Point3f edge_point(x, y, echo_sounder_msg->distance);
    this->edge_point_data.push_back(edge_point);

    // check for ellipse
    if (check_ellipse())
    {
      create_ellipse();
    }

    //
  }
}


bool EchoSounderCameraHandler::check_ellipse()
{
  if (5 <= this->edge_point_data.size())
  {
    return false;
  }

  // sort vector by depth measurement
  std::sort(this->edge_point_data.begin(), this->edge_point_data.end(), appEllipse);

  // check if 5 points exist within range

  return false;
}


void EchoSounderCameraHandler::create_ellipse()
{
  int i = 1;
}


}  // namespace escalibr
