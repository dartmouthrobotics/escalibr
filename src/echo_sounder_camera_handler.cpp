/**
*
* \author     Monika Roznere <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#include "escalibr/echo_sounder_camera_handler.h"
#include "escalibr/ellipse_math.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <string>
#include <algorithm>
#include <vector>


namespace escalibr
{

EchoSounderCameraHandler::EchoSounderCameraHandler(int calibration_method, std::string camera_topic,
  std::string echo_sounder_topic, float min_depth, float max_depth, float ellipse_depth_range)
{
  this->min_depth_ = min_depth;
  this->max_depth_ = max_depth;
  this->ellipse_depth_range_ = ellipse_depth_range;
  this->prev_structure_detected_ = false;
  this->counter_ = 0;

  std::cout << "in camera handler" << std::endl;

  this->img_sub_.subscribe(nh_, camera_topic, 1);
  this->echo_sounder_sub_.subscribe(nh_, echo_sounder_topic, 1);

  this->sync_.reset(new Sync(SyncPolicy(1), this->img_sub_, this->echo_sounder_sub_));

  if (0 == calibration_method)
  {
    std::cout << "SUPER COARSE CALIBRATION" << std::endl;
    this->sync_->registerCallback(boost::bind(&EchoSounderCameraHandler::coarseCallback, this, _1, _2));

    // std::string path = ros::package::getPath("underwater_color_enhance") + "/ros_config.yaml";
    this->prev_image_ = cv::imread("/home/darobot/Research/escalibr/catkin_ws/src/escalibr/circle.png");
  }
}


// Callback to implement the super coarse calibration using the camera and echo sounder data
void EchoSounderCameraHandler::coarseCallback(const sensor_msgs::ImageConstPtr& img_msg,
  const ping_nodelet::Ping::ConstPtr& echo_sounder_msg)
{
  // Convert ROS sensor image message to CV Mat image
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

  // Detect edge point

  // If no structure detected now and structure was detected beforehand
  // Also, structure must have been detected for at least set amount of rounds
  if (this->FINAL_HOLD_COUNT_ <= this->counter_ && this->max_ < echo_sounder_msg->distance
    && this->prev_structure_detected_)
  {
    // Locate edge point and store its 2D pixel coordiante with its image depth measurement
    this->math_handler.detectEdgePoint(echo_sounder_msg->distance, this->prev_image_);

    // TO DO: show images with the detected points in red
    // cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
    // cv::circle(this->prev_image, center, 3, cv::Scalar(0,255,0), -1, 8, 0);

    // Check for ellipse
    int pointer = this->math_handler.checkEllipse(this->range_);
    if (-1 != pointer)  // There is enough data to fit an ellipse
    {
      std::cout << std::endl << "NEW ELLIPSE" << std::endl;

      // Fit an ellipse and then save the data to a vector
      this->math_handler.fitEllipse(pointer);

      // Two ellipses are enough to get params for echo sounder
      if (2 == this->math_handler.getEllipseDataSize())
      {
        // Calculate echo sounder pose and orientation
        std::vector<float> result = this->math_handler.getEchoSounderTf();

        // Save echo sounder pose and orientation
        saveResult(result);

        exit(EXIT_SUCCESS);
      }
      else  // Need one more ellipse
      {
        // Reset
        this->prev_structure_detected_ = false;
      }
    }
  }
  // Structure detected
  else if (this->min_ < echo_sounder_msg->distance < this->max_)
  {
    // Begin counter for the structure to be detected for a period
    if (!this->prev_structure_detected_)
    {
      this->prev_structure_detected_ = true;
      std::cout << std::endl << "STRUCTURE DETECTED: please hold" << std::endl;
      this->counter_ = 0;
    }
    else
    {
      // Hold complete
      if (this->FINAL_HOLD_COUNT_ >= this->counter_)
      {
        std::cout << std::endl << "FINISHED HOLD: move structue out of bounds" << std::endl;
      }
      // Continue to hold
      else
      {
        std::cout << std::endl << "CONTINUE TO HOLD" << std::endl;
        this->counter_++;
      }
    }

    // Save image for potential edge point detection
    this->prev_image_ = cv_ptr->image;
  }
  // No structure detected
  else
  {
    std::cout << std::endl << "NO DETECTION" << std::endl;
    this->counter_ = 0;
  }
}



void EchoSounderCameraHandler::saveResult(std::vector<float> result)
{
  // TO DO: save to a yaml file as well

  std::cout << "x: " << result[0] << std::endl;
  std::cout << "y: " << result[1] << std::endl;
  std::cout << "x: " << result[2] << std::endl;
  std::cout << "pitch: " << result[3] << std::endl;
  std::cout << "yaw: " << result[4] << std::endl;
}

}  // namespace escalibr
