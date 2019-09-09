/**
*
* \author     Monika Roznere <monika.roznere.gr@dartmouth.edu> <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#include "escalibr/echo_sounder_camera_handler.h"
#include "escalibr/calibration_math.h"
#include "escalibr/calibration_gui.h"
#include "escalibr/image_proc.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <string>
#include <algorithm>
#include <vector>


namespace escalibr
{

EchoSounderCameraHandler::EchoSounderCameraHandler(int calibration_method, std::string camera_topic,
  std::string echo_sounder_topic, float min_detect_depth, float max_detect_depth, float detect_depth_range,
  float min_echo_depth, float max_echo_depth, int min_confidence_level, bool save_images)
{
  // Camera calibration parameters
  this->INTRINSIC_DATA_ = (cv::Mat_<float>(3, 3) <<
    1066.342505, 0.000000, 940.432119,
    0.000000, 1065.381509, 552.184409,
    0.000000, 0.000000, 1.000000);
  this->DIST_COEFFS_DATA_ = (cv::Mat_<float>(1, 5) << 0.018838, -0.006986, -0.003403, -0.002292, 0.000000);

  // Information needed to collect edge point data
  this->MIN_DETECT_DEPTH_ = min_detect_depth;
  this->MAX_DETECT_DEPTH_ = max_detect_depth;
  this->DETECT_DEPTH_RANGE_ = detect_depth_range;
  this->MIN_ECHO_DEPTH_ = min_echo_depth;
  this->MAX_ECHO_DEPTH_ = max_echo_depth;
  this->MIN_CONFIDENCE_LEVEL_ = min_confidence_level;

  // Information needed when detecting the sphere structure in the image
  this->prev_structure_detected_ = false;
  this->counter_ = 0;
  this->FINAL_HOLD_COUNT_ = 150;  // TO DO: intialized by YAML file

  // Save images for missed and collected edge points
  this->SAVE_IMAGES_ = save_images;

  // Information that the GUI needs about the range of data that will be collected
  float range = this->MAX_DETECT_DEPTH_ - this->MIN_DETECT_DEPTH_;
  int intervals = static_cast<int>(range / this->DETECT_DEPTH_RANGE_);
  this->calib_gui.visual_point_data_.resize(intervals, std::vector<cv::Point>());
  // TO DO: seems redundant to also save the values in the calibration gui class
  this->calib_gui.MIN_DEPTH_ = this->MIN_DETECT_DEPTH_;
  this->calib_gui.MAX_DEPTH_ = this->MAX_DETECT_DEPTH_;

  this->img_sub_.subscribe(nh_, camera_topic, 1);
  this->echo_sounder_sub_.subscribe(nh_, echo_sounder_topic, 1);

  this->sync_.reset(new Sync(SyncPolicy(5), this->img_sub_, this->echo_sounder_sub_));

  // TO DO: unknown if there will be other calibration methods
  if (0 == calibration_method)
  {
    std::cout << std::endl << "ESCALIBR CALIBRATION" << std::endl << std::endl;

    this->sync_->registerCallback(boost::bind(&EchoSounderCameraHandler::echoSounderAndImageCallback, this, _1, _2));
  }
}


void EchoSounderCameraHandler::echoSounderAndImageCallback(const sensor_msgs::ImageConstPtr& img_msg,
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

  // Undistort image
  cv::Mat cur_image;
  // TO DO: need to do something more intuitive
  cv::undistort(cv_ptr->image, cur_image, this->INTRINSIC_DATA_, this->DIST_COEFFS_DATA_);

  std::string message = "NO DETECTION";

  // Ignore instances of noise coming from echo sounder messages
  if (echo_sounder_msg->distance > this->MIN_ECHO_DEPTH_ && echo_sounder_msg->distance < this->MAX_ECHO_DEPTH_)
  {
    // If user has pressed play then start collecting data
    if (this->calib_gui.play)
    {
      // If --
      //    structure was detected for at least a set amount of rounds AND
      //    structure was detected previously AND
      //    EITHER
      //        no structure detected OR
      //        confidece level is less than minimum confidence level
      // then try to detect sphere in previous instance (image and depth)
      if (this->FINAL_HOLD_COUNT_ <= this->counter_ && this->prev_structure_detected_ &&
         (this->MAX_DETECT_DEPTH_ < echo_sounder_msg->distance ||
          this->MIN_CONFIDENCE_LEVEL_ > static_cast<int>(echo_sounder_msg->confidence)))
      {
        // Locate sphere in image
        cv::KeyPoint sphere = this->image_proc.detectSphere(this->prev_image_);
        // Calculate 3D point of sphere's center and store its information
        bool detect_result = this->math_handler.detectEdgePoint(this->prev_depth_, sphere);

        // If successful, then make sure GUI knows to show it
        if (detect_result)
        {
          // Get index from distance value and store sphere's center point in a data structure
          int d_ind = static_cast<int>((this->prev_depth_ - this->MIN_DETECT_DEPTH_) / this->DETECT_DEPTH_RANGE_);
          this->calib_gui.visual_point_data_.at(d_ind).push_back(sphere.pt);
        }

        if (this->SAVE_IMAGES_)
        {
          saveImage(this->prev_image_, sphere, detect_result);
        }

        this->counter_ = 0;
      }

      // If --
      //    structure detected in range AND
      //    confidence level is equal to or greater than the minimum confidence level
      // then setup for hold or continue to hold
      else if (this->MIN_DETECT_DEPTH_ < echo_sounder_msg->distance &&
        echo_sounder_msg->distance < this->MAX_DETECT_DEPTH_ &&
        this->MIN_CONFIDENCE_LEVEL_ <= echo_sounder_msg->confidence)
      {
        // Begin counter for the structure to be detected for a period
        if (!this->prev_structure_detected_)
        {
          this->prev_structure_detected_ = true;
          this->counter_ = 1;
          message = "STRUCTURE DETECTED";
        }
        else
        {
          // Hold complete
          if (this->FINAL_HOLD_COUNT_ < this->counter_)
          {
            message = "FINISHED HOLD";
          }
          // Continue to hold
          else
          {
            message = "HOLD: " + std::to_string(this->FINAL_HOLD_COUNT_ - this->counter_);
            this->counter_++;
          }
        }

        // Save image for potential edge point detection
        this->prev_image_ = cur_image;
        this->prev_depth_ = echo_sounder_msg->distance;
      }
      // No structure detected
      else
      {
        // Clear parameters
        if (this->prev_structure_detected_)
        {
          this->prev_structure_detected_ = false;
          this->counter_ = 0;
        }
      }
    }
    // User has calibration on pause
    else
    {
      // Pause should clear parameters
      this->prev_structure_detected_ = false;
      this->counter_ = 0;
    }

    // Outline sphere in image if detected and in debug mode
    cv::KeyPoint sphere = cv::KeyPoint();
    if (this->calib_gui.debug && this->counter_ > 0)
    {
      sphere = this->image_proc.detectSphere(cur_image);
    }

    // Get index in collected point data structure for the current depth reading
    int d_ind = static_cast<int>((this->prev_depth_ - this->MIN_DETECT_DEPTH_) / this->DETECT_DEPTH_RANGE_);

    // Display information in GUI window
    this->calib_gui.run(cur_image, sphere, echo_sounder_msg->distance,
      echo_sounder_msg->confidence, message, d_ind);
  }
}


void EchoSounderCameraHandler::saveImage(cv::Mat image, cv::KeyPoint sphere, bool detect_result)
{
  if (detect_result)  // Saving successful detection of sphere
  {
    // Draw sphere
    cv::circle(image, sphere.pt, sphere.size/2, cv::Scalar(0, 0, 255), 3);
    std::string filename = "/home/darobot/Research/escalibr/catkin_ws/src/escalibr/collected_data/image_" +
      std::to_string(this->math_handler.collected_data_points_) + ".jpg";
    cv::imwrite(filename, image);
  }
  else  // Saving failed detection of sphere
  {
    std::string filename = "/home/darobot/Research/escalibr/catkin_ws/src/escalibr/missed_data/image_" +
      std::to_string(this->math_handler.missed_data_points_) + ".jpg";
    cv::imwrite(filename, image);
  }
}


// TO DO: save to a yaml file as well
void EchoSounderCameraHandler::saveResult(std::vector<float> result)
{
  std::cout << "x: " << result[0] << std::endl;
  std::cout << "y: " << result[1] << std::endl;
  std::cout << "z: " << result[2] << std::endl;
  std::cout << "pitch: " << result[3] << std::endl;
  std::cout << "yaw: " << result[4] << std::endl;
}

}  // namespace escalibr
