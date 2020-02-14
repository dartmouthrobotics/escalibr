/**
*
* \author     Monika Roznere <monika.roznere.gr@dartmouth.edu> <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#ifndef ESCALIBR_ECHO_SOUNDER_CAMERA_HANDLER_H
#define ESCALIBR_ECHO_SOUNDER_CAMERA_HANDLER_H

#include "escalibr/calibration_math.h"
#include "escalibr/calibration_gui.h"
#include "escalibr/image_proc.h"

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
// #include <Eigen/Dense>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <ping_nodelet/Ping.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace escalibr
{

/** Echo sounder camera handler class.
 *  Handles ROS messages (images and echo sounder measurements), detects
 *  data points and calculates echo sounder transformation.
 */

class EchoSounderCameraHandler
{
public:
  /** Constructor.
   *  Initializes the parameters and sets up the ROS subscribers and callback.
   *
   *  \param method sets up which calibration method to use.
   *  \param camera_topic is the name of the topic for camera images.
   *  \param echo_sounder_topic is the name of the topic for echo sounder measurements.
   *  \param min_detect_depth - look at MIN_DETECT_DEPTH_.
   *  \param max_detect_depth - look at MAX_DETECT_DEPTH_.
   *  \param detect_depth_range - look at DETECT_DEPTH_RANGE_.
   *  \param save_images - look at SAVE_IMAGES_.
   */
  EchoSounderCameraHandler(int calibration_method, std::string camera_topic,
    std::string echo_sounder_topic, float min_detect_depth, float max_detect_depth,
    float detect_depth_range, float min_echo_depth, float max_echo_depth, int min_confidence_level,
    bool save_images);
  ~EchoSounderCameraHandler() {}

private:
  ros::NodeHandle nh_;

  CalibrationMath math_handler;     /**< handles circle detection, ellipse math, and echo sounder transformation */
  CalibrationGui calib_gui;     /**< handles the GUI that shows images and data collection */
  ImageProc image_proc;         /**< handles image processing needed: sphere detection */

  /** Counter parameters.
   *  Used to make sure that data points are legit and not random noise.
   */
  int counter_;                 /**< for tracking how long the structure is detected */
  int FINAL_HOLD_COUNT_;        /**< after this count, the algorithm will accept data points */

  /** Data collection parameters.
   *  Used for determining acceptable data points and ellipses.
   */
  float MIN_ECHO_DEPTH_;         /**< minimum depth that the echo sounder can detect at */
  float MAX_ECHO_DEPTH_;         /**< maximum depth that the echo sounder can detect at */
  float MIN_DETECT_DEPTH_;       /**< minimum depth to consider the structure as detected */
  float MAX_DETECT_DEPTH_;       /**< maximum depth to consider the structure as detected */
  float DETECT_DEPTH_RANGE_;     /**< data points will be grouped by range in data structure */
  int MIN_CONFIDENCE_LEVEL_;     /**< minimum confidence level deemed to be still good for detection */

  /** Image parameters.
   *  Used for detecting data points.
   */
  bool prev_structure_detected_;  /**< checks if the structure was detected previously */
  cv::Mat prev_image_;            /**< previous image; used for circle and data point detection */
  float prev_depth_;              /**< previous (last) depth measurement while the structure was still detected */

  /** Camera calibration parameters.
    * Used for undistorting the incoming images
    */
  cv::Mat INTRINSIC_DATA_;
  cv::Mat DIST_COEFFS_DATA_;

  bool SAVE_IMAGES_;             /**< boolean to set if images of successful/failed detections will be saved or not. */

  message_filters::Subscriber<sensor_msgs::Image> img_sub_;
  message_filters::Subscriber<ping_nodelet::Ping> echo_sounder_sub_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, ping_nodelet::Ping> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;

  /** Callback for image and echo sounder topics that handles callibration.
   *  Handles the detection of data points and echo sounder transformation.
   *
   *  \param img_msg is the message from the camera/image topic.
   *  \param echo_sounder_msg is the message from the echo sounder topic.
   */
  void echoSounderAndImageCallback(const sensor_msgs::ImageConstPtr& img_msg,
    const ping_nodelet::Ping::ConstPtr& echo_sounder_msg);


  /** Function for saving images of successful and failed detection attempts
   *  Saves to images to their specific folders.
   *
   *  \param image is the image to saved.
   *  \param sphere is the center point of the sphere that was successfully detected.
   *  \param detect_result is the boolean to identify if the detection was a success or a failure.
   */
  void saveImage(cv::Mat image, cv::KeyPoint sphere, bool detect_result);

  /** Function for saving and printing calibration results.
   *  Prints and saves to YAML file the X, Y, Z, pitch, and yaw of the echo sounder.
   *
   *  \param result is the X, Y, Z, pitch, and yaw of the echo sounder.
   */
  void saveResult(std::vector<float> result);
};

}  // namespace escalibr

#endif  // ESCALIBR_ECHO_SOUNDER_CAMERA_HANDLER_H
