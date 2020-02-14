/**
*
* \author     Monika Roznere <monika.roznere.gr@dartmouth.edu> <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#ifndef ESCALIBR_ECHO_SOUNDER_CAMERA_EXTRACTOR_H
#define ESCALIBR_ECHO_SOUNDER_CAMERA_EXTRACTOR_H

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
// #include <Eigen/Dense>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <ping_nodelet/Ping.h>
#include <ORB_SLAM2/Points.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace escalibr
{

/** Echo sounder camera handler class.
 *  Handles ROS messages (images and echo sounder measurements), detects
 *  data points and calculates echo sounder transformation.
 */

class EchoSounderCameraExtractor
{
public:

  EchoSounderCameraExtractor(std::string camera_topic, std::string echo_sounder_topic, std::string orb_slam_topic);
  ~EchoSounderCameraExtractor() {}

private:
  ros::NodeHandle nh_;

  cv::Scalar echo_pose;
  cv::Scalar echo_orient;

  /** Data collection parameters.
   *  Used for determining acceptable data points and ellipses.
   */
  float MIN_ECHO_DEPTH_;         /**< minimum depth that the echo sounder can detect at */
  float MAX_ECHO_DEPTH_;         /**< maximum depth that the echo sounder can detect at */


  /** Camera calibration parameters.
    * Used for undistorting the incoming images
    */
  cv::Mat INTRINSIC_DATA_;
  cv::Mat DIST_COEFFS_DATA_;
  cv::Mat PROJECTION_DATA_;


  message_filters::Subscriber<sensor_msgs::Image> img_sub_;
  message_filters::Subscriber<ping_nodelet::Ping> echo_sounder_sub_;
  message_filters::Subscriber<ORB_SLAM2::Points> orb_slam2_sub_;

  ros::Publisher pub_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, ping_nodelet::Ping, ORB_SLAM2::Points> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;

  void echoSounderAndImageCallback(const sensor_msgs::ImageConstPtr& img_msg,
    const ping_nodelet::Ping::ConstPtr& echo_sounder_msg,
    const ORB_SLAM2::Points::ConstPtr& orb_slam2_msg);

};

}  // namespace escalibr

#endif  // ESCALIBR_ECHO_SOUNDER_CAMERA_EXTRACTOR_H
