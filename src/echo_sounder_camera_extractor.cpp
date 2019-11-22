/**
*
* \author     Monika Roznere <monika.roznere.gr@dartmouth.edu> <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#include "escalibr/echo_sounder_camera_extractor.h"

#include <geometry_msgs/Point.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <string>
#include <algorithm>
#include <vector>


namespace escalibr
{

EchoSounderCameraExtractor::EchoSounderCameraExtractor(std::string camera_topic,
  std::string echo_sounder_topic, std::string orb_slam_topic)
{
  this->echo_pose = cv::Scalar(-0.136, 0.061, 0.044);
  this->echo_orient = cv::Scalar(0.10904085, -0.13293064, 0.9851089);

  // Camera calibration parameters
  this->INTRINSIC_DATA_ = (cv::Mat_<float>(3, 3) <<
    1066.342505, 0.000000, 940.432119,
    0.000000, 1065.381509, 552.184409,
    0.000000, 0.000000, 1.000000);
  this->DIST_COEFFS_DATA_ = (cv::Mat_<float>(1, 5) << 0.018838, -0.006986, -0.003403, -0.002292, 0.000000);
  this->PROJECTION_DATA_ = (cv::Mat_<float>(3,4) <<
    1082.510864, 0.000000, 934.229239, 0.000000,
    0.000000, 1082.330933, 546.763104, 0.000000,
    0.000000, 0.000000, 1.000000, 0.000000);

  this->MIN_ECHO_DEPTH_ = 0.4;
  this->MAX_ECHO_DEPTH_ = 30.0;

  this->img_sub_.subscribe(this->nh_, camera_topic, 1);
  this->echo_sounder_sub_.subscribe(this->nh_, echo_sounder_topic, 1);
  this->orb_slam2_sub_.subscribe(this->nh_, orb_slam_topic, 1);

  this->pub_ = this->nh_.advertise<ORB_SLAM2::Points>("/orb_slam2/escalibr_data", 1);

  this->sync_.reset(new Sync(SyncPolicy(15), this->img_sub_, this->echo_sounder_sub_, this->orb_slam2_sub_));

  std::cout << std::endl << "ESCALIBR EXTRACTION" << std::endl << std::endl;

  this->sync_->registerCallback(boost::bind(&EchoSounderCameraExtractor::echoSounderAndImageCallback, this, _1, _2, _3));
}


void EchoSounderCameraExtractor::echoSounderAndImageCallback(const sensor_msgs::ImageConstPtr& img_msg,
  const ping_nodelet::Ping::ConstPtr& echo_sounder_msg, const ORB_SLAM2::Points::ConstPtr& orb_slam2_msg)
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

  // Ignore instances of noise coming from echo sounder messages
  if (echo_sounder_msg->distance > this->MIN_ECHO_DEPTH_ && echo_sounder_msg->distance < this->MAX_ECHO_DEPTH_)
  {

    if (echo_sounder_msg->confidence == 100 && echo_sounder_msg->distance < 2.5)
    {
      // std::cout << std::endl << "distance: " << echo_sounder_msg->distance << std::endl;

      // Undistort image
      cv::Mat cur_image;
      // TO DO: need to do something more intuitive
      cv::undistort(cv_ptr->image, cur_image, this->INTRINSIC_DATA_, this->DIST_COEFFS_DATA_);

      // Calculate 3D point on cone axis with echo sounder measured distance away
      cv::Scalar middle_point = this->echo_pose + echo_sounder_msg->distance * this->echo_orient;
      // std::cout << "orient: " << this->echo_orient << std::endl;
      // std::cout << "mult: " <<  echo_sounder_msg->distance * this->echo_orient << std::endl;
      // std::cout << "point: " << middle_point << std::endl;


      // Calculate cone base radius
      float base_radius = echo_sounder_msg->distance * tan(0.523599 / 2);
      // std::cout << "cone radius: " << base_radius << std::endl;

      cv::Point3d point_center = cv::Point3d(middle_point[0], middle_point[1], middle_point[2]);
      cv::Point3d point_N = point_center + cv::Point3d(0, -base_radius, 0);
      cv::Point3d point_S = point_center + cv::Point3d(0, base_radius, 0);
      cv::Point3d point_E = point_center + cv::Point3d(base_radius, 0, 0);
      cv::Point3d point_W = point_center + cv::Point3d(-base_radius, 0, 0);

      // std::cout << "north point: " << point_N << std::endl;
      // std::cout << "south point: " << point_S << std::endl;
      // std::cout << "east point: " << point_E << std::endl;
      // std::cout << "west point: " << point_W << std::endl;

      // convert points to pixel points
      std::vector<cv::Point3d> objectPoints;
      objectPoints.push_back(point_center);
      objectPoints.push_back(point_N);
      objectPoints.push_back(point_S);
      objectPoints.push_back(point_E);
      objectPoints.push_back(point_W);

      std::vector<cv::Point2d> imagePoints;
      // cv::Mat rVec = cv::Mat::zeros(cv::Size(3, 1), cv::DataType<double>::type);
      // cv::Mat tVec = cv::Mat::zeros(cv::Size(3, 1), cv::DataType<double>::type);
      //
      // cv::projectPoints(objectPoints, rVec, tVec, this->INTRINSIC_DATA_, this->DIST_COEFFS_DATA_, imagePoints);


      for (int i = 0; i < objectPoints.size(); i ++)
      {
        cv::Mat point = (cv::Mat_<float>(4, 1) << objectPoints.at(i).x, objectPoints.at(i).y, objectPoints.at(i).z, 0.000000);
        // std::cout << this->PROJECTION_DATA_ << std::endl;
        cv::Mat rectify = this->PROJECTION_DATA_ * point;
        float pixel_x = rectify.at<float>(0,0) / rectify.at<float>(2,0);
        float pixel_y = rectify.at<float>(1,0) / rectify.at<float>(2,0);
        imagePoints.push_back(cv::Point2d(pixel_x, pixel_y));
        // std::cout << "x: " << pixel_x << " y: " << pixel_y << std::endl;
      }


      float circle_radius = 0;
      for(int i = 1; i < imagePoints.size(); i++)
      {
        float new_dist = sqrt(pow(imagePoints.at(0).x - imagePoints.at(i).x, 2) +
                              pow(imagePoints.at(0).y - imagePoints.at(i).y, 2));
        // std::cout << "new_dist: " << new_dist << std::endl;
        if (new_dist > circle_radius)
        {
          circle_radius = new_dist;
        }
      }

      cv::Point2d center = imagePoints.at(0);
      // std::cout << "circle radius: " << circle_radius << std::endl;

      cv::circle(cur_image, center, int(circle_radius), cv::Scalar(0, 0, 255), 3);


      // Identify ORB_SLAM2 points within the circle
      // std::vector<cv::Point2d> slam_circle_points;
      std::vector<float> new_distances;
      cv::Point2d detected_point;
      float detected_depth = 100.0;
      for (int i = 0; i < sizeof(orb_slam2_msg->points); i++)
      {
        float temp_dist = sqrt(pow(center.x - orb_slam2_msg->points[i].x, 2) +
                               pow(center.y - orb_slam2_msg->points[i].y, 2));
        // std::cout << orb_slam2_msg->points[i] << std::endl;
        if (temp_dist <= circle_radius)
        {
          if (orb_slam2_msg->distances[i] < detected_depth)
          {
            detected_point = cv::Point2d(orb_slam2_msg->points[i].x, orb_slam2_msg->points[i].y);
            detected_depth = orb_slam2_msg->distances[i];
          }
          // slam_circle_points.push_back(cv::Point(orb_slam2_msg->points[i].x, orb_slam2_msg->points[i].y));
        }

        new_distances.push_back(orb_slam2_msg->distances[i]);
      }

      // std::cout << "Detected slam point" << std::endl;
      // std::cout << detected_point << std::endl;
      // std::cout << detected_depth << std::endl;

      float ratio = echo_sounder_msg->distance / detected_depth;
      // std::cout << "ratio: " << ratio << std::endl;
      // std::cout << "depth: " << detected_depth * ratio << std::endl;


      ORB_SLAM2::Points msg;
      msg.header.stamp = img_msg->header.stamp;
      // msg.points = orb_slam2_msg->points;

      for (int i = 0; i < new_distances.size(); i++)
      {
        // std::cout << std::endl << "before: " << new_distances[i] << std::endl;
        new_distances[i] = new_distances[i] * ratio;
        // std::cout << "after: " << new_distances[i] << std::endl;
        msg.distances.push_back(new_distances[i]);

        geometry_msgs::Point cur_point;
        cur_point.x = orb_slam2_msg->points[i].x;
        cur_point.y = orb_slam2_msg->points[i].y;
        cur_point.z = 0;
        msg.points.push_back(cur_point);

        cv::circle(cur_image, cv::Point(orb_slam2_msg->points[i].x, orb_slam2_msg->points[i].y), 3, cv::Scalar(0, 0, 255), 3);
      }

      // std::cout << "size of points " << sizeof(orb_slam2_msg->points) << std::endl;
      // std::cout << "size of distances " << new_distances.size() << std::endl;
      //
      // for (int i = 0; i < new_distances.size(); i++)
      // {
      //   std::cout << orb_slam2_msg->points[i].x << ", " << orb_slam2_msg->points[i].y << "," << std::endl;
      // }
      //
      // std::cout << std::endl << std::endl << std::endl;
      //
      // for (int i = 0; i < new_distances.size(); i++)
      // {
      //   std::cout << new_distances.at(i) << ", ";
      // }
      //
      // std::cout << std::endl << std::endl;


      this->pub_.publish(msg);


      // std::string filename = "/home/darobot/Research/escalibr/catkin_ws/src/escalibr/image_proc/image_final.jpg";
      // cv::imwrite(filename, cur_image);
      // msg.distances = new_distances;
      //
      // cv::imshow("Extractor", cur_image);
      // //
      // cv::waitKey(1);

    }
  }
}

}  // namespace escalibr
