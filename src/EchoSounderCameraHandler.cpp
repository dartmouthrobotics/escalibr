/**
*
* \author     Monika Roznere <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#include "escalibr/EchoSounderCameraHandler.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>


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
  this->range = 0.1;
  this->prev_structure_detected = false;

  this->img_sub_.subscribe(nh_, camera_topic, 1);
  this->echo_sounder_sub_.subscribe(nh_, echo_sounder_topic, 1);

  this->sync.reset(new Sync(SyncPolicy(1), this->img_sub_, this->echo_sounder_sub_));

  if(0 == calibration_method)
  {
    std::cout << "SUPER COARSE CALIBRATION" << std::endl;
    this->sync->registerCallback(boost::bind(&EchoSounderCameraHandler::super_coarse_callback, this, _1, _2));
    std::vector<float> result = get_echo_sounder_tf();
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
  if (this->max < echo_sounder_msg->distance && this->prev_structure_detected)
  {
    // TO DO!!!!
    // grab edge point

    // well imagine that we got the pixel x and y coordinate for the center of the structure.
    int x = 20;
    int y = 50;

    std::cout << "NEW EDGE POINT" << std::endl;
    std::cout << "depth: " << echo_sounder_msg->distance << " x: " << x << " y: " << y << std::endl << std::endl;

    cv::Point3f edge_point(x, y, echo_sounder_msg->distance);
    this->edge_point_data.push_back(edge_point);

    // check for ellipse
    int pointer = check_ellipse();
    if (-1 != pointer)
    {
      std::cout << "NEW ELLIPSE" << std::endl << std::endl;

      create_ellipse(pointer);

      // two ellipses are enough to get params for echo sounder
      if (2 == this->ellipse_data.size())
      {
        std::vector<float> result = get_echo_sounder_tf();

        save_result(result);

        exit(EXIT_SUCCESS);
      }
      else  // need one more ellipse
      {
        // reset
        this->prev_structure_detected = false;
      }
    }
  }
  else if (this->min < echo_sounder_msg->distance < this->max)
  {
    if(!this->prev_structure_detected)
    {
      this->prev_structure_detected = true;
    }

    this->prev_image = cv_ptr->image;
  }
}


int EchoSounderCameraHandler::check_ellipse()
{
  if (5 <= this->edge_point_data.size())
  {
    return -1;
  }

  // sort vector by depth measurement
  std::sort(this->edge_point_data.begin(), this->edge_point_data.end(), appEllipse);

  // check if 5 points exists within an acceptable depth range
  for(int i = 0; i + 4 < this->edge_point_data.size(); i++)
  {
    float diff = this->edge_point_data[i+4].z - this->edge_point_data[i].z;
    if (diff < this->range)
    {
      // return pointer to where the 5 data points start
      return i;
    }
  }

  return -1;
}


void EchoSounderCameraHandler::create_ellipse(int pt)
{
  // solve Ax = 0
  Eigen::MatrixXf mat(5,6);
  mat << (float) pow(this->edge_point_data[pt].x, 2),   (float) pow(this->edge_point_data[pt].y, 2),   this->edge_point_data[pt].x * this->edge_point_data[pt].y,     this->edge_point_data[pt].x,   this->edge_point_data[pt].y,   1,
         (float) pow(this->edge_point_data[pt+1].x, 2), (float) pow(this->edge_point_data[pt+1].y, 2), this->edge_point_data[pt+1].x * this->edge_point_data[pt+1].y, this->edge_point_data[pt+1].x, this->edge_point_data[pt+1].y, 1,
         (float) pow(this->edge_point_data[pt+2].x, 2), (float) pow(this->edge_point_data[pt+2].y, 2), this->edge_point_data[pt+2].x * this->edge_point_data[pt+2].y, this->edge_point_data[pt+2].x, this->edge_point_data[pt+2].y, 1,
         (float) pow(this->edge_point_data[pt+3].x, 2), (float) pow(this->edge_point_data[pt+3].y, 2), this->edge_point_data[pt+3].x * this->edge_point_data[pt+3].y, this->edge_point_data[pt+3].x, this->edge_point_data[pt+3].y, 1,
         (float) pow(this->edge_point_data[pt+4].x, 2), (float) pow(this->edge_point_data[pt+4].y, 2), this->edge_point_data[pt+4].x * this->edge_point_data[pt+4].y, this->edge_point_data[pt+4].x, this->edge_point_data[pt+4].y, 1;

  Eigen::MatrixXf A = mat.adjoint() * mat;
  Eigen::VectorXf x = Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf>(A).eigenvectors().col(0);

  // calculate average depth of the ellipse
  float avg_depth = 0;
  for (int i = pt; i < pt + 5; i++)
  {
    avg_depth += this->edge_point_data[i].z;
  }
  avg_depth /= 5;

  // remove these data points
  this->edge_point_data.erase(this->edge_point_data.begin()+pt, this->edge_point_data.begin()+pt+5);

  std::vector<float> new_ellipse{x(0), x(1), x(2), x(3), x(4), avg_depth};
  this->ellipse_data.push_back(new_ellipse);
}


std::vector<float> EchoSounderCameraHandler::get_echo_sounder_tf()
{
  // get x and y centers of each ellipse

  Eigen::Matrix2f rot_mat_1;
  // rot_mat_1 << this->ellipse_data[0][0],     this->ellipse_data[0][2] / 2,
  //              this->ellipse_data[0][2] / 2, this->ellipse_data[0][1];
  // Eigen::Matrix2f rot_mat_2;
  // rot_mat_1 << this->ellipse_data[1][0],     this->ellipse_data[1][2] / 2,
  //              this->ellipse_data[1][2] / 2, this->ellipse_data[1][1];

  rot_mat_1 << 4.36, -2.52,
               -2.52, 2.89;

  std::cout << rot_mat_1 << std::endl;

  // get eigenvectors
  Eigen::EigenSolver<Eigen::Matrix2f> es_1;
  es_1.compute(rot_mat_1, true);
  std::cout << es_1.eigenvectors().transpose() << std::endl;
  // Eigen::EigenSolver<Eigen::Matrix2f> es_2;
  // es_2.compute(rot_mat_2);

  Eigen::Vector2f trans_vec_1;
  trans_vec_1 << 30.8, -0.6;

  std::cout << trans_vec_1.transpose() * es_1.eigenvectors().transpose() << std::endl;


  // create a line that goes through these two centers

  // get x, y, z, pitch and yaw of the echo sounder

  std::vector<float> final_parameters = {1, 2, 3, 4, 5};

  return final_parameters;
}


void EchoSounderCameraHandler::save_result(std::vector<float> result)
{
  // TO DO!!!

  std::cout << "x: " << result[0] << std::endl;
  std::cout << "y: " << result[1] << std::endl;
  std::cout << "x: " << result[2] << std::endl;
  std::cout << "pitch: " << result[3] << std::endl;
  std::cout << "yaw: " << result[4] << std::endl;
}

}  // namespace escalibr
