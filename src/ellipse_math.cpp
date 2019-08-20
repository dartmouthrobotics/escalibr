/**
*
* \author     Monika Roznere <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#include "escalibr/ellipse_math.h"

#include <algorithm>
#include <vector>

namespace escalibr
{

struct AppEllipse
{
  bool operator() (Eigen::Vector3f pt1, Eigen::Vector3f pt2) {return (pt1[2] < pt2[2]);}
}
appEllipse;


void EllipseMath::detectEdgePoint(float distance, cv::Mat prev_image)
{
  // Structure is a sphere
  cv::Mat gray_image;
  cv::cvtColor(prev_image, gray_image, CV_BGR2GRAY);

  // CHECK if this needed
  // Reduce noise to avoid false circle detection
  cv::GaussianBlur(gray_image, gray_image, cv::Size(9, 9), 2, 2);

  // Detect circles in the grayscale image
  std::vector<cv::Vec3f> circles;
  cv::HoughCircles(gray_image, circles, CV_HOUGH_GRADIENT, 1, gray_image.rows/8, 200, 100, 0, 0);

  if (1 == circles.size())  // Sphere structure detected in image
  {
    std::cout << std::endl << "NEW EDGE POINT" << std::endl;
    std::cout << "depth: " << distance << "\tx: " << circles[0][0] << "\ty: " << circles[0][1] << std::endl;

    Eigen::Vector3f new_edge_point(circles[0][1], circles[0][1], distance);
    this->edge_point_data_.push_back(new_edge_point);
  }
  else  // No structure detected in image
  {
    std::cout << std::endl << "ERROR: too many 'circles' in the image" << std::endl;
  }
}


int EllipseMath::checkEllipse(float range)
{
  // Check if there are enough data points
  if (this->ELLIPSE_SAMPLE_SIZE_ > this->edge_point_data_.size())
  {
    return -1;
  }

  // Sort vector by increasing depth measurement
  std::sort(this->edge_point_data_.begin(), this->edge_point_data_.end(), appEllipse);

  // Check if the data points exists within an acceptable depth range
  for (size_t i = 0; i + 0 < this->edge_point_data_.size(); i++)
  {
    float diff = this->edge_point_data_[i + this->ELLIPSE_SAMPLE_SIZE_ - 1][2] - this->edge_point_data_[i][2];
    if (diff < range)
    {
      // Return pointer to where the set of data points start
      return i;
    }
  }

  return -1;
}


// TO DO: more comments explaining algorithm
void EllipseMath::fitEllipse(int pt)
{
  float TOLERANCE = 0.2;
  int DIM = 2;

  Eigen::MatrixXf data_points(DIM, this->ELLIPSE_SAMPLE_SIZE_);
  for (size_t i = pt; i < pt + this->ELLIPSE_SAMPLE_SIZE_; i++)
  {
    data_points.col(i - this->ELLIPSE_SAMPLE_SIZE_) = this->edge_point_data_[i].head(2);
  }

  // Remove this if previous block works
  // data_points << this->edge_point_data_[pt].x, this->edge_point_data_[pt+1].x,
  //                     this->edge_point_data_[pt+2].x, this->edge_point_data_[pt+3].x,
  //                     this->edge_point_data_[pt+4].x, this->edge_point_data_[pt+5].x,
  //                     this->edge_point_data_[pt+6].x, this->edge_point_data_[pt+7].x,
  //                     this->edge_point_data_[pt+8].x, this->edge_point_data_[pt+9].x,
  //                this->edge_point_data_[pt].y, this->edge_point_data_[pt+1].y,
  //                     this->edge_point_data_[pt+2].y, this->edge_point_data_[pt+3].y,
  //                     this->edge_point_data_[pt+4].y, this->edge_point_data_[pt+5].y,
  //                     this->edge_point_data_[pt+6].y, this->edge_point_data_[pt+7].y,
  //                     this->edge_point_data_[pt+8].y, this->edge_point_data_[pt+9].y;

  // Test purposes
  // data_points << 5.0, 7.0, 10.0, 0.0,  5.0,  9.0,  1.0, 9.5,  4.0,  4.0,
  //        4.5, 5.0, 10.0, 10.0, 14.0, 14.0, 4.0, 13.5, 14.0, 4.0;

  Eigen::MatrixXf q = data_points;
  q.conservativeResize(data_points.rows() + 1, data_points.cols());

  for (size_t i = 0; i < q.cols(); i++)
  {
    q(q.rows() - 1, i) = 1;
  }

  int count = 1;
  double err = 1;

  const double init_u = 1.0 / static_cast<double>(this->ELLIPSE_SAMPLE_SIZE_);
  Eigen::MatrixXf u = Eigen::MatrixXf::Constant(this->ELLIPSE_SAMPLE_SIZE_, 1, init_u);

  while (err > TOLERANCE)
  {
    // Eigen::MatrixXf Q_tr = q.transpose();
    Eigen::MatrixXf X = q * u.asDiagonal() * q.transpose();
    Eigen::MatrixXf M = (q.transpose() * X.inverse() * q).diagonal();

    int j_x, j_y;
    double maximum = M.maxCoeff(&j_x, &j_y);
    double step_size = (maximum - DIM - 1) / ((DIM + 1) * (maximum + 1));

    Eigen::MatrixXf new_u = (1 - step_size) * u;
    new_u(j_x, 0) += step_size;

    // Find error
    Eigen::MatrixXf u_diff = new_u - u;
    for (size_t i = 0; i < u_diff.rows(); i++)
    {
      for (size_t j = 0; j < u_diff.cols(); j++)
      {
        u_diff(i, j) *= u_diff(i, j);  // Square each element of the matrix
      }
      err = sqrt(u_diff.sum());
      count++;
      u = new_u;
    }

    float avg_depth = 0.0;
    for (size_t i = pt; i < pt + this->ELLIPSE_SAMPLE_SIZE_; i++)
    {
      avg_depth += this->edge_point_data_[i][2];
    }
    avg_depth /= this->ELLIPSE_SAMPLE_SIZE_;

    Eigen::MatrixXf center = data_points * u;
    std::cout << "center: " << center << std::endl;

    Eigen::Vector3f new_ellipse_data {center(0, 0), center(1, 0), avg_depth};
    this->ellipse_data_.push_back(new_ellipse_data);
  }
}


std::vector<float> EllipseMath::getEchoSounderTf()
{
  // Subtract data points and normalize
  Eigen::Vector3f unit;
  if (this->ellipse_data_[0][2] < this->ellipse_data_[1][2])
  {
    unit = (this->ellipse_data_[1] -this->ellipse_data_[0]).normalized();
  }
  else
  {
    unit = (this->ellipse_data_[0] - this->ellipse_data_[1]).normalized();
  }

  // X, Y, Z
  Eigen::Vector3f pose = this->ellipse_data_[0] - 2 * unit;

  // Yaw
  float yaw = atan2(pose[0], -pose[1]);

  // Pitch
  float pitch = atan2(sqrt(pow(pose[0], 2) + pow(pose[1], 2)), pose[2]);

  std::vector<float> final_parameters = {pose[0], pose[1], pose[2], pitch, yaw};
  return final_parameters;
}

}  // namespace escalibr
