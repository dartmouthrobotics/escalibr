/**
*
* \author     Monika Roznere <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#ifndef ESCALIBR_ELLIPSE_MATH_H
#define ESCALIBR_ELLIPSE_MATH_H

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

namespace escalibr
{

class EllipseMath
{
public:
  EllipseMath() {}
  ~EllipseMath() {}

  void detectEdgePoint(float distance, cv::Mat prev_image);
  int checkEllipse(float range);
  void fitEllipse(int pointer);
  int getEllipseDataSize() {return edge_point_data_.size();}
  std::vector<float> getEchoSounderTf();

private:
  int ELLIPSE_SAMPLE_SIZE_ = 10;
  std::vector<Eigen::Vector3f> edge_point_data_;
  std::vector<Eigen::Vector3f> ellipse_data_;
};

}  // namespace escalibr

#endif  // ESCALIBR_ELLIPSE_MATH_H
