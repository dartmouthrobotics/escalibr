/**
*
* \author     Monika Roznere <monika.roznere.gr@dartmouth.edu> <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#ifndef ESCALIBR_CALIBRATION_MATH_H
#define ESCALIBR_CALIBRATION_MATH_H

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
// #include <Eigen/Dense>
#include <tinyxml.h>

namespace escalibr
{

/** Calibration math class.
 *  Handles the mathematics for detecting data points
 *  and calculating echo sounder transformation.
 */

class CalibrationMath
{
public:
  /** Constructor.
   */
  CalibrationMath() {}
  ~CalibrationMath() {}

  /** Counting if data points are successfully detected or not.
    * Used for saving images.
    */
  int missed_data_points_ = 0;
  int collected_data_points_ = 0;

  /** Function for calculating 3D points from 2D sphere pixel units.
   *
   *  \param distance is the measurement given by the echo sounder.
   *  \param sphere is the center point of the sphere that was successfully detected.
   *            Could be no sphere, if it was not detected.
   */
  bool detectEdgePoint(float distance, cv::KeyPoint sphere);

  /** Function for saving successfully found 3D edge points and its echo sounder measurement.
   *
   *  \param edge_point is the 3D point of the sphere's center
   *  \param distanc is the measurement given by the echo sounder.
   */
  void saveData(cv::Scalar edge_point, float distance);

  // TO DO: NOT USED CURRENTLY
  // int checkEllipse(float range);
  // void fitEllipse(int pointer);
  // int getEllipseDataSize() {return ellipse_data_.size();}
  // std::vector<float> getEchoSounderTf();

private:
  std::vector<cv::Scalar> edge_point_data_;    /** Store found edge points -- sphere centers */

  /** Parameters for saving collected dat to XML file.
    */
  int file_counter = 0;   /**< Keeps track of number of data points saved in XML file */
  TiXmlDocument out_doc;  /**< XML file */

  // TO DO: NOT USED CURRENTLY
  // int ELLIPSE_SAMPLE_SIZE_ = 10;
  // std::vector<Eigen::Vector3f> ellipse_data_;
};

}  // namespace escalibr

#endif  // ESCALIBR_CALIBRATION_MATH_H
