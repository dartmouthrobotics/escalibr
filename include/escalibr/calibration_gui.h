/**
*
* \author     Monika Roznere <monika.roznere.gr@dartmouth.edu> <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#ifndef ESCALIBR_CALIBRATION_GUI_H
#define ESCALIBR_CALIBRATION_GUI_H

#include <string>
#include <opencv2/opencv.hpp>
#include <vector>

namespace escalibr
{

/** Calibration GUI handler class.
 *  Handles information from the echo sounder, images, and current detection
 *  events and displays them in a window. Also handles changes in mode for the
 *  calibration system.
 */

class CalibrationGui
{
public:
  /** Constructor.
   */
  CalibrationGui() {}
  ~CalibrationGui() {}

  /** Parameters used for the GUI buttons.
   */
  static bool play;       /**< true: collect data points; false: do not collect data points */
  static bool debug;      /**< true: draw red circle around sphere when detected; false: no drawing */
  static bool calibrate;  /**< true: start calibrating by using the collected data */
  static bool save;       /**< true: save calibration results */

  /** Parameters used for the GUI data chart.
    */
  float MIN_DEPTH_;     /**< Minimum value shown for detection. */
  float MAX_DEPTH_;     /**< Maximum value shown for detection. */
  /**< Stores each data point's sphere center point in their corresponding depth interval */
  std::vector<std::vector<cv::Point>> visual_point_data_;

  /** Function for drawing out the display image, setting up button callbacks, and displaying GUI.
   *
   *  \param image is the image seen by the camera.
   *  \param sphere is the center point of the sphere that was successfully detected.
   *            Could be no sphere, if it was not detected.
   *  \param distanc is the measurement given by the echo sounder.
   *  \param confidence is the confidence level given by the echo sounder.
   *  \param message is the string format of what is currently happening: no detection, hold, or finished hold.
   *  \param d_ind is the index in visual_point_data for the current depth reading
   */
  void run(cv::Mat image, cv::KeyPoint sphere, double distance, int confidence, std::string message, int d_ind);

private:
  /** Function for handling GUI mouse clicks.
   *
   *  \param event is the event that occured from the mouse.
   *  \int x is the x pixel unit of where the mouse is.
   *  \int y is the y pixel unit of where the mouse is.
   */
  static void mouse_callback(int event, int x, int y, int, void*);
};

}  // namespace escalibr

#endif  // ESCALIBR_CALIBRATION_GUI_H
