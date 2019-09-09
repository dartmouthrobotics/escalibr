/**
*
* \author     Monika Roznere <monika.roznere.gr@dartmouth.edu> <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#include "escalibr/calibration_gui.h"

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>


namespace escalibr
{

// Initialize static members
bool CalibrationGui::play = false;
bool CalibrationGui::debug = false;


void CalibrationGui::mouse_callback(int event, int x, int y, int, void*)
{
  if (event == cv::EVENT_LBUTTONDOWN)
  {
    // Check if user pressed play/pause button
    float cx = 1920 + 150;
    float cy = 580;
    float radius = 100;
    float distance = sqrt(static_cast<double>((cx - x)*(cx - x) + (cy - y)*(cy - y)));
    if (distance <= radius)
    {
      CalibrationGui::play = !CalibrationGui::play;
      return;
    }

    // Check if user pressed calibrate button
    cx = 1920 + 400;
    distance = sqrt(static_cast<double>((cx - x)*(cx - x) + (cy - y)*(cy - y)));
    if (distance <= radius)
    {
      std::cout << "TO DO: CALIBRATION TIME" << std::endl;
      return;
    }

    // Check if user pressed debug button
    cx = 1920 + 150;
    cy = 810;
    distance = sqrt(static_cast<double>((cx - x)*(cx - x) + (cy - y)*(cy - y)));
    if (distance <= radius)
    {
      CalibrationGui::debug = !CalibrationGui::debug;
      return;
    }

    // Check if user pressed save button
    cx = 1920 + 400;
    distance = sqrt(static_cast<double>((cx - x)*(cx - x) + (cy - y)*(cy - y)));
    if (distance <= radius)
    {
      std::cout << "TO DO: SAVING TIME" << std::endl;
      return;
    }
  }
}


void CalibrationGui::run(cv::Mat image, cv::KeyPoint sphere, double distance, int confidence,
  std::string message, int d_ind)
{
  // Format such that image will be on the left and info on the right
  cv::Mat display = cv::Mat(image.rows, image.cols + 900, image.type());
  display.setTo(255);
  image.copyTo(display(cv::Rect(cv::Point(0, 0), image.size())));

  if (sphere.size != 0)
  {
    cv::circle(display, sphere.pt, sphere.size/2, cv::Scalar(0, 0, 255), 3);
  }

  // Information: distance, confidence, and what to do (hold / detect / move out)
  cv::putText(display, "distance: " + std::to_string(distance).substr(0, 5),
    cv::Point2f(image.cols + 45, 100), cv::FONT_HERSHEY_DUPLEX, 2.5, cv::Scalar(0, 0, 0), 3);
  cv::putText(display, "confidence: " + std::to_string(confidence), cv::Point2f(image.cols + 45, 250),
    cv::FONT_HERSHEY_DUPLEX, 2.5, cv::Scalar(0, 0, 0), 3);
  cv::putText(display, message, cv::Point2f(image.cols + 45, 400), cv::FONT_HERSHEY_DUPLEX, 2.5,
    cv::Scalar(0, 0, 0), 3);

  // Play/pause buton
  cv::circle(display, cv::Point(image.cols + 150, 580), 100, cv::Scalar(0, 0, 0), 3);
  if (!CalibrationGui::play)
  {
    // Play symbol
    std::vector<cv::Point> play_symbol;
    play_symbol.push_back(cv::Point(image.cols + 115, 530));
    play_symbol.push_back(cv::Point(image.cols + 115, 630));
    play_symbol.push_back(cv::Point(image.cols + 205, 580));
    cv::fillConvexPoly(display, play_symbol, cv::Scalar(0, 0, 0));
  }
  else
  {
    // Pause symbol
    cv::rectangle(display, cv::Point(image.cols + 110, 530), cv::Point(image.cols + 140, 630),
      cv::Scalar(0, 0, 0), CV_FILLED);
    cv::rectangle(display, cv::Point(image.cols + 160, 530), cv::Point(image.cols + 190, 630),
      cv::Scalar(0, 0, 0), CV_FILLED);
  }

  // Calibrate button
  cv::circle(display, cv::Point(image.cols + 400, 580), 100, cv::Scalar(0, 0, 0), 3);
  cv::putText(display, "C", cv::Point2f(image.cols + 355, 620), cv::FONT_HERSHEY_DUPLEX, 4, cv::Scalar(0, 0, 0), 6);

  // Debug button
  if (CalibrationGui::debug)
  {
    cv::circle(display, cv::Point(image.cols + 150, 810), 100, cv::Scalar(220, 220, 220), CV_FILLED);
  }
  cv::circle(display, cv::Point(image.cols + 150, 810), 100, cv::Scalar(0, 0, 0), 3);
  cv::putText(display, "D", cv::Point2f(image.cols + 110, 850), cv::FONT_HERSHEY_DUPLEX, 4, cv::Scalar(0, 0, 0), 6);

  // Save button
  cv::circle(display, cv::Point(image.cols + 400, 810), 100, cv::Scalar(0, 0, 0), 3);
  cv::putText(display, "S", cv::Point2f(image.cols + 360, 850), cv::FONT_HERSHEY_DUPLEX, 4, cv::Scalar(0, 0, 0), 6);

  // Draw line to visualize number of collected data points
  int intervals = this->visual_point_data_.size();
  for (int i = 0; i < intervals; i++)
  {
    float top_left_x = image.cols + 600;
    float top_left_y = 880 - (400 / intervals) * (i + 1);
    float bottom_right_x = image.cols + 600 + (10 * this->visual_point_data_.at(i).size());
    float bottom_right_y = 880 - (400 / intervals) * i;

    cv::Scalar color;
    if (this->visual_point_data_.at(i).size() < 8)
    {
      color = cv::Scalar(0, 0, 255);
    }
    else if (this->visual_point_data_.at(i).size() < 14)
    {
      color = cv::Scalar(51, 255, 255);
    }
    else
    {
      color = cv::Scalar(0, 255, 0);
    }

    if (d_ind == i)
    {
      color = cv::Scalar(255, 0, 0);
    }

    cv::rectangle(display, cv::Point(top_left_x, top_left_y), cv::Point(bottom_right_x, bottom_right_y),
      color, CV_FILLED);
  }
  cv::line(display, cv::Point(image.cols + 600, 880 - (400 / intervals) * this->visual_point_data_.size()),
    cv::Point(image.cols + 600, 880), cv::Scalar(0, 0, 0), 3);
  cv::putText(display, "dist", cv::Point2f(image.cols + 570, 920), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 0), 2);
  cv::putText(display, std::to_string(this->MIN_DEPTH_).substr(0, 3), cv::Point2f(image.cols + 540, 880 - 5),
    cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 0), 2);
  cv::putText(display, std::to_string(this->MAX_DEPTH_).substr(0, 3),
    cv::Point2f(image.cols + 540, 880 + 20 - (400 / intervals) * this->visual_point_data_.size()),
    cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 0), 2);

  // Open window
  cv::namedWindow("ESCalibr", cv::WINDOW_NORMAL);
  // This will handle when user presses one of the three buttons
  cv::setMouseCallback("ESCalibr", mouse_callback, 0);

  cv::imshow("ESCalibr", display);

  cv::waitKey(1);
}

}  // namespace escalibr
