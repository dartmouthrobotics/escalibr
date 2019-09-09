/**
*
* \author     Monika Roznere <monika.roznere.gr@dartmouth.edu> <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#ifndef ESCALIBR_IMAGE_PROC_H
#define ESCALIBR_IMAGE_PROC_H

#include <string>
#include <opencv2/opencv.hpp>

namespace escalibr
{

/** Image processing handler class.
 *  Handles simple image processing needs. For now that is sphere detection in
 *  the images when there is a successful detection.
 */

class ImageProc
{
public:
  /** Constructor.
   */
  ImageProc() {}
  ~ImageProc() {}

  /** Function for detecting sphere in image.
   *
   *  \param image that has a potential sphere in the scene.
   */
  cv::KeyPoint detectSphere(cv::Mat image);

private:
};

}  // namespace escalibr

#endif  // ESCALIBR_IMAGE_PROC_H
