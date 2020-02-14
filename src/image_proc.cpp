/**
*
* \author     Monika Roznere <monika.roznere.gr@dartmouth.edu> <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#include "escalibr/image_proc.h"

#include <algorithm>
#include <vector>
#include <opencv2/opencv.hpp>
#include <string>
#include <math.h>
#include <limits>

namespace escalibr
{

cv::KeyPoint ImageProc::detectSphere(cv::Mat image)
{
  // Detect sphere structure in the grayscale image
  std::vector<cv::KeyPoint> keypoints;

  // Setup SimpleBlobDetector parameters.
  cv::SimpleBlobDetector::Params params;

  // Change thresholds for converting image to binary
  params.thresholdStep = 1;  // default: 10
  params.minThreshold = 35;  // default: 50
  params.maxThreshold = 90;  // default: 220
  params.minRepeatability = 1;  // default: 2
  params.minDistBetweenBlobs = 10000;  // default: 10

  // Filters by instensity at the center of a blob
  // blobColor = 0 for dark blobs
  // blobColor = 255 for light blobs
  params.filterByColor = true;  // default: true
  params.blobColor = 0;  // default: 0

  // Filter by blobs within minArea (inclusive) and maxArea (exclusive)
  params.filterByArea = true;  // default: true
  params.minArea = 1000;  // default: 25
  params.maxArea = 5000000;  // default: 5000

  // Filter by circularity between minCircularity (inclusive) and maxCircularity (exclusive)
  params.filterByCircularity = false;  // default: false
  params.minCircularity = 0.2f;  // default: 0.8f
  params.maxCircularity = std::numeric_limits<float>::max();  // default: std::numeric_limits<float>::max()

  // Filter by inertia between minInertiaRatio (inclusive) and maxInertiaRatio (exclusive)
  params.filterByInertia = true;  // default: true
  params.minInertiaRatio = 0.7f;  // default: 0.1f
  params.maxInertiaRatio = std::numeric_limits<float>::max();  // default: std::numeric_limits<float>::max()

  // Filter by ccnvexity (area / area of blob convex hull) between minConvexity (inclusive) and maxConvexity (exclusive)
  params.filterByConvexity = true;  // default: true
  params.minConvexity = 0.95f;  // default: 0.95f
  params.maxConvexity = std::numeric_limits<float>::max();

  cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
  detector->detect(image, keypoints);

  if (keypoints.size() == 1)
  {
    return keypoints[0];
  }

  return cv::KeyPoint();
}

}  // namespace escalibr
