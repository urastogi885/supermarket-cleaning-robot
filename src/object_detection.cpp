/**
 * BSD 3-Clause License
 *
 * @copyright (c) 2019, Umang Rastogi Naman Gupta
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * @file object_detection.h
 * @author Umang Rastogi - Driver
 * @author Naman Gupta - Navigator
 * @brief Library header file to implement object detection
 * @detail Implements object detection using HSV
 */

#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "object_detection/object_detection.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

ObjectDetection::ObjectDetection() {
  subscibeImages = nh.subscribe("/camera/rgb/image_raw", 1,
    &ObjectDetection::convertImage, this);
}

ObjectDetection::~ObjectDetection() {
}

void ObjectDetection::convertImage(const sensor_msgs::Image::ConstPtr& imageData) {
  try {
    convertedImage = cv_bridge::toCvCopy(imageData, "bgr8")->image;
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
    return;
  }
}

bool ObjectDetection::detectObject() {
  /// Convert image from bgr to hsv
  cv::cvtColor(convertedImage, hsvImage, CV_BGR2HSV);
  /// Detect hsv within the set limits
  cv::inRange(hsvImage, colorLowerLimit, colorUpperLimit, maskImage);
  /// Get image size to modify size of mask image
  imageSize = convertedImage.size();
  maskImage(cv::Rect(0, 0, imageSize.width, 0.8*imageSize.height)) = 0;
  /// Find contours for better visualization
  cv::findContours(maskImage, imageArray, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
  /// Check if contours exist in the image
  if (imageArray.size() != 0) {
    auto contourSize = 0;
    auto maxAreaContour = 0;
    auto count = 0;
    while (count < imageArray.size()) {
      /// Find contour with maximum area
      if (contourSize < imageArray[count].size()) {
         maxAreaContour = count;
         contourSize = imageArray[count].size();
      }
      count++;
    }
    /// Set boundary of the object in the image
    setObjectBoundary(cv::boundingRect(imageArray[maxAreaContour]));
    /// Draw the rectangle using the bounding box
    cv::rectangle(convertedImage, getObjectBoundary(), cv::Scalar(255, 0, 0), 2);
  }
  /// Mask image to limit the future turns affecting the output
  maskImage(cv::Rect(0.7*imageSize.width, 0, \
                0.3*imageSize.width, imageSize.height)) = 0;
  maskImage(cv::Rect(0, 0, 0.3*imageSize.width, imageSize.height)) = 0;

  if (cv::countNonZero(maskImage) == 0) {
    setObjectDetected(true);
  } else {
     setObjectDetected(false);
  }

  return getObjectDetected();
}

cv::Mat ObjectDetection::applyGaussBlur() {
  cv::Mat output;
  /// Apply gaussian filter
  cv::GaussianBlur(convertedImage, output, cv::Size(3, 3), 0.1, 0.1);
  return output;
}