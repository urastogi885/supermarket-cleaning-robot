/**
 * BSD 3-Clause License
 *
 * @copyright (c) 2019, Umang Rastogi, Naman Gupta
 * 
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
 * @file 	  object_detection.h
 * @author 	Umang Rastogi 	- Driver
 * @author 	Naman Gupta 	- Navigator
 * @brief 	Library header file to implement object detection
 * @detail 	Deploys template matching to detect object in the bot's world
 */

#ifndef INCLUDE_OBJECT_DETECTION_OBJECT_DETECTION_H_
#define INCLUDE_OBJECT_DETECTION_OBJECT_DETECTION_H_

#include <vector>
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

class ObjectDetection {
 private:
  /// Define the main access point to communications with the ROS system
  ros::NodeHandle nh;
  /// Define a subscriber object to data of the laser sensor
  ros::Subscriber subscribeImages;
  /// Store copy of the ros image converted into cv image
  cv::Mat hsvImage, maskImage;
  /// Define object coordinates
  cv::Rect objectBoundary;
  /// Define upper and lower limit of color
  /// Set range as can visible in HSV image
  const cv::Scalar colorLowerLimit = {170, 50, 50};
  const cv::Scalar colorUpperLimit = {255, 200, 90};
  /// Size of an image
  cv::Size imageSize;
  /// Define image array for contours
  std::vector<std::vector<cv::Point> > imageArray;
  bool objectDetected;

 public:
/**
 * @brief  Constructor for object detection class
 * @param  none
 * @return none
 */
  ObjectDetection();

/**
 * @brief  Destructor for object detection class
 * @param  none
 * @return none
 */
  ~ObjectDetection();

  /// Container to store converted image from cv_bridge
  cv::Mat convertedImage;

  /**
   * @brief  Callback function for image data
   * @param  Image data from camera node
   * @return none
   * @detail Convert ROS image message to CV image
   */
  void convertImage(const sensor_msgs::Image::ConstPtr& imageData);

  /**
   * @brief   Method to detect object using hsv
   * @param   Image gaussian filtered image of type cv::Mat
   * @return  Match found of type bool
   */
  bool detectObject(cv::Mat image);

  /**
   * @brief   Method to convert gaussian filter on the image
   * @param   Converted opencv image of type cv::Mat
   * @return  Image blurred using gaussian filter of type cv::Mat
   */
  cv::Mat applyGaussBlur(cv::Mat cvtImage);

  /**
   * @brief   Get boundary of the object in the image
   * @param   none
   * @return  rectangular box containing the object
   */
  cv::Rect getObjectBoundary() const {
    return objectBoundary;
  }

  /**
   * @brief   Set object boundary
   * @param   BoundingBox reactagular boundary of the object
   * @return  none
   */
  void setObjectBoundary(cv::Rect boundingBox) {
    objectBoundary = boundingBox;
  }

  /**
   * @brief   Get object detected
   * @param   none
   * @return  Object detected or not of type bool
   */
  bool getObjectDetected() const {
    return objectDetected;
  }

  /**
   * @brief   Set object detected
   * @param   Object detected status
   * @return  none
   */
  void setObjectDetected(bool object) {
    objectDetected = object;
  }
};

#endif  // INCLUDE_OBJECT_DETECTION_OBJECT_DETECTION_H_
