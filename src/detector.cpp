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
 * @file    detector.cpp
 * @author  Umang Rastogi - Navigator
 * @author  Naman Gupta   - Driver
 * @brief   Detector node file to implement object detection algorithm
 * @detail  Implements object detection algorithm using HSV color detection
 */

#include <ros/ros.h>
#include "object_detection/object_detection.h"

/**
 * @brief      main function
 * @param      argc
 * @param      argv
 * @return     int
 */
int main(int argc, char **argv) {
  /// Initialized object detection node
  ros::init(argc, argv, "detector");
  /// Declaring object of class ObjectDetection
  ObjectDetection objDet;

  while (ros::ok()) {
    /// Checks empty image
    if (!objDet.convertedImage.empty()) {
      /// Apply detectObject method to detect cans in the world
        objDet.detectObject(objDet.applyGaussBlur(objDet.convertedImage));
    }
  ros::spinOnce();
  }
  /// Close both the windows of HSVImage and Turtlebot View
  cv::destroyWindow("HSVImage");
  cv::destroyWindow("Turtlebot View");
  return 0;
}
