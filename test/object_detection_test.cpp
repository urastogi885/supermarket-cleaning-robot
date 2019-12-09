/**
 * BSD 3-Clause License
 *
 * @copyright (c) 2019, Naman Gupta, Umang Rastogi
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
 * @file	obstacle_detection_test.cpp
 * @author	Naman Gupta 	- Navigator
 * @author	Umang Rastogi 	- Driver
 * @brief	Class test implementation of class ObstacleAvoidance
 * @details	Test the methods of class ObstacleAvoidance
 */
#include <object_detection/object_detection.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include "opencv2/opencv.hpp"

/**
 * @brief	Test to check getters and setters
 */
TEST(ObjectDetectionTest, objectNotDetected) {
	ObjectDetection objectDetection;
	objectDetection.setObjectDetected(false);
    EXPECT_FALSE(objectDetection.getObjectDetected());
}

/**
 * @brief	Test to check getters and setters
 */
TEST(ObjectDetectionTest, objectDetected) {
	ObjectDetection objectDetection;
    objectDetection.setObjectDetected(true);
    EXPECT_TRUE(objectDetection.getObjectDetected());
}

/**
 * @brief	Test to check setting of object boundary
 */
TEST(ObjectDetectionTest, setBoundary) {
	ObjectDetection objectDetection;
	/// Define object boundary
	cv::Rect boundingBox = {0, 21, 113, 56};
    objectDetection.setObjectBoundary(boundingBox);
    EXPECT_EQ(objectDetection.getObjectBoundary(), boundingBox);
}

/**
 * @brief	Test to check gaussian filtering
 */
TEST(ObjectDetectionTest, gaussFilter) {
	ObjectDetection objectDetection;
	bool gaussCheck = true;
	if (!objectDetection.convertedImage.empty()) {
    	cv::Mat gaussImg = objectDetection.applyGaussBlur(
    		objectDetection.convertedImage);
    	if(gaussImg.size() == objectDetection.convertedImage.size()) {
    		// These two images do not hace the same size due to smoothening
    		gaussCheck = false;
    	}
	}
    EXPECT_TRUE(gaussCheck);
}

/**
 * @brief	Test to check object detection using hsv
 */
TEST(ObjectDetectionTest, checkObject) {
	ObjectDetection objectDetection;
	bool objectDetected;
	if (!objectDetection.convertedImage.empty()) {
    	objectDetected = objectDetection.detectObject(
    		objectDetection.applyGaussBlur(objectDetection.convertedImage));
	}
    EXPECT_FALSE(objectDetected);
}
