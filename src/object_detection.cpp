#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "image_transport/image_transport.h"
#include "object_detection/object_detection.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

ObjectDetection::ObjectDetection() 
  : it(nh) {
  image_sub = it.subscribe("/image_raw", 500,
    &ObjectDetection::convertImage, this);
}

cv::Mat ObjectDetection::readImage(std::string imagePath) {
	// get mat format of image from image reader
  cv::Mat cvImage = cv::imread(imagePath);
  // check validity of image
  if (cvImage.empty()) {
    throw std::runtime_error("Invalid File Path");
  }
  return cvImage;
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

bool ObjectDetection::templateMatching() {
  templ = readImage("../data/template.jpg");
  /// Result is now our source file
  /// It has location of the matching features
  // Match Template
  cv::matchTemplate(convertedImage, templ, result, cv::TemplateMatchModes(CV_TM_CCORR_NORMED));
  if (result.empty()) {
    return false;
  }
  cv::normalize( result, result, 0, 1, 32, -1, cv::Mat());
  return true;
}

cv::Rect ObjectDetection::getObjectLocation() {
  cv::minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());
  matchLoc = maxLoc;
  objectBoundary.x = matchLoc.x;
  objectBoundary.y = matchLoc.y;
  objectBoundary.width = templ.cols;
  objectBoundary.height = templ.rows;
  cv::rectangle(result, objectBoundary, cv::Scalar(255, 0, 0) );
  return objectBoundary;
}

ObjectDetection::~ObjectDetection() {
}
