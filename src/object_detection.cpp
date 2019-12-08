#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "image_transport/image_transport.h"
#include "object_detection/object_detection.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

ObjectDetection::ObjectDetection() {
  it(nh);
  cv::namedWindow(OPENCV_WINDOW);
}

cv::Mat ObjectDetection::readImage() {
	templ = cv::imread("..data/template.png");
	image_sub = it.subscribe("/camera/rgb/image_raw", 1,
    &ObjectDetection::convertImage, this);
  image_pub = it.advertise("/image_converter/output_video", 1);
	return image_sub;
}

void ObjectDetection::convertImage(const sensor_msgs::Image::ConstPtr& imageData) {
	cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(imageData, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
}

void ObjectDetection::templateMatching() {
    int matchMethod;
    image = ObjectDetection.readImage();
    cv::matchTemplate(cv_ptr->image, templ, cv_ptr->result, matchMethod); /// result is now our source file which has new image which location of the matching features
    cv::normalize( cv_ptr->result, result, 0, 1, NORM_MINMAX, -1, Mat());
    double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc; cv::Point matchLoc;
  	cv::minMaxLoc( cv_ptr->result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());
  	matchLoc = maxLoc;
  	cv::rectangle( cv_ptr->result, matchLoc, cv::Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
    cv::imshow(OPENCV_WINDOW, cv_ptr->result);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
}

ObjectDetection::~ObjectDetection() {
  cv::destroyWindow(OPENCV_WINDOW);
}
