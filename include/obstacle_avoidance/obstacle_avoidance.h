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
 *
 * @file obstacle_avoidance.h
 * @author Umang Rastogi - Driver
 * @author Naman Gupta - Navigator
 * @brief Library header file to implement obstacle avoidance
 */

#ifndef INCLUDE_OBSTACLE_AVOIDANCE_H_
#define INCLUDE_OBSTACLE_AVOIDANCE_H_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class ObstacleAvoidance {
private:
  /// Define the main access point to communications with the ROS system
  ros::NodeHandle nh;
  /// Define a subscriber object to data of the laser sensor
  ros::Subscriber subscibeSensor;
  /// Define variable to store if obstacle was detected
  bool obstacleDetected;
  /// Initalize minimum safe distance from an obstacle
  float distanceThreshold;

public:
  /**
  * @brief Constructor for obstacle avoidance class
  * @param none
  * @return a constructor has no return
  */
  ObstacleAvoidance();

  /**
  * @brief Constructor for obstacle avoidance class
  * @param safe distance from an obstacle
  * @return a constructor has no return
  */
  ObstacleAvoidance(float distThreshold);

  /**
  * @brief Destructor for obstacle avoidance class
  * @param none
  * @return a destrcutor has no return
  */
  ~ObstacleAvoidance();

  /**
  * @brief Callback function for subscriber
  * @param messsage data from LaserScan node
  * @return void
  */
  void laserSensorCallback(const sensor_msgs::LaserScan::ConstPtr& \
  												sensorData);

  /**
  * @brief Checks if obstacle is present within safe distance
  * @param none
  * @return boolean obstacle found or not
  */
  bool checkObstacle();

  /**
  * @brief get obstacle detected
  * @param none
  * @return boolean obstacle detected or not
  */
  bool getObstacleDetected() const {
    return obstacleDetected;
  }

  /**
  * @brief set obstacle detected
  * @param obstacle detected status
  * @return void
  */
  void setObstacleDetected(bool obstacle) {
    obstacleDetected = obstacle;
  }
];

#endif	//	INCLUDE_OBSTACLE_AVOIDANCE_H_