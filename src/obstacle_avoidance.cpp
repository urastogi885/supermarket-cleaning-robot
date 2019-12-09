/**
 * BSD 3-Clause License
 *
 * @copyright (c) 2019, Umang Rastogi, Naman Gupta
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
 * @file    obstacle_avoidance.cpp
 * @author  Umang Rastogi - Driver
 * @author  Naman Gupta   - Navigator
 * @brief   Source file to implement obstacle avoidance class
 * @detail  Uses laser sensor for obstacle avoidance 
 *          publishes velocities for the robot upon obstacle detection
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "obstacle_avoidance/obstacle_avoidance.h"

ObstacleAvoidance::ObstacleAvoidance() {
  ROS_INFO_STREAM("Setting up obstacle avoidance for the robot...");
  /// Initialize obstacle detected value with false
  obstacleDetected = false;
  /// Initialize safe distance from an obstacle in meters
  distanceThreshold = 0.8;
  /// Subscribe for data from the laser sensor on the scan topic
  subscibeSensor = nh.subscribe<sensor_msgs::LaserScan>("/scan", 500, \
              &ObstacleAvoidance::laserSensorCallback, this);
  ROS_INFO_STREAM("Set up complete");
}

ObstacleAvoidance::ObstacleAvoidance(float distThreshold) {
  ROS_INFO_STREAM("Setting up obstacle avoidance for the robot...");
  /// Initialize obstacle detected value with false
  obstacleDetected = false;
  /// Initialize safe distance from an obstacle in meters
  distanceThreshold = distThreshold;
  /// Subscribe for data from the laser sensor on the scan topic
  subscibeSensor = nh.subscribe<sensor_msgs::LaserScan>("/scan", 500, \
              &ObstacleAvoidance::laserSensorCallback, this);
  ROS_INFO_STREAM("Set up complete");
}

void ObstacleAvoidance::laserSensorCallback(
  const sensor_msgs::LaserScan::ConstPtr& sensorData) {
  /// Read sensor data to get obstacle distances with respect to the robot
  for (const float &range : sensorData->ranges) {
    if (range <= distanceThreshold) {
      ROS_INFO_STREAM("Distance: " << range);
      setObstacleDetected(true);
      return;
    }
  }
  setObstacleDetected(false);
}

bool ObstacleAvoidance::checkObstacle() {
  /// Check if obstacle is ahead
  if (getObstacleDetected()) {
    ROS_WARN_STREAM("Obstacle ahead!");
    return true;
  }
  return false;
}

ObstacleAvoidance::~ObstacleAvoidance() {}
