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
 * @file turtlebot.cpp
 * @author Umang Rastogi - Driver
 * @author Naman Gupta - Navigator
 * @brief Source file to implement turtlebot class
 * @detail Controls the motion of the bot using obstacle avoidance and go-to-goal strategies
 */

/// Add ROS headers
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
/// Add custom libraries to help control motion of the robot
#include "turtlebot/turtlebot.h"
#include "object_detection/object_detection.h"
#include "obstacle_avoidance/obstacle_avoidance.h"

Turtlebot::Turtlebot() {
  ROS_INFO_STREAM("Initiliazing the robot...");
  ObstacleAvoidance obstacleAvoidance;
  // ObjectDetection objectDetection;
  /// Initialize the current value of velocities in m/s and rad/s
  linearVelocity = 1.0;
  anguarVelocity = 0.52;
  /// Define the main access point to communications with the ROS system
  ros::NodeHandle nh;
  /// Publish the velocities to the robot on the navigation topic
  publishVelocities = nh.advertise<geometry_msgs::Twist>\
               ("/cmd_vel_mux/input/navi", 1000);
  ROS_INFO_STREAM("Set up complete");
}

Turtlebot::Turtlebot(float linVelX, float angVelZ) {
  ROS_INFO_STREAM("Initiliazing the robot...");
  ObstacleAvoidance obstacleAvoidance;
  // ObjectDetection objectDetection;
  /// Initialize the current value of velocities in m/s and rad/s
  linearVelocity = linVelX;
  anguarVelocity = angVelZ;
  /// Define the main access point to communications with the ROS system
  ros::NodeHandle nh;
  /// Publish the velocities to the robot on the navigation topic
  publishVelocities = nh.advertise<geometry_msgs::Twist>\
               ("/cmd_vel_mux/input/navi", 1000);
  ROS_INFO_STREAM("Set up complete");
}

Turtlebot::~Turtlebot() {
  if (resetBot()) {
     ROS_INFO_STREAM("The bot velocities have been reset");
  }
}

float Turtlebot::moveForward(float linVelX) {
  velocities.linear.x = linVelX;
  velocities.angular.z = 0.0;

  return velocities.linear.x;
}

float Turtlebot::turn(float angVelZ) {
  velocities.linear.x = 0.0;
  velocities.angular.z = angVelZ;

  return velocities.angular.z;
}

bool Turtlebot::collectObject() {
  return false;
}

void Turtlebot::moveBot() {
  return;
}

bool Turtlebot::resetBot() {
  return true;
}

bool Turtlebot::checkVelocityChanged() {
  return false;
}
