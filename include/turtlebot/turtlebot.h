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
 * @file turtlebot.h
 * @author Umang Rastogi - Driver
 * @author Naman Gupta - Navigator
 * @brief Library header file to control motion of the bot
 * @detail Takes input form both, obstacle avoidance and object detection
 */

#ifndef INCLUDE_TURTLEBOT_H_
#define INCLUDE_TURTLEBOT_H_

/// Add ROS headers
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
/// Add custom libraries to help control motion of the robot
#include "../obstacle_avoidance/obstacle_avoidance.h"
#include "../object_detection/object_detection.h"

class Turtlebot {
private:

public:
	/**
  * @brief Constructor for obstacle avoidance class
  * @param none
  * @return a constructor has no return
  */
  Turtlebot();

  /**
  * @brief Destructor for obstacle avoidance class
  * @param none
  * @return a destrcutor has no return
  */
  ~Turtlebot();

  /**
  * @brief Make the bot move forward
  * @param none
  * @return void
  */
  void moveForward();

  /**
  * @brief Turn the bot
  * @param none
  * @return void
  */
  void turn();

  /**
  * @brief Collect the object
  * @param none
  * @return void
  */
  void collectObject();

  /**
  * @brief Control the motion of the bot
  * @param none
  * @return void
  * @detail Use obstacle avoidance and go-to-goal strategies
  *					to move towards the object to be collected
  */
  void moveBot();

  /**
  * @brief Reset the velocities of the bot
  * @param none
  * @return void
  */
  void resetBot();

  /**
  * @brief Check change in the velocites of the bot
  * @param none
  * @return boolean velocity changed or not
  */
  bool checkVelocityChanged();
};

#endif	//	INCLUDE_TURTLEBOT_H_