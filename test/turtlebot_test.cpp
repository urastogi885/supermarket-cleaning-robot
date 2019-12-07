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
 * @file    turtlebot_test.h
 * @author  Umang Rastogi 	- Navigator
 * @author  Naman Gupta 	- Driver
 * @brief   Class test implemention for class Turtlebot
 * @detail  Tests the method of class Turtlebot
 */

#include <turtlebot/turtlebot.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

/**
 * @brief	Test to check obstacle detected or not
 */
TEST(TurtlebotTest, velocityChangedTest) {
    Turtlebot turtle;
    EXPECT_TRUE(turtle.checkVelocityChanged());
}

/**
 * @brief	Test for moveForward() method
 * @detail	Test to check linear velocity provided to method
 */
TEST(TurtlebotTest, moveForwardTest) {
	/// Define angular and linear velocities for the robot
	float linVel = 2.0;
	float angVel = 0.52;
    Turtlebot turtle(linVel, angVel);
    EXPECT_EQ(linVel, turtle.moveForward(linVel));
}

/**
 * @brief	Test for turn() method
 * @detail	Test to check angular velocity provided to method
 */
TEST(TurtlebotTest, turnTest) {
	/// Define angular and linear velocities for the robot
	float linVel = 2.0;
	float angVel = 0.52;
    Turtlebot turtle(linVel, angVel);
    EXPECT_EQ(angVel, turtle.turn(angVel));
}

/**
 * @brief	Test for resetting the bot
 */
TEST(TurtlebotTest, resetTest) {
    Turtlebot turtle;
    EXPECT_TRUE(turtle.resetBot());
}