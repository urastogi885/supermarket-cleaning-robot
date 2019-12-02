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
 * @file	obstacle_avoidance_test.cpp
 * @author	Naman Gupta 	- Driver
 * @author	Umang Rastogi 	- Navigator
 * @brief	Class test implementation of class ObstacleAvoidance
 * @details	Test the methods of class ObstacleAvoidance
 */
#include <obstacle_avoidance/obstacle_avoidance.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

/**
 * @brief	Test to check obstacle detected or not
 */
TEST(ObstacleAvoidanceTest, obstacleNotDetected) {
    ObstacleAvoidance obstacle;
    EXPECT_FALSE(obstacle.checkObstacle());
}

/**
 * @brief	Test to check getters and setters
 */
TEST(ObstacleAvoidanceTest, obstacleDetected) {
    ObstacleAvoidance obstacle;
    EXPECT_EQ(obstacle.getObstacleDetected(),
        obstacle.setObstacleDetected(true));
}
