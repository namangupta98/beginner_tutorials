/**
 * Copyright (c) 2019, Naman Gupta
 *
 * Redistribution and use in source and binary forms, with or without  
 * modification, are permitted provided that the following conditions are 
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright 
 * notice, this list of conditions and the following disclaimer in the   
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its 
 * contributors may be used to endorse or promote products derived from this 
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS 
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file 		unitTest.cpp
 * @author 		Naman Gupta
 * @copyright 	GNU
 * @brief 		ROS publisher publishes a message to topic
 * @version     2.0
 */
#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include <std_msgs/String.h>
#include "beginner_tutorials/changeBaseOutputString.h"

/**
 * @brief The function tests the existence of service
 * @param TestSuite, testCase
 * @return void
 */
TEST(MessageTest, checkExistence) {
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  // Service client object initialized
  auto client = n.serviceClient<beginner_tutorials::changeBaseOutputString>
  ("changeBaseService");

  // Checking existence of service
  bool exist(client.waitForExistence(ros::Duration(10)));
  EXPECT_FALSE(exist);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "tester");
	ros::NodeHandle n;
	return RUN_ALL_TESTS();
}
