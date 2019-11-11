#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include <std_msgs/String.h>
#include "beginner_tutorials/changeBaseOutputString.h"

TEST(MessageTest, checkExistence) {
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
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
