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
 * @file 		   talker.cpp
 * @author 		 Naman Gupta
 * @copyright  GNU
 * @brief 		 ROS publisher publishes a message to topic
 * @version    2.0
 */

#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include "beginner_tutorials/changeBaseOutputString.h"

/**
 * String that will be modified by the user
 */
extern std::string message = "Naman talking";

/**
 * @brief function to change the base string output
 * @param req Data request sent to service
 * @param res Response provided to client
 * @return bool
 */
bool newMessage(beginner_tutorials::changeBaseOutputString::Request &req,
                beginner_tutorials::changeBaseOutputString::Response &res) {
  message = req.originalString;
  ROS_INFO_STREAM("The string is changed to ");
  res.newString = req.originalString;
  return true;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  // Using a variable to store freq
  int freq = 2;

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  // Creating an object br for TransforBroadcaster
  static tf::TransformBroadcaster bc;

  // Creating an object transform for Transform
  tf::Transform trans;

  // Creating an object q for Quaternion
  tf::Quaternion quat;

  ros::Rate loop_rate(freq);
  // Let's generate log messages
  if (argc > 1) {
    // Converting string argument to integer
    freq = atoi(argv[1]);

    if (freq < 0) {
      // For negative frequency or less than zero
      ROS_FATAL_STREAM("Frequency cannot be negative");
      ros::shutdown();
      return 1;
    } else if (freq == 0) {
      // For frequency equals to '0'
      ROS_ERROR_STREAM("Frequency cannot be zero!!");
      ROS_INFO_STREAM("Changing frequency to 5hz");
      freq = 5;
    } else if (freq >= 10) {
      // For frequency greater than 10
      ROS_WARN_STREAM("Too fast to read!!");
      ROS_INFO_STREAM("Changing frequency to 5hz");
      freq = 5;
    }
  }

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise < std_msgs::String
      > ("chatter", 1000);

  // For service to changeBaseOutputString
  ros::ServiceServer server = n.advertiseService("changeBaseOutputString",
                                                 newMessage);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok()) {
    ROS_DEBUG_STREAM_ONCE("Current frequency: " << freq);

    // Non zero translation and rotation
    quat.setRPY(0, 0, 1);
    trans.setRotation(quat);
    trans.setOrigin(tf::Vector3(sin(ros::Time::now().toSec()),
                          cos(ros::Time::now().toSec()), 0.0) );
    // Sending transform using TransformBroadcaster
    bc.sendTransform(tf::StampedTransform(trans,
        ros::Time::now(), "world", "talk"));

    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << message;
    msg.data = ss.str();

    ROS_INFO_STREAM(msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
