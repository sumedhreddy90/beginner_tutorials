/*
 *Copyright (C) MIT.
 *Permission is hereby granted, free of charge, to any person obtaining a copy
 *of this software and associated documentation files (the "Software"), to deal
 *in the Software without restriction, including without limitation the rights
 *to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *copies of the Software, and to permit persons to whom the Software is
 *furnished to do so, subject to the following conditions:
 *The above copyright notice and this permission notice shall be included in
 *all copies or substantial portions of the Software.
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *SOFTWARE.
 *
 * @file talker.cpp
 * @author Sumedh Koppula
 * @date 7th Nov 2021
 * @copyright All rights reserved
 * @brief Publishiing messages over master node.
 */
#include "beginner_tutorials/talker.h"
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
// Including header for service
#include "beginner_tutorials/Service.h"


std::string pub_msg = "I bake robots";    // NOLINT
/*
 * @brief Updates publisher string data
 * @param request_ : reference to request object from service
 * @param response_ : reference to response object from service
 * @return bool flag indicating success or failure of function execution
 */
bool IsMessage(beginner_tutorials::Service::Request &request_,   // NOLINT
beginner_tutorials::Service::Response &response_) {   // NOLINT
  ROS_INFO_STREAM("updating message");
  if (request_.input_string.empty()) {
    ROS_ERROR_STREAM("Captured empty string message.");
    return false;
  } else {
    ROS_DEBUG_STREAM("Received message: " << request_.input_string);
    ROS_WARN_STREAM("This will change Publisher message");
    pub_msg = request_.input_string;
    response_.output_string = request_.input_string;
    ROS_DEBUG_STREAM("message changed.");
    return true;
  }
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

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle node;
// Creating service and advertised over ROS
  ros::ServiceServer service = node.advertiseService(
  "Service", &IsMessage);
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
  chatter_pub = node.advertise<std_msgs::String>("chatter", 1000);
  double frequency;
  node.getParam("/frequency", frequency);
  ROS_DEBUG_STREAM("Frequency argument = " << frequency);
  if (isnan(frequency) || frequency < 1) {
    ROS_FATAL_STREAM(
      "Invalid frequency. Frequency must be a nonzero positive number");
    ROS_DEBUG_STREAM("Invalid frequency detected. Changed to default value");
    frequency = 10.0;
  } else if (frequency > 51) {
    ROS_WARN_STREAM("Recommended frequency range is 1-50");
  } else if (frequency > 100) {
    ROS_ERROR_STREAM("Error! Frequency value too large.");
    ROS_DEBUG_STREAM("Large frequency detected. Changed to max allowed value");
    frequency = 30;
  }
  ros::Rate loop_rate(frequency);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  count = 0;
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    ss  << count << ": " << pub_msg << std::endl;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

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
