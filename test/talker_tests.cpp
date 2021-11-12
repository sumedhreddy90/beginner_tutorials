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
 * @file talker.h
 * @author Sumedh Koppula
 * @date 7th Nov 2021
 * @copyright All rights reserved
 * @brief header file for publishing messages over master node.
 * 
 */
#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include "beginner_tutorials/Service.h"

/**
 * @brief A test case to check for successful service
 * @return void
 */
TEST(TalkerNodeTest, checkServiceInitialization) {
  ros::NodeHandle node;
  ros::ServiceClient listener = \
     node.serviceClient<beginner_tutorials::Service>("I love Robots!");
  bool run_success(listener.waitForExistence(ros::Duration(10)));
  EXPECT_TRUE(run_success);
}

///  Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "talker");
  ros::NodeHandle node;
  return RUN_ALL_TESTS();
}
