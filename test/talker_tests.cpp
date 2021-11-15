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
 * @file talker_tests.cpp
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
  ros::ServiceClient listener = node.serviceClient //NOLINT
  <beginner_tutorials::Service>("Service");
  bool run_success(listener.waitForExistence(ros::Duration(1)));
  EXPECT_TRUE(run_success);
}

/**
 * @brief Tests service to update publisher message
 */
TEST(TalkerNodeTest, UpdateString) {
  ros::NodeHandle node;
  ros::ServiceClient client =
      node.serviceClient<beginner_tutorials::Service>("Service");
  beginner_tutorials::Service::Request request;
  beginner_tutorials::Service::Response response;
  request.input_string = "Test string";
  std::string expectedString = request.input_string;
  bool success = client.call(request, response);
  EXPECT_EQ("Test string", response.output_string);
}
