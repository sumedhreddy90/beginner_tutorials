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
#ifndef BEGINNER_TUTORIALS_INCLUDE_BEGINNER_TUTORIALS_TALKER_H_
#define BEGINNER_TUTORIALS_INCLUDE_BEGINNER_TUTORIALS_TALKER_H_

#include "ros/ros.h"
#include "std_msgs/String.h"

//ros::NodeHandle* node;

/**
 * Publisher object to publish data for provided topic name.
 */
ros::Publisher chatter_pub;

/**
 * Message count with unique character count
 */
int count;

/**
 * Message object consisting data to publish message.
 */
std_msgs::String msg;

/**
 * stream object for storing a given sequence of characters.
 */
std::stringstream ss;

/**
 * Service object to advertise data on particular service
 */
//ros::ServiceServer service;

#endif  // BEGINNER_TUTORIALS_INCLUDE_BEGINNER_TUTORIALS_TALKER_H_

