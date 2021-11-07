// Copyright 2021 Sumedh Reddy Koppula
#ifndef BEGINNER_TUTORIALS_TALKER_H_
#define BEGINNER_TUTORIALS_TALKER_H_

#include "ros/ros.h"
#include "std_msgs/String.h"

ros::NodeHandle* node;

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
ros::ServiceServer service;

#endif  // BEGINNER_TUTORIALS_TALKER_H_

