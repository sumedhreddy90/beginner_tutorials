// Copyright 2021 Sumedh Reddy Koppula
#ifndef BEGINNER_TUTORIALS_INCLUDE_BEGINNER_TUTORIALS_LISTENER_H_
#define BEGINNER_TUTORIALS_INCLUDE_BEGINNER_TUTORIALS_LISTENER_H_

#include "ros/ros.h"

ros::NodeHandle* node;

/**
 * This is a subscriber object, to receive the message published under a given
 * topic.
 */
ros::Subscriber subscriber;

#endif  // BEGINNER_TUTORIALS_INCLUDE_BEGINNER_TUTORIALS_LISTENER_H_

