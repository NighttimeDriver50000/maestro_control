// Copyright 2017 Chris McKinney, Sam Dauchert, and University of South Carolina.
// All rights reserved.

#ifndef MAESTRO_ROS_HPP
#define MAESTRO_ROS_HPP
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include "MaestroControl.hpp"

using namespace std;

class MaestroROS {
public:
    MaestroROS(ros::NodeHandle nh, const char *tty, uint8_t channel_count);
    void spin(double frequency);
    void spinOnce();

private:
    MaestroControl ctl;
    uint8_t channel_count;

    void handleTargetMessage(std_msgs::Float32::const_ptr msg);
}

#endif
