// Copyright 2017 Chris McKinney, Sam Dauchert, and University of South Carolina.
// All rights reserved.

#include <string>
#include "MaestroROS.hpp"

MaestroROS::MaestroROS(ros::NodeHandle nh, const char *tty,
        uint8_t channel_count) {
    ctl.open(tty);
    this.channel_count = channel_count;

    for (uint8_t channel = 0; channel < channel_count; ++channel) {
        nh.subscribe("target_" + string(channel), 1,
                &MaestroROS::handleTargetMessage, this);
    }
}

void MaestroROS::spin(double frequency) {
    ros::Rate rate(frequency);
    while (ros::ok()) {
        spinOnce();
        rate.sleep();
    }
}

void MaestroROS::spinOnce() {
    ros::spinOnce();
}

void handleTargetMessage(std_msgs::Float32::const_ptr msg) {
    ROS_INFO(string(msg.data));
}

int main(int argc, char **argv) {
    char *tty;

    if (argc <= 3) {
        try {
            tty = argv[1];
            channel_count = boost::lexical_cast<uint8_t>(argv[2]);
            frequency = boost::lexical_cast<double>(argv[3]);
        } catch (std::exception err) {
            printUsage = true;
        }
    } else {
        // TODO
    }
}
