// Copyright 2017 Chris McKinney, Sam Dauchert, and University of South Carolina.
// All rights reserved.

#include <boost/function.hpp>
#include <functional>
#include "MaestroROS.hpp"

MaestroROS::MaestroROS(ros::NodeHandle &nh, const char *tty,
        uint8_t channel_count, const map<uint8_t, string> &channel_names) {
    ctl.open(tty);
    this->channel_count = channel_count;
    for (uint8_t channel = 0; channel < channel_count; ++channel) {
        string topic_name = "target_" + channel_names.at(channel);
        ROS_INFO_STREAM("Subscribing to topic " << topic_name << " (" <<
                (channel + 1) << " of " << +channel_count << ").");
        boost::function<void(std_msgs::Float32)> callback =
                [this,channel](std_msgs::Float32 msg) {
                    handleTargetMessage(channel, msg);
                };
        subs.push_back(nh.subscribe<std_msgs::Float32>(
                topic_name, 1, callback));
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

void MaestroROS::handleTargetMessage(uint8_t channel, std_msgs::Float32 msg) {
    ctl.setTarget(channel, msg.data);
}

int main(int argc, char **argv) {
    bool print_usage = false;
    char *tty;
    uint8_t channel_count;
    double frequency;
    map<uint8_t, string> channel_names;

    ROS_INFO("Entered main.");
    if (argc < 4 || argc % 2 == 1) {
        ROS_FATAL_STREAM("Invalid argc: " << argc);
        print_usage = true;
    } else {
        try {
            tty = argv[1];
            channel_count = boost::numeric_cast<uint8_t>(atoi(argv[2]));
            frequency = boost::lexical_cast<double>(argv[3]);
            for (uint8_t i = 0; i < channel_count; ++i) {
                channel_names[i] = to_string(+i);
            }
            for (int i = 4; i < argc; i += 2) {
                uint8_t channel = boost::lexical_cast<uint8_t>(argv[i]);
                if (channel >= channel_count) {
                    ROS_FATAL_STREAM("Invalid channel: " << channel << " (>= "
                            << channel_count << ")");
                    print_usage = true;
                    break;
                }
                channel_names[channel] = string(argv[i + 1]);
            }
        } catch (std::exception err) {
            ROS_FATAL_STREAM("Error while parsing args: " << err.what());
            print_usage = true;
        }
    }
    if (print_usage) {
        ROS_FATAL_STREAM("Usage: " << argv[0] << " <tty> <# channels> <freq> [<channel> <name>]...");
        return EXIT_FAILURE;
    }
    ROS_INFO("Args parsed. Initializing ROS.");
    ros::init(argc, argv, "maestro_ros");
    ros::NodeHandle nh("maestro_ros");
    ROS_INFO("Opening resources.");
    MaestroROS mros(nh, tty, channel_count, channel_names);
    ROS_INFO("Starting spin.");
    mros.spin(frequency);
    return 0;
}
