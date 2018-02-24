#include <ros/ros.h>
#include <std_msgs/String.h>
#include "minimal_example/GetCounter.h"

static int g_counter = 0;

void chatterCallback(const std_msgs::String::ConstPtr& msg) {
    g_counter++;
}

bool getCounter(minimal_example::GetCounter::Request  &req,
                minimal_example::GetCounter::Response &res) {
    res.counter = g_counter;
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("chatter", 10, chatterCallback);
    ros::ServiceServer service = n.advertiseService("counter", getCounter);
    ros::spin();
    return 0;
}
