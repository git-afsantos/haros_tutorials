#include <ros/ros.h>
#include <std_msgs/Int8.h>

void aCallback(const std_msgs::Int8::ConstPtr& msg) {}

void pCallback(const std_msgs::Int8::ConstPtr& msg) {}

void qCallback(const std_msgs::Int8::ConstPtr& msg) {}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mini");
    ros::NodeHandle n;
    ros::Subscriber p = n.subscribe("p", 10, pCallback);
    ros::Subscriber a = n.subscribe("a", 10, aCallback);
    ros::Subscriber q = n.subscribe("q", 10, qCallback);
    ros::Publisher b = n.advertise<std_msgs::Int8>("b", 10);
    ros::spin();
    return 0;
}
