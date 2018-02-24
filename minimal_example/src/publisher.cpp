#include <ros/ros.h>
#include <std_msgs/String.h>
#include "minimal_example/GetCounter.h"

void publishHello(ros::Publisher &pub) {
    std_msgs::String msg;
    msg.data = "hello world";
    pub.publish(msg);
}

void getCounter(ros::ServiceClient &client) {
    minimal_example::GetCounter srv;
    if (client.call(srv)) {
        ROS_INFO("Counter: %d", (int) srv.response.counter);
    } else {
        ROS_ERROR("Failed to call service counter");
    }
}

int main(int argc, char **argv) {
    if (argc < 2) { return 1; }
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::String>("monologue", 10);
    ros::ServiceClient client = n.serviceClient<minimal_example::GetCounter>("counter");
    double frequency = -1;
    if (n.getParam(argv[1], frequency)) {
        ROS_INFO("Got frequency from parameter.");
    } else {
        ROS_ERROR("Could not get frequency from parameter.");
        frequency = 10;
    }
    ros::Rate loop_rate(frequency);
    while (ros::ok()) {
        publishHello(pub);
        getCounter(client);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
