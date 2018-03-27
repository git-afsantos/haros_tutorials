#include <ros/ros.h>

#include "fictibot_multiplex/trichannel_multiplex.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fictibot_multiplex");
    ros::NodeHandle n;

    TriChannelMultiplexer multiplexer(n, 10 /*Hz*/);

    ros::Rate loop_rate(10 /*Hz*/);

    while (ros::ok())
    {
        multiplexer.spin();

        loop_rate.sleep();
    }

  return 0;
}
