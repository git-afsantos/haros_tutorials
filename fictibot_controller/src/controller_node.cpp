#include <ros/ros.h>

#include "fictibot_controller/random_controller.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fictibot_controller");
    ros::NodeHandle n;

    RandomController controller(n, 10 /*Hz*/);

    ros::Rate loop_rate(10 /*Hz*/);

    while (ros::ok())
    {
        controller.spin();

        loop_rate.sleep();
    }

  return 0;
}
