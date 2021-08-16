#include <ros/ros.h>

#include "fictibot_safety_controller/safety_controller.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fictibot_safety_controller");
    ros::NodeHandle n;

    SafetyController controller(n, 10 /*Hz*/);

    ros::Rate loop_rate(10 /*Hz*/);

    while (ros::ok())
    {
        ros::spinOnce();

        controller.spin();

        loop_rate.sleep();
    }

  return 0;
}
