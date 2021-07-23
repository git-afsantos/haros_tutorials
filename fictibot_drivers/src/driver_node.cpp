#include <ros/ros.h>

#include "fictibot_drivers/sensor_manager.h"
#include "fictibot_drivers/motor_manager.h"

#define FREQ 10 /*Hz*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fictibot_driver");
    ros::NodeHandle n;

    SensorManager sensor_man(n, FREQ);
    MotorManager motor_man(n, FREQ);

    ros::Rate loop_rate(FREQ);

    while (ros::ok())
    {
        ros::spinOnce();

        sensor_man.spin();
        motor_man.spin();

        loop_rate.sleep();
    }

  return 0;
}
