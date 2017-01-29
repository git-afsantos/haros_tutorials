#include <ros/ros.h>

#include "fictibot_drivers/sensor_manager.h"
#include "fictibot_drivers/motor_manager.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fictibot_driver");
    ros::NodeHandle n;

    SensorManager sensor_man(n, 10 /*Hz*/);
    MotorManager motor_man(n, 10 /*Hz*/);

    ros::Rate loop_rate(10 /*Hz*/);

    while (ros::ok())
    {
        sensor_man.spin();
        motor_man.spin();

        loop_rate.sleep();
    }

  return 0;
}
