#ifndef SENSOR_MANAGER_H_
#define SENSOR_MANAGER_H_

#include <ros/ros.h>

class SensorManager
{
public:
    SensorManager(ros::NodeHandle n, double hz);

    ~SensorManager(){};


    void spin();

private:
    ros::Publisher bumper_publisher_, laser_publisher_,
                   wheel_drop_publisher_;

    int8_t read_bumper();

    int8_t read_laser();

    int8_t read_wheels();
};

#endif /*SENSOR_MANAGER_H_*/
