
#include <cstdlib>
#include <std_msgs/Int8.h>
#include "fictibot_drivers/sensor_manager.h"

SensorManager::SensorManager(ros::NodeHandle& n, double hz)
{
    uint32_t queue_size   = (uint32_t) hz * 2 + 1;

    bumper_publisher_     = n.advertise<std_msgs::Int8>("bumper", queue_size);
    laser_publisher_      = n.advertise<std_msgs::Int8>("laser", queue_size);
    wheel_drop_publisher_ = n.advertise<std_msgs::Int8>("wheel", queue_size);
}


void SensorManager::spin()
{
    std_msgs::Int8 bumper_msg;
    bumper_msg.data = read_bumper();
    bumper_publisher_.publish(bumper_msg);

    std_msgs::Int8 laser_msg;
    laser_msg.data = read_laser();
    laser_publisher_.publish(laser_msg);

    std_msgs::Int8 wheel_msg;
    wheel_msg.data = read_wheels();
    wheel_drop_publisher_.publish(wheel_msg);
}


int8_t SensorManager::read_bumper()
{
    // left[0,1] | center[0,1] | right[0,1]
    return (int8_t) (std::rand() % 8);
}

int8_t SensorManager::read_laser()
{
    return (int8_t) (std::rand() % 128);
}

int8_t SensorManager::read_wheels()
{
    // left[0,3] | right[0,3]
    return (int8_t) (std::rand() % 16);
}
