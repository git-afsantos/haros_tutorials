
#include <cstdlib>
#include <std_msgs/Int8.h>
#include <fictibot_msgs/BumperEvent.h>
#include <fictibot_msgs/WheelDropEvent.h>
#include "fictibot_drivers/sensor_manager.h"

#define BUMPER_LEFT_BIT_MASK 4
#define BUMPER_CENTER_BIT_MASK 2
#define BUMPER_RIGHT_BIT_MASK 1

#define WHEEL_LEFT_BIT_MASK 2
#define WHEEL_RIGHT_BIT_MASK 1

SensorManager::SensorManager(ros::NodeHandle& n, double hz)
{
    bumper_publisher_     = n.advertise<fictibot_msgs::BumperEvent>("bumper", 10);
    laser_publisher_      = n.advertise<std_msgs::Int8>("laser", 10);
    wheel_drop_publisher_ = n.advertise<fictibot_msgs::WheelDropEvent>("wheel", 10);
}


void SensorManager::spin()
{
    int8_t data = read_bumper();
    fictibot_msgs::BumperEvent bumper_msg;
    bumper_msg.left = (data & BUMPER_LEFT_BIT_MASK) ? true : false;
    bumper_msg.center = (data & BUMPER_CENTER_BIT_MASK) ? true : false;
    bumper_msg.right = (data & BUMPER_RIGHT_BIT_MASK) ? true : false;
    bumper_publisher_.publish(bumper_msg);

    std_msgs::Int8 laser_msg;
    laser_msg.data = read_laser();
    laser_publisher_.publish(laser_msg);

    data = read_wheels();
    fictibot_msgs::WheelDropEvent wheel_msg;
    wheel_msg.left = (data & WHEEL_LEFT_BIT_MASK) ? true : false;
    wheel_msg.left = (data & WHEEL_RIGHT_BIT_MASK) ? true : false;
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
    // left[0,1] | right[0,1]
    return (int8_t) (std::rand() % 4);
}
