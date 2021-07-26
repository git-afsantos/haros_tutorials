
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

#define TIME_TO_OBSTACLE 2.0
#define TIME_TO_RECOVER 0.5

SensorManager::SensorManager(ros::NodeHandle& n, double hz)
    : internal_timer_(TIME_TO_OBSTACLE)
    , facing_obstacle_(false)
{
    delta_t_ = 1.0 / hz;
    bumper_publisher_     = n.advertise<fictibot_msgs::BumperEvent>("bumper", 10);
    laser_publisher_      = n.advertise<std_msgs::Int8>("laser", 10);
    wheel_drop_publisher_ = n.advertise<fictibot_msgs::WheelDropEvent>("wheel", 10);
}


void SensorManager::spin()
{
    internal_timer_ -= delta_t_;
    if (facing_obstacle_)
    {
        if (internal_timer_ <= 0.0)
        {
            facing_obstacle_ = false;
            internal_timer_ = TIME_TO_OBSTACLE;
        }
    }
    else
    {
        if (internal_timer_ <= 0.0)
        {
            facing_obstacle_ = true;
            internal_timer_ = TIME_TO_RECOVER;
        }
    }

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
    if (facing_obstacle_)
    {
        return (int8_t) (std::rand() % 7 + 1); // non-zero, [1, 7]
    }
    else
    {
        return (int8_t) 0;
    }
}

int8_t SensorManager::read_laser()
{
    if (facing_obstacle_)
    {
        return (int8_t) (std::rand() % 100); // [0, 99]
    }
    else
    {
        return (int8_t) (std::rand() % 28 + 100); // [100, 127]
    }
}

int8_t SensorManager::read_wheels()
{
    // left[0,1] | right[0,1]
    if (facing_obstacle_)
    {
        return (int8_t) (std::rand() % 3 + 1); // non-zero, [1, 4]
    }
    else
    {
        return (int8_t) 0;
    }
}
