
#define _USE_MATH_DEFINES

#include <math.h>
#include "fictibot_drivers/motor_manager.h"

MotorManager::MotorManager(ros::NodeHandle& n, double hz)
    : teleop_priority_(0)
    , stop_cycles_(0)
    , command_(0)
    , velocity_(0)
{
    uint32_t queue_size    = (uint32_t) hz * 2 + 1;

    stop_subscriber_       = n.subscribe("stop_cmd", queue_size,
            &MotorManager::stop_callback, this);
    teleop_subscriber_     = n.subscribe("teleop_cmd", queue_size,
            &MotorManager::teleop_callback, this);
    controller_subscriber_ = n.subscribe("controller_cmd", queue_size,
            &MotorManager::controller_callback, this);
}


void MotorManager::spin()
{
    ros::spinOnce();
    apply_commands();
}


void MotorManager::teleop_callback(const std_msgs::Float64::ConstPtr& msg)
{
    command_ = msg->data;
    teleop_priority_ = 5;
}

void MotorManager::controller_callback(const std_msgs::Float64::ConstPtr& msg)
{
    if (teleop_priority_ < 0)
    {
        command_ = msg->data;
    }
}


void MotorManager::stop_callback(const std_msgs::Empty::ConstPtr& msg)
{
    stop_cycles_ = 50;
}


void MotorManager::apply_commands()
{
    teleop_priority_--;
    stop_cycles_--;
    if (stop_cycles_ >= 0)
    {
        velocity_ -= 0.125;
        if (velocity_ < 0)
        {
            velocity_ = 0;
        }
        return;
    }

    if (command_ > M_PI / 8 || command_ < -M_PI / 8)
    {
        velocity_ -= 0.125;
        if (velocity_ < 0)
        {
            velocity_ = 0;
        }

        if (velocity_ == 0)
        {
            if (command_ < 0)
            {
                command_ += M_PI / 64;
                if (command_ > 0)
                {
                    command_ = 0;
                }
            }
            else
            {
                command_ -= M_PI / 64;
                if (command_ < 0)
                {
                    command_ = 0;
                }
            }
        }
    }
    else
    {
        if (command_ < 0)
        {
            command_ += M_PI / 64;
            if (command_ > 0)
            {
                command_ = 0;
            }
        }
        else
        {
            command_ -= M_PI / 64;
            if (command_ < 0)
            {
                command_ = 0;
            }
        }
    }
    if (command_ == 0)
    {
        velocity_ += 0.0625;
        if (velocity_ > 0.5)
        {
            velocity_ = 0.5;
        }
    }
}
