
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


/**
 * A stop command does not stop the robot immediately. It gradually reduces
 *  the speed over a number of cycles, until the robot comes to a full stop.
 * If there are no stop cycles to count, the robot should try to move.
 * The `command_` tells the turning direction in radians.
 *  `command_` == 0 means move forward, it should increase velocity, up to max.
 *  `command_` < 0 means turning to the right.
 *  `command_` > 0 means turning to the left.
 *  `command_` beyond PI/8 means a hard turn, that should decrease velocity.
 *      Only when fully stopped should the robot start turning, in this case.
 *  To simulate turning, reduce `command_` by PI/64 per cycle, towards zero.
 */
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
