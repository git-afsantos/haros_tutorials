
#define _USE_MATH_DEFINES

#include <math.h>
#include "fictibot_drivers/motor_manager.h"

// emergency stop lasts for 5 seconds (default)
#define STOP_TIME 5.0
// max. linear acceleration is 0.5 m/s^2
#define LINEAR_ACCEL 0.5
// max. angular acceleration is PI/2 rad/s^2
#define ANGULAR_ACCEL M_PI/2.0
// top linear speed is +-1 m/s
#define MAX_LINEAR 1.0
#define MIN_LINEAR -1.0
// top angular speed is +-PI rad/s
#define MAX_ANGULAR M_PI
#define MIN_ANGULAR -M_PI

MotorManager::MotorManager(ros::NodeHandle& n, double hz)
    : timer_stop_(0.0)
{
    delta_t_ = 1.0 / hz;
    n.param<double>("~stop_time", stop_time_, STOP_TIME);
    stop_subscriber_ = n.subscribe("cmd_stop", 1,
            &MotorManager::stop_callback, this);
    cmd_subscriber_ = n.subscribe("cmd_vel", 10,
            &MotorManager::velocity_callback, this);
    vel_publisher_ = n.advertise<fictibot_msgs::VelocityCommand>("vel", 10);
}


void MotorManager::velocity_callback(const fictibot_msgs::VelocityCommand::ConstPtr& msg)
{
    command_.linear = msg->linear;
    command_.angular = msg->angular;
}


void MotorManager::stop_callback(const std_msgs::Empty::ConstPtr& msg)
{
    timer_stop_ = stop_time_;
}


/**
 * Emergency stops activate the `timer_stop_`.
 * While not elapsed, the robot's velocities should be zero.
 *  `command_` gives us the target (desired) velocities.
 *  `velocity_` gives us the current velocities.
 * If the difference between the two is larger than the acceleration
 *  limits, `LINEAR_ACCEL` and `ANGULAR_ACCEL`, we should accelerate
 *  or decelerate only by the limit value.
 * This does not apply to the emergency stop.
 * Commands that do not obey the maximum/minimum valid limits for
 *  linear and angular velocities should be discarded and produce
 *  no effect on the current velocity.
 */
void MotorManager::spin()
{
    bool stopped = false;
    if (timer_stop_ > 0.0)
    {
        timer_stop_ -= delta_t_;
        if (timer_stop_ > 0.0)
        {
            stopped = true;
            velocity_.linear = 0.0;
            velocity_.angular = 0.0;
        }
    }

    double delta_v, limit;
    if (!stopped)
    {
        delta_v = command_.linear - velocity_.linear;
        limit = LINEAR_ACCEL * delta_t_;
        if (command_.linear >= MIN_LINEAR)
        {
            if (command_.linear <= MAX_LINEAR)
            {
                if (delta_v < 0)
                {
                    if (delta_v < -limit)
                    {
                        delta_v = -limit;
                    }
                }
                else
                {
                    if (delta_v > limit)
                    {
                        delta_v = limit;
                    }
                }
                velocity_.linear += delta_v;
            }
        }

        delta_v = command_.angular - velocity_.angular;
        limit = ANGULAR_ACCEL * delta_t_;
        if (command_.angular >= MIN_ANGULAR)
        {
            if (command_.angular <= MAX_ANGULAR)
            {
                if (delta_v < 0)
                {
                    if (delta_v < -limit)
                    {
                        delta_v = -limit;
                    }
                }
                else
                {
                    if (delta_v > limit)
                    {
                        delta_v = limit;
                    }
                }
                velocity_.angular += delta_v;
            }
        }
    }

    vel_publisher_.publish(velocity_);
}
