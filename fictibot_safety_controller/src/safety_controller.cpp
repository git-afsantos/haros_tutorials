#include <std_msgs/Empty.h>
#include <fictibot_msgs/VelocityCommand.h>

#include "fictibot_safety_controller/safety_controller.h"

#define REACTION_TIME 1.0

SafetyController::SafetyController(ros::NodeHandle& n, double hz)
    : safe_(true)
    , timer_(0.0)
{
    delta_t_ = 1.0 / hz;
    n.param<double>("reaction_time", reaction_time_, REACTION_TIME);

    stop_publisher_ = n.advertise<std_msgs::Empty>("cmd_stop", 1);
    command_publisher_ = n.advertise<fictibot_msgs::VelocityCommand>("cmd_vel", 1);

    laser_subscriber_ = n.subscribe("laser", 10,
            &SafetyController::laser_callback, this);
    bumper_subscriber_ = n.subscribe("bumper", 10,
            &SafetyController::bumper_callback, this);
    wheel_drop_subscriber_ = n.subscribe("wheel", 10,
            &SafetyController::wheel_callback, this);
}


void SafetyController::spin()
{
    if (!safe_)
    {
        timer_ -= delta_t_;
        if (timer_ <= 0.0)
        {
            safe_ = true;
        }
    }
}


void SafetyController::laser_callback(const std_msgs::Int8::ConstPtr& msg)
{
    if (!safe_) { return; }

    // laser data in [0, 127] (cm)
    if (msg->data <= 32)
    {
        emergency_stop();
    }
    else
    {
        if (msg->data <= 64)
        {
            slow_down();
        }
    }
}

void SafetyController::bumper_callback(const fictibot_msgs::BumperEvent::ConstPtr& msg)
{
    if (!safe_) { return; }

    if (msg->left || msg->center || msg->right)
    {
        emergency_stop();
    }
}

void SafetyController::wheel_callback(const fictibot_msgs::WheelDropEvent::ConstPtr& msg)
{
    if (!safe_) { return; }

    if (msg->left || msg->right)
    {
        emergency_stop();
    }
}

void SafetyController::emergency_stop()
{
    safe_ = false;
    timer_ = reaction_time_;
    std_msgs::Empty stop_msg;
    stop_publisher_.publish(stop_msg);
}

void SafetyController::slow_down()
{
    safe_ = false;
    timer_ = reaction_time_;
    fictibot_msgs::VelocityCommand vel_msg;
    vel_msg.linear = 0.0;
    vel_msg.angular = 0.0;
    command_publisher_.publish(vel_msg);
}
