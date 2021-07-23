
#define _USE_MATH_DEFINES

#include <math.h>
#include <cstdlib>
#include <std_msgs/Empty.h>
#include <fictibot_msgs/VelocityCommand.h>

#include "fictibot_controller/random_controller.h"

RandomController::RandomController(ros::NodeHandle& n, double hz)
    : timer_(0.0)
    , laser_proximity_(false)
    , bumper_left_pressed_(false)
    , bumper_center_pressed_(false)
    , bumper_right_pressed_(false)
    , wheel_left_dropped_(false)
    , wheel_right_dropped_(false)
{
    delta_t_ = 1.0 / hz;
    n.param<double>("~change_time", change_time_, 5.0);

    command_publisher_ = n.advertise<fictibot_msgs::VelocityCommand>("/cmd_vel", 1);

    laser_subscriber_ = n.subscribe("laser", 10,
            &RandomController::laser_callback, this);
    bumper_subscriber_ = n.subscribe("bumper", 10,
            &RandomController::bumper_callback, this);
    wheel_drop_subscriber_ = n.subscribe("wheel", 10,
            &RandomController::wheel_callback, this);
}


void RandomController::spin()
{
    bool obstacle;
    if (laser_proximity_)
    {
        obstacle = true;
    }
    else if (bumper_left_pressed_)
    {
        obstacle = true;
    }
    else if (bumper_center_pressed_)
    {
        obstacle = true;
    }
    else if (bumper_right_pressed_)
    {
        obstacle = true;
    }
    else if (wheel_left_dropped_)
    {
        obstacle = true;
    }
    else if (wheel_right_dropped_)
    {
        obstacle = true;
    }

    if (timer_ > 0.0)
    {
        timer_ -= delta_t_;
    }

    if (obstacle || timer_ <= 0.0)
    {
        fictibot_msgs::VelocityCommand vel_msg;
        vel_msg.linear = (double) (std::rand() % 2000 - 1000) / 1000.0;
        vel_msg.angular = (double) (std::rand() % 360 - 180) * M_PI / 180.0;
        command_publisher_.publish(vel_msg);
        timer_ = change_time_;
    }
}


void RandomController::laser_callback(const std_msgs::Int8::ConstPtr& msg)
{
    // laser data in [0, 127] (cm)
    if (msg->data <= 50)
    {
        laser_proximity_ = true;
    }
    else
    {
        laser_proximity_ = false;
    }
}

void RandomController::bumper_callback(const fictibot_msgs::BumperEvent::ConstPtr& msg)
{
    if (msg->left)
    {
        bumper_left_pressed_ = true;
    }
    else
    {
        bumper_left_pressed_ = false;
    }
    if (msg->center)
    {
        bumper_center_pressed_ = true;
    }
    else
    {
        bumper_center_pressed_ = false;
    }
    if (msg->right)
    {
        bumper_right_pressed_ = true;
    }
    else
    {
        bumper_right_pressed_ = false;
    }
}

void RandomController::wheel_callback(const fictibot_msgs::WheelDropEvent::ConstPtr& msg)
{
    if (msg->left)
    {
        wheel_left_dropped_ = true;
    }
    else
    {
        wheel_left_dropped_ = false;
    }

    if (msg->right)
    {
        wheel_right_dropped_ = true;
    }
    else
    {
        wheel_right_dropped_ = false;
    }
}
