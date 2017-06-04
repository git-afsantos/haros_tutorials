
#define _USE_MATH_DEFINES

#include <math.h>
#include <cstdlib>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>

#include "fictibot_controller/random_controller.h"

RandomController::RandomController(ros::NodeHandle& n, double hz)
    : stop_(false)
    , laser_proximity_(false)
    , bumper_left_pressed_(false)
    , bumper_center_pressed_(false)
    , bumper_right_pressed_(false)
    , wheel_left_dropped_(false)
    , wheel_right_dropped_(false)
    , stop_counter_(0)
{
    stop_cycles_ = 2 * hz + 1;

    uint32_t queue_size    = (uint32_t) hz * 2 + 1;

    command_publisher_     = n.advertise<std_msgs::Float64>("controller_cmd",
                                                            1);
    stop_publisher_        = n.advertise<std_msgs::Empty>("/stop_cmd",
                                                          0);

    laser_subscriber_      = n.subscribe("laser", queue_size,
            &RandomController::laser_callback, this);
    bumper_subscriber_     = n.subscribe("bumper", queue_size,
            &RandomController::bumper_callback, this);
    wheel_drop_subscriber_ = n.subscribe("wheel", queue_size,
            &RandomController::wheel_callback, this);
}


void RandomController::spin()
{
    ros::spinOnce();

    bool prev_stop = stop_;
    stop_cycles_--;

    if (laser_proximity_)
    {
        stop_ = true;
    }
    else if (bumper_left_pressed_)
    {
        stop_ = true;
    }
    else if (bumper_center_pressed_)
    {
        stop_ = true;
    }
    else if (bumper_right_pressed_)
    {
        stop_ = true;
    }
    else if (wheel_left_dropped_)
    {
        stop_ = true;
    }
    else if (wheel_right_dropped_)
    {
        stop_ = true;
    }
    else
    {
        stop_ = false;
    }

    if (!prev_stop && stop_)
    {
        std_msgs::Empty stop_msg;
        stop_publisher_.publish(stop_msg);

        stop_counter_ = stop_cycles_;
        std_msgs::Float64 vel_msg;
        vel_msg.data = (double) (std::rand() % 360 - 180) * M_PI / 180.0;
        command_publisher_.publish(vel_msg);
    }

    if (stop_ && stop_counter_ < 0)
    {
        stop_counter_ = stop_cycles_;
        std_msgs::Float64 vel_msg;
        vel_msg.data = (double) (std::rand() % 360 - 180) * M_PI / 180.0;
        command_publisher_.publish(vel_msg);
    }
}


void RandomController::laser_callback(const std_msgs::Int8::ConstPtr& msg)
{
    if (msg->data <= 50)
    {
        laser_proximity_ = true;
    }
    else
    {
        laser_proximity_ = false;
    }
}

void RandomController::bumper_callback(const std_msgs::Int8::ConstPtr& msg)
{
    int left    = msg->data & 4;
    int center  = msg->data & 2;
    int right   = msg->data & 1;
    if (left)
    {
        bumper_left_pressed_ = true;
    }
    else
    {
        bumper_left_pressed_ = false;
    }
    if (center)
    {
        bumper_center_pressed_ = true;
    }
    else
    {
        bumper_center_pressed_ = false;
    }
    if (right)
    {
        bumper_right_pressed_ = true;
    }
    else
    {
        bumper_right_pressed_ = false;
    }
}

void RandomController::wheel_callback(const std_msgs::Int8::ConstPtr& msg)
{
    int left = msg->data & 12;
    int right = msg->data & 3;
    if (left == 3)
    {
        wheel_left_dropped_ = true;
    }
    else if (left == 2 && right >= 1)
    {
        wheel_left_dropped_ = true;
    }
    else if (left < 2)
    {
        wheel_left_dropped_ = false;
    }

    if (right == 3)
    {
        wheel_right_dropped_ = true;
    }
    else if (right == 2 && left >= 1)
    {
        wheel_right_dropped_ = true;
    }
    else if (right < 2)
    {
        wheel_right_dropped_ = false;
    }
}
