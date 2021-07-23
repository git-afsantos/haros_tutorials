
#include <cstdlib>
#include <std_msgs/Int8.h>

#include "fictibot_multiplex/trichannel_multiplex.hpp"

#define LOW_PRIORITY -1
#define NORMAL_PRIORITY 0
#define HIGH_PRIORITY 1

TriChannelMultiplexer::TriChannelMultiplexer(ros::NodeHandle& n, double hz)
    : channel_(LOW_PRIORITY)
    , priority_cycles_(10)
    , inactivity_counter_(0)
{
    command_publisher_ = n.advertise<fictibot_msgs::VelocityCommand>("cmd_vel", 10);
    stop_publisher_ = n.advertise<std_msgs::Empty>("cmd_stop", 10);
    state_publisher_ = n.advertise<std_msgs::Int8>("state", 10);

    high_cmd_subscriber_ = n.subscribe("high_cmd_vel", 10,
            &TriChannelMultiplexer::high_cmd_callback, this);
    high_stop_subscriber_ = n.subscribe("high_cmd_stop", 10,
            &TriChannelMultiplexer::high_stop_callback, this);
    normal_cmd_subscriber_ = n.subscribe("normal_cmd_vel", 10,
            &TriChannelMultiplexer::normal_cmd_callback, this);
    normal_stop_subscriber_ = n.subscribe("normal_cmd_stop", 10,
            &TriChannelMultiplexer::normal_stop_callback, this);
    low_cmd_subscriber_ = n.subscribe("low_cmd_vel", 10,
            &TriChannelMultiplexer::low_cmd_callback, this);
    low_stop_subscriber_ = n.subscribe("low_cmd_stop", 10,
            &TriChannelMultiplexer::low_stop_callback, this);

    std_msgs::Int8 state_msg;
    state_msg.data = (int8_t) LOW_PRIORITY;
    state_publisher_.publish(state_msg);
}


void TriChannelMultiplexer::spin()
{
    inactivity_counter_--;

    if (inactivity_counter_ < 0)
    {
        channel_ = LOW_PRIORITY;
        inactivity_counter_ = priority_cycles_;
        std_msgs::Int8 state_msg;
        state_msg.data = (int8_t) LOW_PRIORITY;
        state_publisher_.publish(state_msg);
    }
}


void TriChannelMultiplexer::high_cmd_callback(const fictibot_msgs::VelocityCommand::ConstPtr& msg)
{
    command_publisher_.publish(msg);
    channel_ = HIGH_PRIORITY;
    inactivity_counter_ = priority_cycles_;
    std_msgs::Int8 state_msg;
    state_msg.data = (int8_t) HIGH_PRIORITY;
    state_publisher_.publish(state_msg);
}

void TriChannelMultiplexer::high_stop_callback(const std_msgs::Empty::ConstPtr& msg)
{
    stop_publisher_.publish(msg);
    channel_ = HIGH_PRIORITY;
    inactivity_counter_ = priority_cycles_;
    std_msgs::Int8 state_msg;
    state_msg.data = (int8_t) HIGH_PRIORITY;
    state_publisher_.publish(state_msg);
}

void TriChannelMultiplexer::normal_cmd_callback(const fictibot_msgs::VelocityCommand::ConstPtr& msg)
{
    if (channel_ != HIGH_PRIORITY)
    {
        command_publisher_.publish(msg);
        channel_ = NORMAL_PRIORITY;
        inactivity_counter_ = priority_cycles_;
        std_msgs::Int8 state_msg;
        state_msg.data = (int8_t) NORMAL_PRIORITY;
        state_publisher_.publish(state_msg);
    }
}

void TriChannelMultiplexer::normal_stop_callback(const std_msgs::Empty::ConstPtr& msg)
{
    if (channel_ != HIGH_PRIORITY)
    {
        stop_publisher_.publish(msg);
        channel_ = NORMAL_PRIORITY;
        inactivity_counter_ = priority_cycles_;
        std_msgs::Int8 state_msg;
        state_msg.data = (int8_t) NORMAL_PRIORITY;
        state_publisher_.publish(state_msg);
    }
}

void TriChannelMultiplexer::low_cmd_callback(const fictibot_msgs::VelocityCommand::ConstPtr& msg)
{
    if (channel_ == LOW_PRIORITY)
    {
        command_publisher_.publish(msg);
        inactivity_counter_ = priority_cycles_;
    }
}

void TriChannelMultiplexer::low_stop_callback(const std_msgs::Empty::ConstPtr& msg)
{
    if (channel_ == LOW_PRIORITY)
    {
        stop_publisher_.publish(msg);
        inactivity_counter_ = priority_cycles_;
    }
}
