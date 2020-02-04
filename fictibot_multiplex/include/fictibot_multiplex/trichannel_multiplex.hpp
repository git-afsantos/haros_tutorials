#ifndef TRICHANNEL_MULTIPLEX_HPP_
#define TRICHANNEL_MULTIPLEX_HPP_

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>

class TriChannelMultiplexer
{
public:
    TriChannelMultiplexer(ros::NodeHandle& n, double hz);

    ~TriChannelMultiplexer(){};


    void spin();

private:
    int channel_, priority_cycles_, inactivity_counter_;

    ros::Publisher stop_publisher_, command_publisher_, state_publisher_;

    ros::Subscriber high_cmd_subscriber_, high_stop_subscriber_,
                    normal_cmd_subscriber_, normal_stop_subscriber_,
                    low_cmd_subscriber_, low_stop_subscriber_;


    void high_cmd_callback(const std_msgs::Float64::ConstPtr& msg);
    void high_stop_callback(const std_msgs::Empty::ConstPtr& msg);

    void normal_cmd_callback(const std_msgs::Float64::ConstPtr& msg);
    void normal_stop_callback(const std_msgs::Empty::ConstPtr& msg);

    void low_cmd_callback(const std_msgs::Float64::ConstPtr& msg);
    void low_stop_callback(const std_msgs::Empty::ConstPtr& msg);
};

#endif /*TRICHANNEL_MULTIPLEX_HPP_*/
