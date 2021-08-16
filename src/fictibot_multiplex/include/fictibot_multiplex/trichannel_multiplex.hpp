#ifndef TRICHANNEL_MULTIPLEX_HPP_
#define TRICHANNEL_MULTIPLEX_HPP_

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <fictibot_msgs/VelocityCommand.h>

class TriChannelMultiplexer
{
public:
    TriChannelMultiplexer(ros::NodeHandle& n, double hz);

    ~TriChannelMultiplexer(){};


    void spin();

private:
    int channel_;
    int priority_cycles_;
    int inactivity_counter_;

    ros::Publisher stop_publisher_;
    ros::Publisher command_publisher_;
    ros::Publisher state_publisher_;

    ros::Subscriber high_cmd_subscriber_;
    ros::Subscriber high_stop_subscriber_;
    ros::Subscriber normal_cmd_subscriber_;
    ros::Subscriber normal_stop_subscriber_;
    ros::Subscriber low_cmd_subscriber_;
    ros::Subscriber low_stop_subscriber_;


    void high_cmd_callback(const fictibot_msgs::VelocityCommand::ConstPtr& msg);
    void high_stop_callback(const std_msgs::Empty::ConstPtr& msg);

    void normal_cmd_callback(const fictibot_msgs::VelocityCommand::ConstPtr& msg);
    void normal_stop_callback(const std_msgs::Empty::ConstPtr& msg);

    void low_cmd_callback(const fictibot_msgs::VelocityCommand::ConstPtr& msg);
    void low_stop_callback(const std_msgs::Empty::ConstPtr& msg);
};

#endif /*TRICHANNEL_MULTIPLEX_HPP_*/
