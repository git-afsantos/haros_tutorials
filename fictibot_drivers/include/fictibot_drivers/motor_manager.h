#ifndef MOTOR_MANAGER_H_
#define MOTOR_MANAGER_H_

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>

class MotorManager
{
public:
    MotorManager(ros::NodeHandle& n, double hz);

    ~MotorManager(){};


    void spin();

private:
    int teleop_priority_, stop_cycles_;
    double command_, velocity_;

    ros::Subscriber teleop_subscriber_, controller_subscriber_,
                    stop_subscriber_;


    void teleop_callback(const std_msgs::Float64::ConstPtr& msg);

    void controller_callback(const std_msgs::Float64::ConstPtr& msg);

    void stop_callback(const std_msgs::Empty::ConstPtr& msg);

    void apply_commands();
};

#endif /*MOTOR_MANAGER_H_*/
