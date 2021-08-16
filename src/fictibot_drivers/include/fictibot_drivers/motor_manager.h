#ifndef MOTOR_MANAGER_H_
#define MOTOR_MANAGER_H_

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <fictibot_msgs/VelocityCommand.h>

class MotorManager
{
public:
    MotorManager(ros::NodeHandle& n, double hz);

    ~MotorManager(){};


    void spin();

private:
    double delta_t_;
    double stop_time_;
    double timer_stop_;

    fictibot_msgs::VelocityCommand command_;
    fictibot_msgs::VelocityCommand velocity_;

    ros::Subscriber stop_subscriber_;
    ros::Subscriber cmd_subscriber_;
    ros::Publisher vel_publisher_;


    void stop_callback(const std_msgs::Empty::ConstPtr& msg);

    void velocity_callback(const fictibot_msgs::VelocityCommand::ConstPtr& msg);
};

#endif /*MOTOR_MANAGER_H_*/
