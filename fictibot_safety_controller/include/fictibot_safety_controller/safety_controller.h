#ifndef SAFETY_CONTROLLER_H_
#define SAFETY_CONTROLLER_H_

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <fictibot_msgs/BumperEvent.h>
#include <fictibot_msgs/WheelDropEvent.h>

class SafetyController
{
public:
    SafetyController(ros::NodeHandle& n, double hz);

    ~SafetyController(){};


    void spin();

private:
    double delta_t_;
    double timer_;
    double reaction_time_;

    bool safe_;

    ros::Publisher stop_publisher_;
    ros::Publisher command_publisher_;

    ros::Subscriber laser_subscriber_;
    ros::Subscriber bumper_subscriber_;
    ros::Subscriber wheel_drop_subscriber_;


    void laser_callback(const std_msgs::Int8::ConstPtr& msg);

    void bumper_callback(const fictibot_msgs::BumperEvent::ConstPtr& msg);

    void wheel_callback(const fictibot_msgs::WheelDropEvent::ConstPtr& msg);

    void emergency_stop();

    void slow_down();
};

#endif /*SAFETY_CONTROLLER_H_*/
