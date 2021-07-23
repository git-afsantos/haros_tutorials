#ifndef RANDOM_CONTROLLER_H_
#define RANDOM_CONTROLLER_H_

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <fictibot_msgs/BumperEvent.h>
#include <fictibot_msgs/WheelDropEvent.h>

class RandomController
{
public:
    RandomController(ros::NodeHandle& n, double hz);

    ~RandomController(){};


    void spin();

private:
    double delta_t_;
    double timer_;
    double change_time_;

    bool laser_proximity_;
    bool bumper_left_pressed_;
    bool bumper_center_pressed_;
    bool bumper_right_pressed_;
    bool wheel_left_dropped_;
    bool wheel_right_dropped_;

    ros::Publisher command_publisher_;

    ros::Subscriber laser_subscriber_;
    ros::Subscriber bumper_subscriber_;
    ros::Subscriber wheel_drop_subscriber_;


    void laser_callback(const std_msgs::Int8::ConstPtr& msg);

    void bumper_callback(const fictibot_msgs::BumperEvent::ConstPtr& msg);

    void wheel_callback(const fictibot_msgs::WheelDropEvent::ConstPtr& msg);
};

#endif /*RANDOM_CONTROLLER_H_*/
