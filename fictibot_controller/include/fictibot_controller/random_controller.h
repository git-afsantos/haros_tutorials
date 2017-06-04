#ifndef RANDOM_CONTROLLER_H_
#define RANDOM_CONTROLLER_H_

#include <ros/ros.h>
#include <std_msgs/Int8.h>

class RandomController
{
public:
    RandomController(ros::NodeHandle& n, double hz);

    ~RandomController(){};


    void spin();

private:
    bool stop_;
    bool laser_proximity_;
    bool bumper_left_pressed_;
    bool bumper_center_pressed_;
    bool bumper_right_pressed_;
    bool wheel_left_dropped_;
    bool wheel_right_dropped_;
    double stop_cycles_, stop_counter_;

    ros::Publisher stop_publisher_, command_publisher_;

    ros::Subscriber laser_subscriber_, bumper_subscriber_,
                    wheel_drop_subscriber_;


    void laser_callback(const std_msgs::Int8::ConstPtr& msg);

    void bumper_callback(const std_msgs::Int8::ConstPtr& msg);

    void wheel_callback(const std_msgs::Int8::ConstPtr& msg);
};

#endif /*RANDOM_CONTROLLER_H_*/
