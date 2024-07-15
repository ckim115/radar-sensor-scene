#pragma once
// Standard Library
#include <string>
// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <robot_state_publisher/robot_state_publisher.h>

class Sim_Node
{
public:
    Sim_Node();
    
    void timerCallback(const ros::TimerEvent &event);
    void update_position(const geometry_msgs::Twist& msg);
private:
    // ROS
    ros::NodeHandle n;
    // Timers
    ros::Timer _timer;
    // Subscribers
    ros::Subscriber _subscriber;
    // Publishers
    ros::Publisher _publisher;
    // Transforms
    tf2_ros::TransformBroadcaster tfb;
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::TransformStamped wheelTransforms;
    double linear_x = 0;
    double linear_y = 0;
    double linear_z = 0;
    double angular_x = 0;
    double angular_y = 0;
    double angular_z = 0;
    // Parameters
    double _timer_rate;
};
