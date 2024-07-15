// Standard Library
#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <nav_msgs/Odometry.h>

// Local Includes
#include "vehicle_and_obj/Sim_Node.h"

//  - Successfully subscribe to twist cmds
//  - Confirmed that model displayed & responds to cmds
//      - Note: must launch robot.launch, rosrun node, and rosrun keyboard
//  - Transform pose of model based on twist cmds and time
//  - Periodically publish position of model
double angle_in = 0;
double angle_out = 0;
Sim_Node::Sim_Node()
{
    ros::NodeHandle node("~");
    
    // Setup Timers: node_handle.createTimer(duration, callback, instance)
    _timer = node.createTimer(ros::Duration { 0.1 }, &Sim_Node::timerCallback, this); // run callback every 0.1 seconds
    
    // Setup publisher/subscriber: Publish to vehicle position, subscribe to cmd_vel
    _publisher = node.advertise<geometry_msgs::TransformStamped>("/vehicle_position", 10);
    //rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=broadcaster/cmd_vel
    _subscriber = node.subscribe("/broadcaster/cmd_vel", 1000, &Sim_Node::update_position, this);
}

void Sim_Node::timerCallback(const ros::TimerEvent &event)
{
    std::stringstream buf;
    // compute time since last callback
    ros::Duration delta_time = event.current_real - event.last_real;
    buf << "timerCallback(): delta_time = " << delta_time;
    // publish a message
    std_msgs::String msg;
    msg.data = buf.str();
    //ROS_INFO("Time: %s", msg.data.c_str());
}

//determine next position of vehicle from twist & time elapsed
    //maybe calculate where the front wheels would be and use that to determine it?
        // calculate the wheel's next position to determine position of vehicle relative
                // x = (wheel_in.x + wheel_out.x)/2, etc
        // the angle of the vehicle += (angle_in+angle_out)/2
void Sim_Node::update_position(const geometry_msgs::Twist& msg) {
    double time_elapsed = ros::Time::now().toSec() - transformStamped.header.stamp.toSec();
    wheelTransforms.header.frame_id = "body_link";
    wheelTransforms.child_frame_id = "wheel3_joint";//in angle
    wheelTransforms.header.stamp = ros::Time::now();
    tf2::Quaternion q_Wheel(wheelTransforms.transform.rotation.x, wheelTransforms.transform.rotation.y, wheelTransforms.transform.rotation.z, wheelTransforms.transform.rotation.w);
    // get angle for orientation
    double orientation = q_Wheel.getAngle();
    ROS_INFO("linear: (%f, %f, %f)", wheelTransforms.transform.translation.x, wheelTransforms.transform.translation.y, wheelTransforms.transform.translation.z);
    ROS_INFO("orientation (should be 1.393570942: %f", orientation);
    
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "body_link";
    transformStamped.header.stamp = ros::Time::now();
    
    linear_x += msg.linear.x*time_elapsed;
    linear_y += msg.linear.y*time_elapsed;
    linear_z += msg.linear.z*time_elapsed;
    
    angular_x += 0.5*msg.angular.x*time_elapsed;
    angular_y += 0.5*msg.angular.y*time_elapsed;
    angular_z += 0.5*msg.angular.z*time_elapsed;
    
    transformStamped.transform.translation.x = linear_x;
    transformStamped.transform.translation.y = linear_y;
    transformStamped.transform.translation.z = linear_z;
    
    tf2::Quaternion q;
        q.setRPY(angular_x, angular_y, angular_z);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    
    // Publish tranformation
    _publisher.publish(transformStamped);
    
    tfb.sendTransform(transformStamped);
}
