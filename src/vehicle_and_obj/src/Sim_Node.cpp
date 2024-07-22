// Standard Library
#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

// Local Includes
#include "vehicle_and_obj/Sim_Node.h"


double angle;
geometry_msgs::TransformStamped t;
//  - Successfully subscribe to twist cmds
//  - Confirmed that model displayed & responds to cmds
//      - Note: must launch robot.launch, rosrun node, and rosrun keyboard
//  - Transform pose of model based on twist cmds and time
//  - Periodically publish position of model
Sim_Node::Sim_Node()
{
    angle = 0;
    ros::NodeHandle node("~");
    
    poseStamped.header.frame_id = "body_link";
    t.header.frame_id = "world";
    t.child_frame_id = "body_link";
    poseStamped.pose.position.y = -3;
    t.transform.translation.y = -3;
    
    // Setup publisher/subscriber: Publish to vehicle position, subscribe to cmd_vel
    _publisher = node.advertise<geometry_msgs::PoseStamped>("/vehicle_position", 10);
    twist_publisher = node.advertise<geometry_msgs::Twist>("/vehicle_velocity", 10);
    //rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=broadcaster/cmd_vel
    _subscriber = node.subscribe("/broadcaster/cmd_vel", 1000, &Sim_Node::update_position, this);
    
    // Setup Timers: node_handle.createTimer(duration, callback, instance)
    _timer = node.createTimer(ros::Duration { 0.1 }, &Sim_Node::timerCallback, this); // run callback every 0.1 seconds
}

// updates position of vehicle whenever twist from teleop_twist_keyboard received
void Sim_Node::timerCallback(const ros::TimerEvent &event)
{
    if(twist.linear.x != 0 || twist.linear.y != 0 || twist.linear.z != 0) {
        // compute time since last callback
        ros::Duration delta_time = event.current_real - event.last_real;
        double secs = delta_time.toSec();
        
        poseStamped.header.stamp = ros::Time::now();
        ROS_INFO("pos posestamped: (%f, %f, %f)", poseStamped.pose.position.x, poseStamped.pose.position.y, poseStamped.pose.position.z);
        
        t.header.stamp = ros::Time::now();
        
        
        //NEW: try ackermann with distance
        double vel = sqrt(pow(twist.linear.x,2) + pow(twist.linear.y,2));
        double dist = vel*secs;
        if(twist.linear.x < 0 || twist.linear.y < 0) {
            dist *= -1;
        }
        double angle_in = atan2(dist,2.7);
        double angle_out = atan2(dist,3.3);
        angle += (angle_in+angle_out)/2;
        linear_x = dist*cos(angle);
        linear_y = dist*sin(angle);
        
        twistPub.linear.x = vel*cos(angle);
        twistPub.linear.y = vel*sin(angle);
//        twistPub.linear.z = 0;
//        twistPub.angular.x = 0;
//        twistPub.angular.y = 0;
//        twistPub.angular.z = 0;
        ROS_INFO("vehicle velocity: (%f, %f, %f)", twistPub.linear.x, twistPub.linear.y, twistPub.linear.z);
        
        angular_x += twist.angular.x*secs;
        angular_y += twist.angular.y*secs;
        angular_z = angle;
        t.transform.translation.x += linear_x;
        t.transform.translation.y += linear_y;
        
        tf2::Quaternion q;
        q.setRPY(angular_x, angular_y, angular_z);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        
        //set all for posestamped as well
        poseStamped.pose.position.x += linear_x;
        poseStamped.pose.position.y += linear_y;
        poseStamped.pose.position.z += linear_z;
        
        poseStamped.pose.orientation.w = q.x();
        poseStamped.pose.orientation.w = q.y();
        poseStamped.pose.orientation.w = q.z();
        poseStamped.pose.orientation.w = q.w();
        
        // Publish tranformation
        _publisher.publish(poseStamped);
        twist_publisher.publish(twistPub);
        
        // send transform
        tfb.sendTransform(t);
    }
}

// updates twist var with msg for use in timer
void Sim_Node::update_position(const geometry_msgs::Twist& msg) {
    twist = msg;
}
