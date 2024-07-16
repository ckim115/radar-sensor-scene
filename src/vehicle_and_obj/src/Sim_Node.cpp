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
//  - Successfully subscribe to twist cmds
//  - Confirmed that model displayed & responds to cmds
//      - Note: must launch robot.launch, rosrun node, and rosrun keyboard
//  - Transform pose of model based on twist cmds and time
//  - Periodically publish position of model
Sim_Node::Sim_Node()
{
    angle = 0;
    ros::NodeHandle node("~");
    
    // Setup publisher/subscriber: Publish to vehicle position, subscribe to cmd_vel
    _publisher = node.advertise<geometry_msgs::PoseStamped>("/vehicle_position", 10);
    //rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=broadcaster/cmd_vel
    _subscriber = node.subscribe("/broadcaster/cmd_vel", 1000, &Sim_Node::update_position, this);
    
    // Setup Timers: node_handle.createTimer(duration, callback, instance)
    _timer = node.createTimer(ros::Duration { 0.1 }, &Sim_Node::timerCallback, this); // run callback every 0.1 seconds
}

void Sim_Node::timerCallback(const ros::TimerEvent &event)
{
    if(twist.linear.x != 0 || twist.linear.y != 0 || twist.linear.z != 0) {
        // compute time since last callback
        ros::Duration delta_time = event.current_real - event.last_real;
        double secs = delta_time.toSec();
        
        poseStamped.header.frame_id = "body_link";
        poseStamped.header.stamp = ros::Time::now();
        
        geometry_msgs::TransformStamped t;
        t.header.frame_id = "world";
        t.child_frame_id = "body_link";
        t.header.stamp = ros::Time::now();
        //for ackermann, ignore whether its anything but moving. if >0 clockwise, <0 counterclockwise
        // rotate car by (angle in+out)/2, move x += distance*cos(angle) y += distance*sin(angle)
        angle += 0.1999431628;
        double vel = sqrt(pow(twist.linear.x,2) + pow(twist.linear.y,2) + pow(twist.linear.z,2));
        linear_x += vel*secs*cos(angle); // linear_x += distance*cos(angle)
        linear_y += vel*secs*sin(angle);
        linear_z += twist.linear.z*secs;
        //    linear_x += twist.linear.x*secs; // linear_x += distance*cos(angle)
        //    linear_y += twist.linear.y*secs;
        //    linear_z += twist.linear.z*secs;
        
        angular_x += twist.angular.x*secs;
        angular_y += twist.angular.y*secs;
        angular_z += twist.angular.z*secs;
        
        t.transform.translation.x = linear_x;
        t.transform.translation.y = linear_y;
        t.transform.translation.z = linear_z;
        
        tf2::Quaternion q;
        q.setRPY(angular_x, angular_y, angular_z);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        
        //set all for posestamped as well
        poseStamped.pose.position.x = linear_x;
        poseStamped.pose.position.y = linear_y;
        poseStamped.pose.position.z = linear_z;
        
        poseStamped.pose.orientation.w = q.x();
        poseStamped.pose.orientation.w = q.y();
        poseStamped.pose.orientation.w = q.z();
        poseStamped.pose.orientation.w = q.w();
        
        ROS_INFO("linear: (%f, %f, %f)", t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
        
        // Publish tranformation
        _publisher.publish(poseStamped);
        
        tfb.sendTransform(t);
    }
    
//    buf << "timerCallback(): delta_time = " << delta_time;
//    // publish a message
//    std_msgs::String msg;
//    msg.data = buf.str();
    //ROS_INFO("Time: %s", msg.data.c_str());
}

//determine next position of vehicle from twist & time elapsed
    //maybe calculate where the front wheels would be and use that to determine it?
        // calculate the wheel's next position to determine position of vehicle relative
                // x = (wheel_in.x + wheel_out.x)/2, etc
        // the angle of the vehicle += (angle_in+angle_out)/2
void Sim_Node::update_position(const geometry_msgs::Twist& msg) {
    //'teleop_twist_keyboard' only sends an update message when you press a key to change the velocity. So, the position will only update exactly once when you press a key. You should add a geometry_msgs::Twist member variable to the Sim_Node class, and have the _subscriber callback update that geometry_msgs::Twist each time it is called. That is we want to store the most recent twist message received by the node. You can then update the position with this stored twist in the _timer callback at a regular interval.
    twist = msg;
    
//    transformStamped.header.frame_id = "world";
//    transformStamped.child_frame_id = "body_link";
//    transformStamped.header.stamp = ros::Time::now();
//    
//    linear_x += msg.linear.x*time_elapsed;
//    linear_y += msg.linear.y*time_elapsed;
//    linear_z += msg.linear.z*time_elapsed;
//    
//    angular_x += msg.angular.x*time_elapsed;
//    angular_y += msg.angular.y*time_elapsed;
//    angular_z += msg.angular.z*time_elapsed;
//    
//    transformStamped.transform.translation.x = linear_x;
//    transformStamped.transform.translation.y = linear_y;
//    transformStamped.transform.translation.z = linear_z;
//    
//    tf2::Quaternion q;
//        q.setRPY(angular_x, angular_y, angular_z);
//    transformStamped.transform.rotation.x = q.x();
//    transformStamped.transform.rotation.y = q.y();
//    transformStamped.transform.rotation.z = q.z();
//    transformStamped.transform.rotation.w = q.w();
//    
//    ROS_INFO("linear: (%f, %f, %f)", transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
//    
//    // Publish tranformation
//    _publisher.publish(transformStamped);
//    
//    tfb.sendTransform(transformStamped);
}



//    wheelTransforms.header.frame_id = "body_link";
//    wheelTransforms.child_frame_id = "wheel3_joint";//in angle
//    wheelTransforms.header.stamp = ros::Time::now();
//    tf2::Quaternion q_Wheel(wheelTransforms.transform.rotation.x, wheelTransforms.transform.rotation.y, wheelTransforms.transform.rotation.z, wheelTransforms.transform.rotation.w);
//    // get angle for orientation
//    double orientation = q_Wheel.getAngle();
//    ROS_INFO("linear: (%f, %f, %f)", wheelTransforms.transform.translation.x, wheelTransforms.transform.translation.y, wheelTransforms.transform.translation.z);
//    ROS_INFO("orientation (should be 1.393570942: %f", orientation);
