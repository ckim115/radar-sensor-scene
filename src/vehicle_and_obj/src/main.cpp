// ROS
#include <ros/ros.h>
// Local
#include "vehicle_and_obj/Sim_Node.h"
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "main"); // init ros
    
    Sim_Node node;
    
    ros::spin();
}

