#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <tf2_msgs/TFMessage.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <tf2/LinearMath/Quaternion.h>
#include <urdf/model.h>
#include <random>
#include <vector>
#include <numeric>

// TODO: More detailed random point generation; currently looks for the closest object and determines from there
    // maybe compare the position of all objects facing the vehicle with each other, randomly choose one, then check the height of all objects between it and the vehicle?
struct Vehicle
{
    double x {}, y {}, z {}; //x y z
    double vel_x {}, vel_y {}, vel_z {};
    double orientation {};
};

struct Obstacle
{
    double w {}, l {}, h {}; //w l h
    double x {}, y {}, z {};
};

struct Random_Point
{
    double x {}, y {}, z {};
};

ros::Publisher vis_pub;
Vehicle vehicle {};
Random_Point rand_pt {};
std::vector<Obstacle> obstacles {};

//create green marker to show random point
void generate_marker() {
    // marker initialization
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "object_marker";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = rand_pt.x;
    marker.pose.position.y = rand_pt.y;
    marker.pose.position.z = rand_pt.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    vis_pub.publish(marker);
}

//With a dot product, if we treat the two vectors as directions, then a value less than 0 means that v1 is behind v2
double in_view(std::vector<double> v1, std::vector<double> v2) {
    double v1_length = sqrt(pow(v1[0],2)+pow(v1[1],2));
    double v2_length = sqrt(pow(v2[0],2)+pow(v2[1],2));
    //unitize the vectors to get direction vectors
    std::vector<double> new_v1 = {v1_length*v1[0],v1_length*v1[1]};
    std::vector<double> new_v2 = {v2_length*v2[0],v2_length*v2[1]};
    //if dotProduct > 0, then we know that our vehicle is facing the object
    double dotProduct = inner_product(new_v1.begin(), new_v1.end(), new_v2.begin(), 0);
    return dotProduct;
}

//generates 1 or -1 randomly
int gen_pos_or_neg() {
    if ((rand()%2) == 0) {
        return 1;
    } else {
        return -1;
    }
}

void gen_rand_num(Obstacle o, bool x, bool y) {
    // randomly generate a point on the SA of the object
    std::default_random_engine re{std::random_device{}()};
    std::uniform_real_distribution<double> genz(0, o.h/2);
    std::uniform_real_distribution<double> geny(0, o.l/2);
    std::uniform_real_distribution<double> genx(0, o.w/2);
    
    int num_gen = rand()%2;
    int param_gen = rand()%3;
    
    if(num_gen == 0) {
        if(x && y) {// if it can be either x or y
            if(param_gen == 1) {
                rand_pt.x = gen_pos_or_neg()*genx(re) + o.x;
            } else if(param_gen == 2) {
                rand_pt.y = gen_pos_or_neg()*geny(re) + o.y;
            } else {
                rand_pt.z = gen_pos_or_neg()*genz(re) + o.z;
            }
        } else { // if it can only be x or only be y
            if(gen_pos_or_neg() > 0) {
                if(x) {
                    rand_pt.x = gen_pos_or_neg()*genx(re) + o.x;
                } else {
                    rand_pt.y = gen_pos_or_neg()*geny(re) + o.y;
                }
            } else {
                rand_pt.z = gen_pos_or_neg()*genz(re) + o.z;
            }
        }
        // x or y or z
    } else { // z and (x or y)
        rand_pt.z = gen_pos_or_neg()*genz(re) + o.z;
        if(x && y) { // if it can be either
            if(gen_pos_or_neg() > 0) { // choose x
                rand_pt.x = gen_pos_or_neg()*genx(re) + o.x;
            } else { // choose y
                rand_pt.y = gen_pos_or_neg()*geny(re) + o.y;
            }
        } else if(x) {
            rand_pt.x = gen_pos_or_neg()*genx(re) + o.x;
        } else {
            rand_pt.y = gen_pos_or_neg()*geny(re) + o.y;
        }
    }
}

// defines the random points
bool define_random_point(Obstacle o) {
    // edge_1 -> edge_2 && edge_1 -> edge 3
    // edge_2 -> edge_4
    // edge 3 -> edge_4
    
    //edge x point shared by edges 1 and 3
    double ex_13 = o.x+o.w/2;
    //edge y point shared by edges 1 and 2
    double ey_12 = o.y+o.l/2;
    //edge x point shared by edges 2 and 4
    double ex_24 = o.x-o.w/2;
    //edge y point shared by edges 3 and 4
    double ey_34 = o.y-o.l/2;
    
    if(vehicle.x > ex_13) {
        rand_pt.x = ex_13;
        if(vehicle.y > ey_12) {
            //1 2 3 x or y or z, OR z and (x or y)
            rand_pt.y = ey_12;
            gen_rand_num(o, true, true);
        } else if(vehicle.y > ey_34){
            rand_pt.y = ey_34;
            //can see 1 3; manipulate only z and/or y-axis
            gen_rand_num(o, false, true);
        } else {
            rand_pt.y = ey_34;
            // can see 1 3 4; x or y or z, OR z and (x or y)
            gen_rand_num(o, true, true);
        }
    } else if(vehicle.x > ex_24) {
        rand_pt.x = ex_24;
        if(vehicle.y > ey_12) {
            // can see 1 2; manipulate only z and/or x-axis
            rand_pt.y = ey_12;
            gen_rand_num(o, true, false);
        } else if(vehicle.y < ey_34) {
            // can see 3 4; manipulate only z and/or x-axis
            rand_pt.y = ey_34;
            gen_rand_num(o, true, false);
        } else {
            return false; // means it is inside the object
        }
    } else {
        rand_pt.x = ex_24;
        if(vehicle.y > ey_12) {
            // can see 1 2 4; x or y or z, OR z and (x or y)
            rand_pt.y = ey_12;
            gen_rand_num(o, true, true);
        } else if(vehicle.y > ey_34) {
            rand_pt.y = ey_34;
            // can see 2 4; manipulate only z and/or y-axis
            gen_rand_num(o, false, true);
        } else {
            rand_pt.y = ey_34;
            // can see 2 4 3; x or y or z, OR z and (x or y)
            gen_rand_num(o, true, true);
        }
    }
    return true;
}

double calc_range(double x1, double y1, double z1, double x2, double y2, double z2) {
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2));
}

void generate_point_information() {
    // Range
    double range = calc_range(vehicle.x, vehicle.y, vehicle.z, rand_pt.x, rand_pt.y, rand_pt.z);
    // Azimuth
    double azimuth = atan2(vehicle.y - rand_pt.y, vehicle.x - rand_pt.x);
    // Elevation angle
    double elevation = asin(vehicle.z - rand_pt.z/range);
    // Doppler velocity
    double dop_vel = -vehicle.vel_x*cos(azimuth)*cos(elevation)-vehicle.vel_y*sin(azimuth)*cos(elevation)-vehicle.vel_z*sin(elevation);
    
    ROS_INFO("\nrange: %f\nazimuth: %f\nelevation: %f\ndoppler velocity: %f", range, azimuth, elevation, dop_vel);
}

// get a random point from the h l w and x y z given
// should be on the SA; only vars should be h&l, h&w, just h or l or w
    // random num to determine how many, and from that another rand for which ones
bool rand_point() {
    // if there are no other objects where the vehicle is facing, no rand pos
    // else if there is an object at that position, get a random point that the vehicle can see
    // determine vehicle vector by cos/sin on the orientation to get its facing direction
    if(!obstacles.empty()) {
        int s = obstacles.size();
        Obstacle minObs = obstacles.front();
        double temp_distance;
        std::vector<double> vehicle_v = {cos(vehicle.orientation), sin(vehicle.orientation)};
        std::vector<double> obj_v;
        double min_distance = std::numeric_limits<double>::max();
        for(Obstacle o : obstacles) {
            // vector of object; always points towards vehicle
            obj_v = {o.x-vehicle.x, o.y-vehicle.y};
            temp_distance = calc_range(vehicle.x, vehicle.y, vehicle.z, o.x, o.y, o.z);
            if(temp_distance <= min_distance && in_view(vehicle_v, obj_v) >= 0) {
                minObs = o;
                min_distance = temp_distance;
            }
        }
        bool exists_point = define_random_point(minObs);
        if(exists_point) {
            ROS_INFO("rand_pos: (%f, %f, %f)", rand_pt.x, rand_pt.y, rand_pt.z);
            generate_marker();
            return true;
        }
    }
    ROS_INFO("No possible random points");
    return false;
}

// call when subscribing to tf/static
void callback_obj(const tf2_msgs::TFMessage msg)
{
    urdf::Model model;
    std::shared_ptr<const urdf::Link> link;
    std::shared_ptr<urdf::Box> box;
    double temp_distance;
    double temp_x, temp_y, temp_z;
    std::string param_name = "obj_description_";
    // getting position
    for(geometry_msgs::TransformStamped static_tf : msg.transforms) {
        // use urdf model to determine dimensions of obstacle
        param_name += static_tf.child_frame_id.back();
        ROS_INFO("%s, %s", param_name.c_str(), static_tf.child_frame_id.c_str());
        model.initParam(param_name);
        link = model.getLink(static_tf.child_frame_id);
        box = std::dynamic_pointer_cast<urdf::Box> (link->visual->geometry);
        
        Obstacle obs {box->dim.x, box->dim.y, box->dim.z, static_tf.transform.translation.x, static_tf.transform.translation.y, static_tf.transform.translation.z};
        obstacles.push_back(obs);
    }
}

// Print coordinates and rotation of vehicle
void callback_vehicle(const geometry_msgs::TransformStamped msg)
{
    vehicle.x = msg.transform.translation.x;
    vehicle.y = msg.transform.translation.y;
    vehicle.z = msg.transform.translation.z;
    
    // convert to Euler; need to make new quaternion for this
    tf2::Quaternion q(msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w);
    // get angle for orientation
    vehicle.orientation = q.getAngle();
    
    ROS_INFO("\nvehicle position: (%f, %f, %f)\norientation: %f", vehicle.x, vehicle.y, vehicle.z, vehicle.orientation);
    //whenever the vehicle changes, update the random point
    rand_point();
    generate_point_information();
}

// Get velocity of vehicle; required for doppler velocity
void callback_velocity(const geometry_msgs::Twist& msg)
{
    vehicle.vel_x = msg.linear.x;
    vehicle.vel_y = msg.linear.y;
    vehicle.vel_z = msg.linear.z;
    ROS_INFO("vehicle velocity: (%f, %f, %f)", vehicle.vel_x, vehicle.vel_y, vehicle.vel_z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n("~");
    
    vis_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 0 );
    ros::Subscriber sub_vel = n.subscribe("/broadcaster/cmd_vel", 1000, callback_velocity);
    ros::Subscriber sub_vehicle = n.subscribe("/vehicle_position", 10, callback_vehicle);
    ros::Subscriber sub_obj = n.subscribe("/tf_static", 10, callback_obj);
    ros::spin();
    
    return 0;
}
