#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <tf2_msgs/TFMessage.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <tf2/LinearMath/Quaternion.h>
#include <urdf/model.h>
#include <random>
#include <vector>
#include <numeric>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

struct RadarPoint
{
  PCL_ADD_POINT4D;
  float range;
  float azimuth;
  float elevation;
  float vrel_rad;  // Doppler velocity
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(RadarPoint,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, range, range)
                                  (float, azimuth, azimuth)
                                  (float, elevation, elevation)
                                  (float, vrel_rad, vrel_rad));
ros::Publisher pcl_pub;
pcl::PointCloud<RadarPoint> pointcloud;

struct Vehicle
{
    double x {}, y {}, z {}; //x y z
    double vel_x {}, vel_y {}, vel_z {};
    double orientation {};
};

struct Plane
{
    double x {}, y {}, z {};
};

struct Obstacle
{
    double w {}, l {}, h {}; //w l h
    double x {}, y {}, z {};
    std::vector<Plane> planes {};
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

//With a dot product, if we treat the two vectors as directions can determine if they face each other or not
double in_view(std::vector<double> v1, std::vector<double> v2) {
    //unitize the vectors to get direction vectors
    double length = 0;
    for(double i : v1) length += pow(i,2);
    length = sqrt(length);
    std::vector<double> new_v1{};
    for(double i : v1) new_v1.push_back(i/length);
    //ROS_INFO("vehicle_v: <%f, %f, %f>", v1[0], v1[1], v1[2]);
    
    length = 0;
    for(double i : v2) length += pow(i,2);
    length = sqrt(length);
    std::vector<double> new_v2{};
    for(double i : v2) new_v2.push_back(i/length);
    //ROS_INFO("plane_v: <%f, %f, %f>", v2[0], v2[1], v2[2]);
    
    double dotProduct = 0;
    for(int i = 0; i < v1.size(); i++) {
        dotProduct += v1[i]*v2[i];
    }
    //ROS_INFO("Dot Prod: %f", dotProduct);
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

void gen_rand_num(Obstacle o, Plane p) {
    std::default_random_engine re{std::random_device{}()};
    std::uniform_real_distribution<double> genx(o.x - o.w/2, o.x + o.w/2);
    std::uniform_real_distribution<double> geny(o.y - o.l/2, o.y + o.l/2);
    std::uniform_real_distribution<double> genz(-o.h/2, o.h/2);
    rand_pt.x = p.x;
    rand_pt.y = p.y;
    rand_pt.z = p.z;
    if(p.x == o.x+o.w/2 || p.x == o.x-o.w/2) {
        rand_pt.y = geny(re);
        rand_pt.z = genz(re);
    } else if (p.y == o.y+o.l/2 || p.y == o.y-o.l/2) {
        rand_pt.x = genx(re);
        rand_pt.z = genz(re);
    } else {
        rand_pt.x = genx(re);
        rand_pt.y = geny(re);
    }
}

// defines the random points
bool define_random_point(Obstacle o) {
    std::vector<Plane> visible_planes {};
    std::vector<double> obj_v = {vehicle.x - o.x, vehicle.y - o.y};
    std::vector<double> plane_v;
    for(Plane p: o.planes) {
        plane_v = {p.x-o.x, p.y-o.y, p.z-o.z};
        if(in_view(obj_v, plane_v) > 0) visible_planes.push_back(p);
    }
    if(visible_planes.empty()) return false;
    int randIndex = rand()%visible_planes.size();
    gen_rand_num(o, visible_planes[randIndex]);
    return true;
}

double calc_range(double x1, double y1, double z1, double x2, double y2, double z2) {
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2));
}

void generate_point_information() {
    ROS_INFO("vehicle velocity: (%f, %f, %f)", vehicle.vel_x, vehicle.vel_y, vehicle.vel_z);
    // Range
    double range = calc_range(vehicle.x, vehicle.y, vehicle.z, rand_pt.x, rand_pt.y, rand_pt.z);
    // Azimuth
    double azimuth = atan2(vehicle.y - rand_pt.y, vehicle.x - rand_pt.x);
    // Elevation angle
    double elevation = asin(vehicle.z - rand_pt.z/range);
    // Doppler velocity
    double dop_vel = -vehicle.vel_x*cos(azimuth)*cos(elevation)-vehicle.vel_y*sin(azimuth)*cos(elevation)-vehicle.vel_z*sin(elevation);
    
    RadarPoint point;
    point.x = rand_pt.x;
    point.y = rand_pt.y;
    point.z = rand_pt.z;
    
    point.range = range;
    point.azimuth = azimuth;
    point.elevation = elevation;
    point.vrel_rad = dop_vel;  // Doppler velocity
    
    pointcloud.push_back(point);
    
    pointcloud.header.frame_id = "world";
    ros::Time time_st = ros::Time::now();
    pointcloud.header.stamp = time_st.toNSec()/1e3;
    pcl_pub.publish(pointcloud);
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
            obj_v = {vehicle.x - o.x, vehicle.y - o.y};
            temp_distance = calc_range(vehicle.x, vehicle.y, vehicle.z, o.x, o.y, o.z);
            if(temp_distance <= min_distance && in_view(vehicle_v, obj_v) > 0) {
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
        
        for(int i = -1; i < 2; i+=2) {
            obs.planes.push_back(Plane{obs.x+i*obs.w*0.5,obs.y,obs.z});
            obs.planes.push_back(Plane{obs.x,obs.y+i*obs.l*0.5,obs.z});
        }
        //cant see bottom (ground)
        //for case of top of object
        if(vehicle.z >= obs.z+obs.h*0.5) obs.planes.push_back(Plane{obs.x,obs.y,obs.z+obs.h*0.5});
        
        ROS_INFO("object position: (%f, %f, %f)", obs.x, obs.y, obs.z);
        
        obstacles.push_back(obs);
    }
}

// Print coordinates and rotation of vehicle
void callback_vehicle(const geometry_msgs::PoseStamped msg)
{
    vehicle.x = msg.pose.position.x;
    vehicle.y = msg.pose.position.y;
    vehicle.z = msg.pose.position.z;
    
    // convert to Euler; need to make new quaternion for this
    tf2::Quaternion q(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
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
    ROS_INFO("callback vehicle velocity: (%f, %f, %f)", vehicle.vel_x, vehicle.vel_y, vehicle.vel_z);
    vehicle.vel_x = msg.linear.x;
    vehicle.vel_y = msg.linear.y;
    vehicle.vel_z = msg.linear.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n("~");
    
    vis_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 0 );
    pcl_pub = n.advertise<pcl::PointCloud<RadarPoint>>("/radar_pointcloud_topic", 0);
    
    ros::Subscriber sub_vel = n.subscribe("/vehicle_velocity", 10, callback_velocity);
    ros::Subscriber sub_vehicle = n.subscribe("/vehicle_position", 10, callback_vehicle);
    ros::Subscriber sub_obj = n.subscribe("/tf_static", 10, callback_obj);
    ros::spin();
    
    return 0;
}
