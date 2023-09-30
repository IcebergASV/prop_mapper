#include <ros/ros.h>
#include <navigation_pkg/Compass.h> 
#include <navigation_pkg/PropInProgress.h>
#include <navigation_pkg/Prop.h>
#include <navigation_pkg/SimpleGPS.h> //temporary
#include <geographic_msgs/GeoPoint.h>
#include <cmath> 

class CoordFinder {
public:
    CoordFinder()
    {
        gps_sub_ = nh_.subscribe("/rectbot_coords", 1, &CoordFinder::gpsCallback, this);
        compass_sub_ = nh_.subscribe("/rectbot_heading", 1, &CoordFinder::compassCallback, this );
        prop_sub_ = nh_.subscribe("/prop_closest_point", 1, &CoordFinder::propCallback, this);
        prop_pub_ = nh_.advertise<navigation_pkg::Prop>("/completed_props", 1);
        //nh_.getParam("safety_range", safety_range); not working right now
        //nh_.getParam("degrees_lat_per_meter", degrees_lat_per_meter);
        //nh_.getParam("degrees_lon_per_meter", degrees_lon_per_meter);
        
    }

    void spin() {
        ros::Rate rate(10); // 10 Hz
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    void gpsCallback(const navigation_pkg::SimpleGPS::ConstPtr& msg)
    {
        robot_lat_ = msg->latitude;
        robot_lon_ = msg->longitude;
        robot_alt_ = msg->altitude;
    }

    void compassCallback(const navigation_pkg::Compass::ConstPtr& msg)
    {
        robot_heading = msg->heading;
    }

    void propCallback(const navigation_pkg::PropInProgress::ConstPtr& msg)
    {
        //// Calculate the GPS coordinates of the prop
        double dist = msg->closest_pnt_dist;
        double angle = msg->closest_pnt_angle;
        double prop_heading;
        
        if ((robot_heading - angle) > (2*M_PI))
            prop_heading = robot_heading - angle - (2* M_PI);
        else 
            prop_heading = robot_heading - angle;

        double north_dist = dist * cos(prop_heading);
        double east_dist = dist * sin(prop_heading);

        double lat_diff = north_dist * degrees_lat_per_meter;
        double lon_diff = east_dist * degrees_lon_per_meter;

        double prop_lat = robot_lat_ + lat_diff;
        double prop_lon = robot_lon_ + lon_diff;
        double prop_alt = robot_alt_;


        double lat_safety_range = degrees_lat_per_meter * safety_range;
        double lon_safety_range = degrees_lon_per_meter * safety_range;

        
        // Create and publish the Prop message with the prop coordinates
        navigation_pkg::Prop prop_msg;
        prop_msg.prop_type = msg->prop_type;

        prop_msg.prop_coords.latitude = prop_lat;
        prop_msg.prop_coords.longitude = prop_lon;
        prop_msg.prop_coords.altitude = prop_alt;

        prop_msg.prop_coord_range.min_latitude = prop_lat - lat_safety_range;
        prop_msg.prop_coord_range.max_latitude = prop_lat + lat_safety_range;
        prop_msg.prop_coord_range.min_longitude = prop_lon - lon_safety_range;
        prop_msg.prop_coord_range.max_longitude = prop_lon + lon_safety_range;

        prop_pub_.publish(prop_msg);
    }

    ros::NodeHandle nh_;
    ros::Subscriber gps_sub_;
    ros::Subscriber prop_sub_;
    ros::Subscriber compass_sub_;
    ros::Publisher prop_pub_;
    double robot_lat_;
    double robot_lon_;
    double robot_alt_;
    double robot_heading;
    double safety_range = 0.1;
    double degrees_lat_per_meter = 8.9942910391e-06;
    double degrees_lon_per_meter = 1.32865719904e-05;




};

int main(int argc, char** argv) {
    ros::init(argc, argv, "coord_finder_node");
    CoordFinder coord_finder;
    coord_finder.spin();
    return 0;
}