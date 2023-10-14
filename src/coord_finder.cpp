#include <ros/ros.h>
#include <prop_mapper/PropDistances.h>
#include <prop_mapper/Prop.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath> 
#include <ros/console.h>

/**
* @brief Maps props detected by LiDAR to local coordinate frame
* 
* Takes the x&y distances from the robot to the prop, and the robot's current local position to map the 
* prop in the local coordinate frame. 
*
*/
class CoordFinder {
public:
    CoordFinder()
    {
        // get ROS parameters
        private_nh_.param<double>("coord_mapping_error_estimation", coord_mapping_error_estimation_, 0.0);
        
        // Specify ROS topic names - using parameters for this so that we can change names from launch files
        private_nh_.param<std::string>("prop_topic", prop_distances_topic_, "/prop_angle_range");
        private_nh_.param<std::string>("local_pose_topic", local_pose_topic_, "/local_position/pose");
        
        // set up subscribers
        sub_pose_ = nh_.subscribe(local_pose_topic_, 1, &CoordFinder::poseCallback, this);
        sub_prop_distances_ = nh_.subscribe(prop_distances_topic_, 1, &DistanceFinder::propCallback, this);

        // set up publishers
        pub_prop_coords_ = nh_.advertise<prop_mapper::Prop>("/completed_props", 1);
        
    }

    void spin() {
        ros::Rate rate(10); // 10 Hz
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:

    ros::NodeHandle nh_;            
    ros::NodeHandle private_nh_;                  
    ros::Subscriber sub_pose_;                     
    ros::Subscriber sub_prop_distances_;        
    ros::Publisher pub_prop_coords_;             
    std::string prop_distances_topic_;               
    std::string local_pose_topic_;   
    
    double coord_mapping_error_estimation_;       //!< used to place a safety range around props to avoid duplicates in the map
    
    prop_mapper::PropDistances prop_distances_msg_;
    geometry_msgs::PoseStamped pose_msg_;

    double robot_x_;
    double robot_y_;
    double robot_z_;

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        robot_x_ = msg->pose->position->x;
        robot_y_ = msg->pose->position->y;
        robot_z_ = msg->pose->position->z;
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
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
        ros::console::notifyLoggerLevelsChanged();
    CoordFinder coord_finder;
    coord_finder.spin();
    return 0;
}