#include <ros/ros.h>
#include <prop_mapper/Prop.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath> 
#include <ros/console.h>
#include <string>
#include <iostream>
#include <nav_msgs/Odometry.h>


std::string TAG = "WP_SENDER: ";
/**
* @brief Maps props detected by LiDAR to local coordinate frame
* 
* Takes the x&y distances from the robot to the prop, and the robot's current local position and orientation to map the 
* prop in the local coordinate frame. 
*
*/
class CoordFinder {
public:
    CoordFinder()
    {
        
        // Specify ROS topic names - using parameters for this so that we can change names from launch files
        private_nh_.param<std::string>("prop_topic", prop_distances_topic_, "/prop_xy_dist");
        private_nh_.param<std::string>("local_pose_topic", local_pose_topic_, "/mavros/local_position/pose");
        
        // set up subscribers
        sub_pose_ = nh_.subscribe(local_pose_topic_, 1, &CoordFinder::poseCallback, this);
        sub_prop_distances_ = nh_.subscribe(prop_distances_topic_, 1, &CoordFinder::propCallback, this);

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

    geometry_msgs::PoseStamped pose_msg_;

    /**
    @brief Gets the robot's current position and orientation
    */
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        // ROS_ERROR_STREAM("Hello");
        pose_msg_ = *msg;
        ROS_INFO_STREAM("boat_x: " << pose_msg_.pose.position.x
        << "boat_y: " << pose_msg_.pose.position.y
        << "boat_z: " << pose_msg_.pose.position.z
        << "qw: " << pose_msg_.pose.orientation.w
        << "qx: " << pose_msg_.pose.orientation.x
        << "qy: " << pose_msg_.pose.orientation.y
        << "qz: " << pose_msg_.pose.orientation.z
        );
    }

    /**
    @brief Gets the prop's coordinates relative to the position and orientation of the robot and converts to local coordinates and publishes the prop's local coordinates
    */
    void propCallback(const prop_mapper::Prop::ConstPtr& msg)
    {

        // Relative prop coordinates
        double prop_x_rel = msg->vector.x;
        double prop_y_rel = msg->vector.y;
        double prop_z_rel = msg->vector.z;

        // Fix for gazebo
        // double prop_x_rel = msg->vector.y;
        // double prop_y_rel = msg->vector.x;
        // double prop_z_rel = msg->vector.z;

        // Convert relative coordinates to local coordinates
        double qw = pose_msg_.pose.orientation.w;
        double qx = pose_msg_.pose.orientation.x;
        double qy = pose_msg_.pose.orientation.y; 
        double qz = pose_msg_.pose.orientation.z;

        // Boat coordinates
        double boat_x = pose_msg_.pose.position.x;
        double boat_y = pose_msg_.pose.position.y;
        double boat_z = pose_msg_.pose.position.z;

        // Fix for gazebo
        // double boat_x = pose_msg_.pose.position.y;
        // double boat_y = -pose_msg_.pose.position.x;
        // double boat_z = pose_msg_.pose.position.z;

        double siny_cosp = 2 * (qw * qz + qx * qy);
        double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);

        double yaw = std::atan2(siny_cosp, cosy_cosp);

        double local_x = prop_x_rel*cos(yaw - M_PI_2) - prop_y_rel*sin(yaw - M_PI_2);
        double local_y = prop_x_rel*sin(yaw - M_PI_2) + prop_y_rel*cos(yaw - M_PI_2);
        
        // Create and publish the Prop message with the prop's local coordinates 
        prop_mapper::Prop local_prop_msg;
        local_prop_msg.prop_label = msg->prop_label;

        ROS_ERROR_STREAM(TAG << " \nprop_x_rel: " << prop_x_rel
        << " \nprop_y_rel: " << prop_y_rel
        << " \nprop_z_rel: " << prop_z_rel
        << " \nboat_x: " << boat_x
        << " \nboat_y: " << boat_y
        << " \nboat_z: " << boat_z
        << " \nqw: " << qw
        << " \nqx: " << qx
        << " \nqy: " << qy
        << " \nqz: " << qz
        << " \nlocal_x: " << local_x
        << " \nlocal_y: " << local_y
        << " \nglobal_prop_x: " << boat_x + local_x
        << " \nglobal_prop_y: " << boat_y + local_y
        );

        // Coordinates
        local_prop_msg.vector.x = boat_x + local_x;
        local_prop_msg.vector.y = boat_y + local_y;
        local_prop_msg.vector.z = boat_z;

        pub_prop_coords_.publish(local_prop_msg);
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "coord_finder_node");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
        ros::console::notifyLoggerLevelsChanged();
    CoordFinder coord_finder;
    coord_finder.spin();
    return 0;
}