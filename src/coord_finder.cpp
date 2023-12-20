#include <ros/ros.h>
#include <prop_mapper/PropDistances.h>
#include <prop_mapper/Prop.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath> 
#include <ros/console.h>

/**
* @brief Maps props detected by LiDAR to local coordinate frame
* 
* Takes the x&y distances from the robot to the prop, and the robot's current local position and orientation to map the 
* prop in the local coordinate frame 
*
*/
class CoordFinder {
public:
    CoordFinder()
    {
        
        // Specify ROS topic names - using parameters for this so that we can change names from launch files
        private_nh_.param<std::string>("prop_topic", prop_distances_topic_, "/prop_xy_dist");
        private_nh_.param<std::string>("local_pose_topic", local_pose_topic_, "/local_position/pose");
        
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
        pose_msg_ = *msg;
    }

    /**
    @brief Gets the prop's coordinates relative to the position and orientation of the robot and converts to local coordinates and publishes the prop's local coordinates
    */
    void propCallback(const prop_mapper::Prop::ConstPtr& msg)
    {

        // Relative prop coordinates
        double prop_x_rel_ = msg->vector.x;
        double prop_y_rel_ = msg->vector.y;
        double prop_z_rel_ = msg->vector.z;

        // Convert relative coordinates to local coordinates - TODO

        
        // Create and publish the Prop message with the prop's local coordinates 
        prop_mapper::Prop local_prop_msg;
        local_prop_msg.prop_label = msg->prop_label;

        // Coordinates TODO

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