#include <ros/ros.h>
#include <prop_mapper/PropPolarCoords.h>
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
    CoordFinder() : nh_(""), private_nh_("~")
    {
        
        // Specify ROS topic names - using parameters for this so that we can change names from launch files
        private_nh_.param<std::string>("prop_topic", prop_distances_topic_, "/prop_polar_coords");
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
    std::string TAG = "COORD_FINDER: ";

    geometry_msgs::PoseStamped pose_msg_;
    prop_mapper::PropPolarCoords prop_msg_;

    bool isValidLabel(std::string label)
    {
        std::vector<std::string> prop_labels = {"red_marker", "green_marker", "red_buoy", "green_buoy", "yellow_buoy", "blue_buoy", "black_buoy"}; // TODO update with all props
        auto it = std::find(prop_labels.begin(), prop_labels.end(), label);

        return it != prop_labels.end();
    }

    double getGazeboHeading(const geometry_msgs::Quaternion q)
    {
        // yaw (z-axis rotation)
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        double yaw = std::atan2(siny_cosp, cosy_cosp);

        return yaw;
    }

    /**
    @brief Gets the robot's current position and orientation publishes the prop's local coordinates
    */
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        ROS_DEBUG_STREAM(TAG << "poseCallback() called");
        // check that the prop msg has been set before publishing a prop
        if (isValidLabel(prop_msg_.prop_label))
        {
            pose_msg_ = *msg;

            ROS_DEBUG_STREAM(TAG << "heading: " << getGazeboHeading(pose_msg_.pose.orientation)*(180/M_PI));

            double radius = prop_msg_.radius;
            double angle = prop_msg_.angle;

            // Convert relative polar coordinates to local coordinates 

            double heading = getGazeboHeading(pose_msg_.pose.orientation);

            double prop_x_aligned = radius*cos(angle-((M_PI/2)-heading));
            double prop_y_aligned = radius*sin(angle-((M_PI/2)-heading));


            double prop_x =  pose_msg_.pose.position.x + prop_x_aligned;
            double prop_y =  pose_msg_.pose.position.y + prop_y_aligned;

            // Create and publish the Prop message with the prop's local coordinates 
            prop_mapper::Prop local_prop_msg;
            local_prop_msg.prop_label = prop_msg_.prop_label;
            local_prop_msg.point.x = prop_x;
            local_prop_msg.point.y = prop_y;

            ROS_DEBUG_STREAM(TAG << "x robot = " << pose_msg_.pose.position.y);
            ROS_DEBUG_STREAM(TAG << "y robot = " << -pose_msg_.pose.position.x);
            ROS_DEBUG_STREAM(TAG << "x prop = " << prop_x);
            ROS_DEBUG_STREAM(TAG << "y prop = " << prop_y);
            pub_prop_coords_.publish(local_prop_msg);
        }
        else
        {
            ROS_DEBUG_STREAM(TAG << "No valid props");
        }

    }




    /**
    @brief Gets the prop message
    */
    void propCallback(const prop_mapper::PropPolarCoords::ConstPtr& msg)
    {
        ROS_DEBUG_STREAM(TAG << "propCallback() called");
        prop_msg_ = *msg;
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