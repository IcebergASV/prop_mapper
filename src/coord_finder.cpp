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
    CoordFinder()
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
        ros::Rate rate(1); // 10 Hz
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

    geometry_msgs::Vector3 getLocalCoords()
    {
        // Case 1: 0 < heading < 90 && prop in first relative coordinate
        geometry_msgs::Vector3 local_coords;



        return local_coords;
    }

    double getGazeboHeading(const geometry_msgs::Quaternion q)
    {
        // yaw (z-axis rotation)
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        double yaw = std::atan2(siny_cosp, cosy_cosp);

        // convert to gazebo heading
        double heading = yaw - (M_PI/2);
        if (heading < 0)
        {
            heading = heading + (2*M_PI);
        }
        return heading;
    }

    /**
    @brief Gets the robot's current position and orientation
    */
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        pose_msg_ = *msg;

        ROS_DEBUG_STREAM(TAG << "heading: " << getGazeboHeading(pose_msg_.pose.orientation)*(180/M_PI));

    }



    /**
    @brief Gets the prop's coordinates relative to the position and orientation of the robot and converts to local coordinates and publishes the prop's local coordinates
    */
    void propCallback(const prop_mapper::PropPolarCoords::ConstPtr& msg)
    {
        // Relative prop coordinates
        double radius = msg->radius;
        double angle = msg->angle;

        // Convert relative polar coordinates to local coordinates 

        double heading = getGazeboHeading(pose_msg_.pose.orientation);

        double prop_x_aligned = radius*cos(angle-((M_PI/2)-heading));
        double prop_y_aligned = radius*sin(angle-((M_PI/2)-heading));


        double prop_x =  pose_msg_.pose.position.y + prop_x_aligned;
        double prop_y =  -pose_msg_.pose.position.x + prop_y_aligned;

        // Create and publish the Prop message with the prop's local coordinates 
        prop_mapper::Prop local_prop_msg;
        local_prop_msg.prop_label = msg->prop_label;
        local_prop_msg.vector.x = prop_x;
        local_prop_msg.vector.y = prop_y;

        ROS_DEBUG_STREAM(TAG << "x robot = " << pose_msg_.pose.position.y);
        ROS_DEBUG_STREAM(TAG << "y robot = " << -pose_msg_.pose.position.x);
        ROS_DEBUG_STREAM(TAG << "x prop = " << prop_x);
        ROS_DEBUG_STREAM(TAG << "y prop = " << prop_y);
        pub_prop_coords_.publish(local_prop_msg);
    }


};

int main(int argc, char** argv) {
    ros::init(argc, argv, "coord_finder_node");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
        ros::console::notifyLoggerLevelsChanged();
    CoordFinder coord_finder;
    coord_finder.spin();
    return 0;
}