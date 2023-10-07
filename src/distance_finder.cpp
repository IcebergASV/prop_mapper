#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <prop_follower/PropAngleRange.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>
#include <vector>
#include <stdexcept>
#include "lidarPoint.h"
#include <string>
#include <iostream>
#include <ros/console.h>

/**
* @brief Finds the relative local coordinates of the prop with the robot's current position as a reference
* 
* Using the angle range and LiDAR scan, picks the closest point within the angle range.
* The props are small enough that this point can be taken to represent the center of the prop. 
* Converts this distance and angle to x and y distance relative to robot's current position
*
*/
class DistanceFinder {
public:
    /**
    @brief Class constructor

    Subscribers: 
     - /prop_angle_range (angle range of identified object)
     - /scan (LiDAR data)

    Publishers:
     - /prop_xy_dist

    */
    DistanceFinder() : nh_(""), private_nh_("~") {
        
        // get ROS parameters
        private_nh_.param<double>("angle_error_adjustment", angle_error_adjustment_, 0.0);

        // Specify ROS topic names - using parameters for this so that we can change names from launch files
        private_nh_.param<std::string>("prop_topic", prop_angles_topic_, "/prop_angle_range");
        private_nh_.param<std::string>("scan_topic", scan_topic_, "/scan");
        
        // set up subscribers
        sub_scan_ = nh_.subscribe(scan_topic_, 1, &CoordFinder::scanCallback, this);
        sub_prop_angles_ = nh_.subscribe(prop_topic_, 1, &CoordFinder::propCallback, this);

        // set up publishers
        pub_prop_xy_dist_ = nh_.advertise<geometry_msgs::Vector3>("/prop_xy_dist", 1);
    }

    void spin() {
        ros::Rate rate(10); // Spin at 10 Hz
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }
    

private:

    ros::NodeHandle nh_;                            //!< node handle
    ros::NodeHandle private_nh_;                    //!< private node handle
    ros::Subscriber sub_scan_;                      //!< subscriber to scan topic
    ros::Subscriber sub_prop_angles_;               //!< subscriber to prop's angle ranges from bounding boxes
    ros::Publisher pub_prop_xy_dist_;               //!< publisher for prop's relative x and y distances
    std::string prop_angles_topic_;                 //!< prop angles topic name
    std::string scan_topic_;                        //!< scan topic name
    
    double laser_angle_min_;                        //!< minimum angle of the LiDAR
    double laser_angle_max_;                        //!< maximum angle of the LiDAR
    double laser_angle_increment_;                  //!< increment between readings within a LiDAR scan
    double angle_error_adjustment_;                 //!< used to expand the angles provided from bounding boxes
    
    prop_follower::PropAngleRange prop_angles_msg_; //!< prop angles message from bounding boxes
    sensor_msgs::LaserScan scan_msg_;               //!< 

    std::string TAG = "DISTANCE_FINDER: ";          //!< tag for logging and debug messages

    //-----------------------------------------------------------------------------------------------------
    // Helper Functions
    //-----------------------------------------------------------------------------------------------------
   
    /**
    @brief Checks if PropAngleRange message is valid
    @returns true if message is valid    
    */
    bool isValidProp(const prop_follower::PropAngleRange::ConstPtr& msg)
    {
        bool valid_msg = true;

        if (msg.prop_label.empty()) {
            ROS_WARN_STREAM(TAG << "Invalid PropInProgress message received - Prop type is empty");
            valid_msg = false;
        }
        if (std::isnan(msg.theta_small)) {
            ROS_WARN_STREAM(TAG << "Invalid PropInProgress message received - theta 1 is empty");
            valid_msg = false;
        }
        if (std::isnan(msg.theta_large)) {
            ROS_WARN_STREAM(TAG << "Invalid PropInProgress message received - theta 2 is empty");
            valid_msg = false;
        }
        return valid_msg;
    }

    // ----------------------------------------------------------------------------------------------------
    // Callback functions
    // ----------------------------------------------------------------------------------------------------
    /**
    * @brief Gets the prop label and angles
    * 
    * Runs whenever a message is published on /prop_angle_range 
    */
    void propCallback(const prop_follower::PropAngleRange::ConstPtr& msg) {
        // save the PropAngleRange message for later use
        prop_angles_msg_ = *msg;
        ROS_DEBUG_STREAM(TAG << "Received PropInProgress message with theta_small=" << prop_angles_msg_.theta_small << " and theta_large=" << prop_angles_msg_.theta_large);
    }

    /**
    * @brief Finds the nearest detected point within the angle range and converts it to x and y coordinates. 
    * 
    * Runs whenever message is published on /scan topic. 
    * Publishes a Vector3 message containing NED coordinates of the detected prop. (North East Down)
    * 
    * */
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        
        // save the scan message
        scan_msg_ = *msg;

        // get LiDAR specs
        laser_angle_min_ = scan_msg_.angle_min;
        laser_angle_max_ = scan_msg_.angle_max;
        laser_angle_increment_ = scan_msg_.angle_increment;

        // display warning and exit function if PropAngleRange message is invalid
        if(!isValidProp(prop_angles_msg_)) {return;}

        //add a safety range onto the bounding box angles
        double index1_angle = prop_angles_msg_.theta_small + angle_error_adjustment_;
        double index2_angle = prop_angles_msg_.theta_large - angle_error_adjustment_;

        // calculate the range indexes for the given theta angles
        double steps = (laser_angle_max_ * 2) / laser_angle_increment_; 
        int index1 = (int)(((index1_angle + (laser_angle_max_ - (M_PI/2))) / (laser_angle_max_*2))* steps);
        int index2 = (int)(((index2_angle + (laser_angle_max_ - (M_PI/2))) / (laser_angle_max_*2))* steps);
        ROS_DEBUG_STREAM(TAG << "index1 :" << index1 << " index2: " << index2);
        ROS_DEBUG_STREAM(TAG << "size of scan message ranges " << scan_msg_.ranges.size());

        // check that the range indexes are within the range of the scan message and that index1 > index2
        if (index1 < 0 || index2 < 0 || index1 >= scan_msg_.ranges.size() || index2 >= scan_msg_.ranges.size() || index1 >= index2) {
            ROS_WARN_STREAM(TAG << "PropInProgress message range indexes are out of bounds for the given scan message");
            return;
        }

        //create a 2D vector containing distance angle pairs for points detected by lidar      
        ROS_DEBUG_STREAM(TAG << "Laser angle min" << laser_angle_min_);
        ROS_DEBUG_STREAM(TAG << "Laser angle increment" << laser_angle_increment_);
        double starting_angle = laser_angle_min_ + (M_PI/2.0); //starting angle for lidar scan 
        std::vector<lidarPoint> scanPoints = CoordFinder::createLidarPoints(scan_msg_.ranges, starting_angle, laser_angle_increment_);
        if (scanPoints.size()<1){
            ROS_WARN_STREAM(TAG << "No points added to scanPoints vector");
            return;
        }

        //create a smaller vector of only points within the camera provided range
        std::vector<lidarPoint> selectedPoints;
        for (int i = index1; i <= index2; i++) {

            selectedPoints.push_back(scanPoints[i]);
            ROS_DEBUG_STREAM(TAG << "Pushing back points within camera range: " << scanPoints[i]);
        }
        if (selectedPoints.size()<1){
            ROS_WARN_STREAM(TAG << "No points added to vector containing points within camera range ");
            return;
        }

        // find the distance from the center of closest point and angle within the given range

        int i = 0;
        double closest_distance = selectedPoints[i].getDistance();
        double closest_angle = selectedPoints[i].getAngle();
        for (int i = 0; i < selectedPoints.size(); ++i) {
            if (std::isnan(selectedPoints[i].getDistance())) {
                continue; 
            }
            if(selectedPoints[i].getDistance() < closest_distance){
                closest_distance = selectedPoints[i].getDistance(); 
                closest_angle = selectedPoints[i].getAngle();       
            }
        }

        ROS_DEBUG_STREAM(TAG << "closest_distance " << closest_distance);
        ROS_DEBUG_STREAM(TAG << "closest angle " << closest_angle);

        // Message to publish
        geometry_msgs::Vector3 prop_coords_msg;
        prop_coords_msg.x = closest_distance*sin(closest_angle); //North
        prop_coords_msg.y = closest_distance*cos(closest_angle); //East 
        prop_coords_msg.z = 0; //Down
        pub_prop_xy_dist_.publish(prop_coords_msg);
    }


    /**
    * @brief Creates a vector of LidarPoints
    * 
    * @param[in] distances detected by Lidar
    * @param[in] start_angle - the angle to start at
    * @param[in] angle_increment - the amoung to increment the angle for each distance
    * @returns the lidarPoints vector
    */
    std::vector<lidarPoint> createLidarPoints(const std::vector<float>& distances, double start_angle , double angle_increment) {
        std::vector<lidarPoint> lidarPoints;
        ROS_DEBUG_STREAM(TAG << "start angle: " << start_angle);
        // Add the first Lidar point
        lidarPoint firstPoint(distances[0], start_angle);
        lidarPoints.push_back(firstPoint);

        // Add the remaining Lidar points
        double currentAngle = start_angle + angle_increment;
        for (size_t i = 1; i < distances.size(); i++) {
            double distance = distances[i];
            lidarPoint point(distance, currentAngle);
            lidarPoints.push_back(point);

            currentAngle += angle_increment;
        }

    return lidarPoints;
    }
  

};
int main(int argc, char** argv) {
    ros::init(argc, argv, "coord_finder");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
        ros::console::notifyLoggerLevelsChanged();
    CoordFinder coord_finder;

    coord_finder.spin();
    return 0;
}