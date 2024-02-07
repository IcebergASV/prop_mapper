#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <prop_mapper/PropAngleRange.h>
#include <prop_mapper/PropPolarCoords.h>
#include <cmath>
#include <vector>
#include <stdexcept>
#include "lidar_point.h"
#include <string>
#include <iostream>
#include <ros/console.h>
#include "lidar_calculations.h"
#include <algorithm>

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
        private_nh_.param<double>("angle_error_adjustment", angle_error_adjustment_p, 0.0);
        private_nh_.param<double>("max_lidar_range", max_lidar_range_p, 29.0);
        private_nh_.param<double>("lidar_position", lidar_position_p, 1.6);
        private_nh_.param<int>("min_circle_pts", min_circle_pts_p, 6);
        private_nh_.param<double>("marker_radius", marker_radius_p, 0.127);
        private_nh_.param<double>("small_buoy_radius", buoy_radius_sm_p, 0.1015);
        private_nh_.param<double>("large_buoy_radius", buoy_radius_lg_p, 0.184);
        private_nh_.param<double>("marker_radius_range", marker_radius_range_p, 0.05);
        private_nh_.param<double>("small_buoy_radius_range", buoy_sm_radius_range_p, 0.075);
        private_nh_.param<double>("large_buoy_radius_range", buoy_lg_radius_range_p, 0.1);
        private_nh_.param<double>("lidar_point_range", lidar_point_range_p, 0.3);
        private_nh_.param<bool>("check_small_buoys", check_small_buoys_p, true);
        private_nh_.getParam("valid_prop_labels", valid_prop_labels_p);

        // Specify ROS topic names - using parameters for this so that we can change names from launch files
        private_nh_.param<std::string>("prop_topic", prop_angles_topic_, "/prop_angle_range");
        private_nh_.param<std::string>("scan_topic", scan_topic_, "/scan");
        
        // set up subscribers
        sub_scan_ = nh_.subscribe(scan_topic_, 1, &DistanceFinder::scanCallback, this);
        sub_prop_angles_ = nh_.subscribe(prop_angles_topic_, 1, &DistanceFinder::propCallback, this);

        // set up publishers
        pub_prop_polar_coords_ = nh_.advertise<prop_mapper::PropPolarCoords>("/prop_polar_coords", 1);
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
    ros::Publisher pub_prop_polar_coords_;               //!< publisher for prop's relative x and y distances
    std::string prop_angles_topic_;                 //!< prop angles topic name
    std::string scan_topic_;                        //!< scan topic name
    
    double laser_angle_min_;                        //!< minimum angle of the LiDAR
    double laser_angle_max_;                        //!< maximum angle of the LiDAR
    double laser_angle_increment_;                  //!< increment between readings within a LiDAR scan
    double angle_error_adjustment_p;                 //!< used to expand the angles provided from bounding boxes
    double max_lidar_range_p;
    double lidar_position_p;
    int min_circle_pts_p;
    double marker_radius_p;
    double buoy_radius_sm_p;
    double buoy_radius_lg_p;
    double marker_radius_range_p;
    double buoy_sm_radius_range_p;
    double buoy_lg_radius_range_p;
    double lidar_point_range_p;
    bool check_small_buoys_p;
    std::vector<std::string> valid_prop_labels_p;
    
    prop_mapper::PropAngleRange prop_angles_msg_; //!< prop angles message from bounding boxes
    sensor_msgs::LaserScan scan_msg_;              

    std::string TAG = "DISTANCE_FINDER: ";          //!< tag for logging and debug messages

    //-----------------------------------------------------------------------------------------------------
    // Helper Functions
    //-----------------------------------------------------------------------------------------------------
   
    /**
    @brief Checks if PropAngleRange message is valid

    @param[in] msg PropAngleRange message
    @returns true if message is valid    
    */
    bool isValidProp(const prop_mapper::PropAngleRange msg)
    {
        bool valid_msg = true;

        if (!isLabelValid(msg)) { // if label not in valid labels
            ROS_WARN_STREAM(TAG << "Invalid Prop message received - Prop type is invalid");
            valid_msg = false;
        }
        if (std::isnan(msg.theta_small)) {
            ROS_WARN_STREAM(TAG << "Invalid Prop message received - theta 1 is empty");
            valid_msg = false;
        }
        if (std::isnan(msg.theta_large)) {
            ROS_WARN_STREAM(TAG << "Invalid Prop message received - theta 2 is empty");
            valid_msg = false;
        }
        return valid_msg;
    }

    bool isLabelValid(const prop_mapper::PropAngleRange prop) {
        for (int i = 0; i < valid_prop_labels_p.size(); i++) {
            if (valid_prop_labels_p[i] == prop.prop_label) {
                return true;
            }
        }
        return false;
    }

    bool validateRadius(std::string &prop_label, double radius) {
        if ((prop_label == "red_marker" || prop_label == "green_marker") && (radius>marker_radius_p-marker_radius_range_p && radius<marker_radius_p+marker_radius_range_p)) {
            return true;
        }
        if ((prop_label == "blue_buoy") && (radius>buoy_radius_lg_p-buoy_lg_radius_range_p && radius<buoy_radius_lg_p+buoy_lg_radius_range_p)) {
            return true;
        }
        if ((prop_label == "black_buoy") && (radius>buoy_radius_sm_p-buoy_sm_radius_range_p && radius<buoy_radius_sm_p+buoy_sm_radius_range_p)) {
            return true;
        }
        if ((prop_label == "red_buoy" || prop_label == "green_buoy" || prop_label == "yellow_buoy")) {
            if(!check_small_buoys_p && ((radius>(((buoy_radius_sm_p+buoy_radius_lg_p)/2)-buoy_lg_radius_range_p)) && (radius<(((buoy_radius_sm_p+buoy_radius_lg_p)/2)+buoy_lg_radius_range_p)))) {
                return true;
            }
            if (radius>buoy_radius_lg_p-buoy_lg_radius_range_p && radius<buoy_radius_lg_p+buoy_lg_radius_range_p) {
                prop_label.append("_lg");
                return true;
            }
            if (radius>buoy_radius_sm_p-buoy_sm_radius_range_p && radius<buoy_radius_sm_p+buoy_sm_radius_range_p) {
                prop_label.append("_sm");
                return true;
            }
        }
        else { // prop did not match an expected radius
            return false;
        }
    }

    /**
    @brief find the indexes for scan data that will get data within the angle range

    @param[out] index1 
    @param[out] index2 
    @returns true if indexes are valid   
    */
    bool getScanIndexes(int &index1, int &index2, double index1_angle, double index2_angle, double laser_angle_max, double laser_angle_increment, const sensor_msgs::LaserScan &scan_msg)
    {
        double steps = (laser_angle_max * 2) / laser_angle_increment; 
        index1 = (int)(((index1_angle + (laser_angle_max - (M_PI/2))) / (laser_angle_max*2))* steps);
        index2 = (int)(((index2_angle + (laser_angle_max - (M_PI/2))) / (laser_angle_max*2))* steps);
        ROS_DEBUG_STREAM(TAG << "index1 :" << index1 << " index2: " << index2);
        ROS_DEBUG_STREAM(TAG << "size of scan message ranges " << scan_msg.ranges.size());

        // check that the range indexes are within the range of the scan message and that index1 > index2
        if (index1 < 0 || index2 < 0 || index1 >= scan_msg.ranges.size() || index2 >= scan_msg.ranges.size() || index1 >= index2) {
            ROS_WARN_STREAM(TAG << "PropInProgress message range indexes are out of bounds for the given scan message");
            return false;
        }

        return true;
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

    /**
    @brief get the relative distance and angle of the closest detected object from array of distances
    
    @param[in] distance_array
    @param[out] closest_distance
    @param[out] closest_angle

    */
    void getClosestObject(const std::vector<lidarPoint> &distances_array, double &closest_distance, double &closest_angle)
    {
        int i = 0;
        closest_distance = distances_array[i].getDistance();
        closest_angle = distances_array[i].getAngle();
        for (int i = 0; i < distances_array.size(); ++i) {
            if (std::isnan(distances_array[i].getDistance())) {
                continue; 
            }
            if(distances_array[i].getDistance() < closest_distance){
                closest_distance = distances_array[i].getDistance(); 
                closest_angle = distances_array[i].getAngle();       
            }
        }

        ROS_DEBUG_STREAM(TAG << "closest_distance " << closest_distance);
        ROS_DEBUG_STREAM(TAG << "closest angle " << closest_angle);

        return;
    }

    // ----------------------------------------------------------------------------------------------------
    // Callback functions
    // ----------------------------------------------------------------------------------------------------

    /**
    * @brief Gets the prop label and angles
    * 
    * Runs whenever a message is published on /prop_angle_range 
    */
    void propCallback(const prop_mapper::PropAngleRange::ConstPtr& msg) {
        // save the PropAngleRange message for later use
        prop_angles_msg_ = *msg;
        ROS_DEBUG_STREAM(TAG << "Prop: " << prop_angles_msg_.prop_label);
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
        ROS_DEBUG_STREAM(TAG << "Laser angle min" << laser_angle_min_);
        ROS_DEBUG_STREAM(TAG << "Laser angle increment" << laser_angle_increment_);

        // display warning and exit function if PropAngleRange message is invalid
        if(!isValidProp(prop_angles_msg_))
        {
            return;
        }

        //add a safety range onto the bounding box angles
        double index1_angle = prop_angles_msg_.theta_small + angle_error_adjustment_p;
        double index2_angle = prop_angles_msg_.theta_large - angle_error_adjustment_p;

        // calculate the range indexes for the given theta angles
        int index1;
        int index2;
        getScanIndexes(index1, index2, index1_angle, index2_angle, laser_angle_max_, laser_angle_increment_, scan_msg_);


        //create a 2D vector containing distance angle pairs for points detected by lidar      
        double starting_angle = laser_angle_min_ + (M_PI/2.0); //starting angle for lidar scan 
        std::vector<lidarPoint> scanPoints = DistanceFinder::createLidarPoints(scan_msg_.ranges, starting_angle, laser_angle_increment_);
        if (scanPoints.size()<1){
            ROS_WARN_STREAM(TAG << "No points added to scanPoints vector");
            return;
        }

        //create a smaller vector of only points within the camera provided range
        std::vector<lidarPoint> selected_points;
        for (int i = index1; i <= index2; i++) {
            if (scanPoints[i].getDistance() < max_lidar_range_p) {
                selected_points.push_back(scanPoints[i]);
            }
        }
        if (selected_points.size()<1){
            ROS_WARN_STREAM(TAG << "No points added to vector containing points within camera range ");
            return;
        }
        
        // find the distance from the center of closest point and angle within the given range 
        double closest_distance;
        double closest_angle;
        getClosestObject(selected_points, closest_distance, closest_angle);

        // get all points on the circle, points that fall within a distance of each other
        std::vector<lidarPoint> circle_points;
        circle_points.push_back(lidarPoint(closest_distance, closest_angle));
        for (int i = 0; i < selected_points.size(); i++) {
            double i_dist = selected_points[i].getDistance();
            double i_angle = selected_points[i].getAngle();
            double dist = sqrt(pow(i_dist, 2) + pow(closest_distance, 2) - 2*i_dist*closest_distance*cos(closest_angle - i_angle));
            if (dist < lidar_point_range_p) {
                circle_points.push_back(selected_points[i]);
            }
        }
        if (circle_points.size()<min_circle_pts_p) {
            ROS_WARN_STREAM(TAG << "Expected at least " << min_circle_pts_p << " points for radius calculation but got " << circle_points.size());
            return;
        }

        // do radius checking, and confirm that the prop found has the expected radius
        lidarCalculations lidarCalc;
        double radius = lidarCalc.calculateRadius(circle_points, min_circle_pts_p);
        ROS_DEBUG_STREAM(TAG << "r = " << radius << ", close pt = " << closest_distance);

        if (!validateRadius(prop_angles_msg_.prop_label, radius)) {
            ROS_WARN_STREAM(TAG << "Calculated radius does not match expected " << prop_angles_msg_.prop_label << " radius");
            return;
        }

        // Publish Message

        if (closest_distance < max_lidar_range_p)
        {
            prop_mapper::PropPolarCoords prop_rel_coords_msg;
            prop_rel_coords_msg.prop_label = prop_angles_msg_.prop_label;
            prop_rel_coords_msg.radius = radius + closest_distance; 
            prop_rel_coords_msg.angle = closest_angle; 
            pub_prop_polar_coords_.publish(prop_rel_coords_msg);
        }
    }

};
int main(int argc, char** argv) {
    ros::init(argc, argv, "distance_finder");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
        ros::console::notifyLoggerLevelsChanged();
    DistanceFinder distance_finder;

    distance_finder.spin();
    return 0;
}