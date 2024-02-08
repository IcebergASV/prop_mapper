#include <ros/ros.h>
#include <prop_mapper/PropAngleRange.h>
#include <cmath>
#include <ros/console.h>

/**
 * @brief Node to publish fake angle ranges for testing without camera
 *
 * Publishes a prop labelled buoy with an angle range of 0 to 180 degrees.
 *
 * Set object_detection to false to run this node. 
 */
void mock_marker_angles() {
    ros::NodeHandle nh("");
    ros::NodeHandle private_nh_("~");

    ros::Publisher pub = nh.advertise<prop_mapper::PropAngleRange>("prop_angle_range", 1);
    ros::Rate rate(10); // 10 Hz
    prop_mapper::PropAngleRange msg;

    while (ros::ok()) 
    {
        msg.prop_label = "green_buoy";
        msg.theta_small = 0.0; 
        msg.theta_large = M_PI * 1/3;
        pub.publish(msg);

        rate.sleep();
        msg.prop_label = "yellow_buoy";
        msg.theta_small = M_PI * 1/3;
        msg.theta_large = M_PI * 2/3;
        pub.publish(msg);
        
        rate.sleep(); 

        msg.prop_label = "red_buoy";
        msg.theta_small = M_PI * 2/3;
        msg.theta_large = M_PI;
        pub.publish(msg);

        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "fake_angle_finder");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn))
    ros::console::notifyLoggerLevelsChanged();
    try {
        mock_marker_angles();
    }
    catch (ros::Exception& e) {
        ROS_ERROR_STREAM("An exception occurred in the fake_angle_finder node: " << e.what());
    }
    return 0;
}