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
void fake_bbox_angles() {
    ros::NodeHandle nh("");
    ros::NodeHandle private_nh_("~");

    // Get params
    double theta_small;
    double theta_large; 
    private_nh_.param<double>("theta_small", theta_small, 0.0);
    private_nh_.param<double>("theta_large", theta_large, M_PI);

    ros::Publisher pub = nh.advertise<prop_mapper::PropAngleRange>("prop_angle_range", 1);
    ros::Rate rate(10); // 10 Hz
    prop_mapper::PropAngleRange msg;

    // Message
    msg.prop_label = "red_marker";
    msg.theta_small = theta_small; 
    msg.theta_large = theta_large;

    while (ros::ok()) {
        ROS_DEBUG_STREAM(msg);
        pub.publish(msg);
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "fake_angle_finder");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn))
    ros::console::notifyLoggerLevelsChanged();
    try {
        fake_bbox_angles();
    }
    catch (ros::Exception& e) {
        ROS_ERROR_STREAM("An exception occurred in the fake_angle_finder node: " << e.what());
    }
    return 0;
}