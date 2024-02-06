#include <ros/ros.h>
#include <yolov5_ros_msgs/BoundingBoxes.h>
#include <yolov5_ros_msgs/BoundingBox.h>
#include <prop_mapper/PropAngleRange.h>
#include <vector>
#include <cmath>
#include <ros/console.h>


class AngleFinder {
public:
    AngleFinder() : nh_(""), private_nh_("~")
    {
        // Get params
        private_nh_.param<double>("realsense_fov", realsense_fov_p, 0.0);
        private_nh_.param<int>("realsense_res_x", realsense_res_x_p, 0.0);
        private_nh_.param<double>("min_detection_probability", min_detection_probability_p, 0.0);
  
        // publishers and subscribers
        yolo_sub_ = nh_.subscribe("/yolov5/BoundingBoxes", 1, &AngleFinder::yoloCallback, this);
        prop_pub_ = nh_.advertise<prop_mapper::PropAngleRange>("/prop_angle_range", 1);
    }

    void spin() {
        ros::Rate rate(10); // 10 Hz
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:

    void yoloCallback(const yolov5_ros_msgs::BoundingBoxes::ConstPtr& msg)
    {
        std::vector<yolov5_ros_msgs::BoundingBox> boundingBoxes;
        boundingBoxes.assign(msg->bounding_boxes.begin(), msg->bounding_boxes.end());

        for (yolov5_ros_msgs::BoundingBox box : boundingBoxes)
        {
            if (box.probability >= min_detection_probability_p)
            {   
                ROS_DEBUG_STREAM(TAG << "Prop: " << box.Class << ", found with acceptable probability: " << box.probability );
                x_min_ = box.xmin;
                x_max_ = box.xmax;

                // Calculate the angle range for the prop
                double theta_right = fov_end_ - ((x_max_ / realsense_res_x_p) * realsense_fov_p); 
                double theta_left = fov_end_ - ((x_min_ / realsense_res_x_p) * realsense_fov_p);

                // Create and publish the Prop message with the prop coordinates
                prop_mapper::PropAngleRange prop_msg;
                prop_msg.prop_label = box.Class; //assign object classification label to the prop
                prop_msg.theta_small = theta_right;
                prop_msg.theta_large = theta_left;
                prop_pub_.publish(prop_msg);
            }
            else 
                ROS_DEBUG_STREAM(TAG << "Prop: " << box.Class << ", probability to low: " << box.probability );
        }
    }
    

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;  
    ros::Subscriber yolo_sub_;
    ros::Publisher prop_pub_;
    double x_min_;
    double x_max_;

    double min_detection_probability_p;
    double realsense_fov_p; //radians - 69 degrees
    double fov_end_ = (M_PI / 2) + (realsense_fov_p / 2 );
    int realsense_res_x_p;
    std::string TAG = "ANGLE_FINDER: ";  

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "angle_finder");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
        ros::console::notifyLoggerLevelsChanged();

    AngleFinder angle_finder;
    angle_finder.spin();
    return 0;
}