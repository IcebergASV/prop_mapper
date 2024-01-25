#include <ros/ros.h>
#include <yolov5_ros_msgs/BoundingBoxes.h>
#include <yolov5_ros_msgs/BoundingBox.h>
#include <prop_mapper/PropAngleRange.h>
#include <vector>
#include <cmath>


class AngleFinder {
public:
    AngleFinder()
    {
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
            x_min = box.xmin;
            x_max = box.xmax;

            // Calculate the angle range for the prop
            double theta_right = fov_end - ((x_max / realsense_res_x) * realsense_fov); 
            double theta_left = fov_end - ((x_min / realsense_res_x) * realsense_fov);

            // Create and publish the Prop message with the prop coordinates
            prop_mapper::PropAngleRange prop_msg;
            prop_msg.prop_label = box.Class; //assign object classification label to the prop
            prop_msg.theta_small = theta_right;
            prop_msg.theta_large = theta_left;
            prop_pub_.publish(prop_msg);
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber yolo_sub_;
    ros::Publisher prop_pub_;
    double x_min;
    double x_max;
    double const realsense_fov = 1.204277184; //radians - 69 degrees
    double const fov_end = (M_PI / 2) + (realsense_fov / 2 );
    int const realsense_res_x = 1920;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "angle_finder");
    AngleFinder angle_finder;
    angle_finder.spin();
    return 0;
}