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
        private_nh_.param<bool>("filter_out_farther_props", filter_out_farther_props_p, false);
  
        // publishers and subscribers
        yolo_sub_ = nh_.subscribe("/yolov5/BoundingBoxes", 1, &AngleFinder::yoloCallback, this);
        prop_pub_ = nh_.advertise<prop_mapper::PropAngleRange>("/prop_angle_range", 1);

        fov_end_ = (M_PI / 2) + (realsense_fov_p / 2 );
    }

    void spin() {
        ros::Rate rate(10); // 10 Hz
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:

    // Only keeps the closest prop of each type 
    // Uses box widths to determine proximity
    // Ex - if there are 2 red markers - it will remove the farther one
    void onlyKeepClosestProps(std::vector<yolov5_ros_msgs::BoundingBox>& boxes)
    {
        std::vector<yolov5_ros_msgs::BoundingBox> closestBoundingBoxes;

        for (int i = 0; i < boxes.size(); i++)
        {
            bool already_exists = false;
            int box_width = boxes[i].xmax - boxes[i].xmin;

            for (int j = 0; j < closestBoundingBoxes.size(); j++)
            {
                if (boxes[i].Class == closestBoundingBoxes[j].Class)
                {
                    ROS_DEBUG_STREAM(TAG << "box type: " << boxes[i].Class<< " already in frame");
                    already_exists = true;
                    int closest_box_width = closestBoundingBoxes[j].xmax - closestBoundingBoxes[j].xmin;
                    if ( box_width > closest_box_width ) // replace a prop of same type if larger box width
                    {
                        ROS_DEBUG_STREAM(TAG << "current box: " << box_width <<  " larger than existing box: " << closest_box_width << " - replacing");
                        closestBoundingBoxes.erase(closestBoundingBoxes.begin() + j);
                        closestBoundingBoxes.push_back(boxes[i]);
                    }
                }
            }

            if (!already_exists) // if no props of the same type already exist - add it
            {
                ROS_DEBUG_STREAM(TAG << "Adding " << boxes[i].Class << " with width: " << box_width);
                closestBoundingBoxes.push_back(boxes[i]);
            }
        }

        ROS_DEBUG_STREAM(TAG << "closestBoundingBoxes: ");
        for (int j = 0; j < closestBoundingBoxes.size(); j++)
        {
            ROS_DEBUG_STREAM(TAG << closestBoundingBoxes[j].Class);
        }

        boxes = closestBoundingBoxes;      

        return;
    }

    void yoloCallback(const yolov5_ros_msgs::BoundingBoxes::ConstPtr& msg)
    {
        ROS_DEBUG_STREAM(TAG << "yoloCallback");
        std::vector<yolov5_ros_msgs::BoundingBox> boundingBoxes;
        boundingBoxes.assign(msg->bounding_boxes.begin(), msg->bounding_boxes.end());

        if (filter_out_farther_props_p)
        {
            onlyKeepClosestProps(boundingBoxes);
        }
 
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

                ROS_DEBUG_STREAM(TAG << "theta_right" << theta_right);
                ROS_DEBUG_STREAM(TAG << "theta_left" << theta_left);

                // Create and publish the Prop message with the prop coordinates
                prop_mapper::PropAngleRange prop_msg;
                prop_msg.prop_label = box.Class; //assign object classification label to the prop
                prop_msg.theta_small = theta_right;
                prop_msg.theta_large = theta_left;
                prop_pub_.publish(prop_msg);
            }
            else 
                ROS_DEBUG_STREAM(TAG << "Prop: " << box.Class << ", probability too low: " << box.probability );
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
    double fov_end_;
    int realsense_res_x_p;
    bool filter_out_farther_props_p;
    std::string TAG = "ANGLE_FINDER: ";  

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "angle_finder");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
        ros::console::notifyLoggerLevelsChanged();

    AngleFinder angle_finder;
    angle_finder.spin();
    return 0;
}