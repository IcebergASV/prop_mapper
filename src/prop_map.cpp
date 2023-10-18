#include <ros/ros.h>
#include <prop_mapper/Prop.h>
#include <prop_mapper/PropArray.h>

class PropMapping {
public:
    PropMapping()
    {
        // get ROS parameters
        private_nh_.param<double>("mapping_safety_radius", safety_radius_, 1.0);
       
        prop_sub_ = nh_.subscribe("/completed_props", 1, &PropMapping::propCallback, this);
        prop_pub_ = nh_.advertise<prop_mapper::PropArray>("/prop_array", 1);
        
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
    ros::Subscriber prop_sub_;
    ros::Publisher prop_pub_;
    prop_mapper::PropArray prop_array;

    std::string TAG = "PROP_MAP: ";          //!< tag for logging and debug messages

    double safety_radius_;

    void propCallback(const prop_mapper::Prop::ConstPtr& msg)
    {
        // get prop info from message - TODO
        prop_mapper::Prop prop;

        //make sure prop is not already in array
        if (isPropInArray(prop) == false){
            // add prop to array if not already there
            prop_array.props.push_back(prop);
            ROS_INFO_STREAM(TAG << "Added a Prop to the map");
        }

        //publish array       
        prop_pub_.publish(prop_array);
    }



    bool isPropInArray(prop_mapper::Prop prop){
        //use safety ranges to decide if prop is already in array
        for (int i = 0; i < prop_array.props.size(); i++) {
            prop_mapper::Prop checkprop = prop_array.props[i];
            
            if (0)// TODO Replace with checking conditions to see if prop falls within the safety range of any existing props
            {
                //prop is already in array, don't add it to the array
                return true;
                ROS_INFO_STREAM(TAG << "Prop is already in the array");
            }

        }
        return false;       

    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "prop_mapping_node");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
        ros::console::notifyLoggerLevelsChanged();
    PropMapping prop_mapper;
    prop_mapper.spin();
    return 0;
}