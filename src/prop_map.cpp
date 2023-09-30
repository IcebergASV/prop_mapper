#include <ros/ros.h>
#include <navigation_pkg/Prop.h>
#include <navigation_pkg/PropArray.h>

class PropMapping {
public:
    PropMapping()
    {
        prop_sub_ = nh_.subscribe("/completed_props", 1, &PropMapping::propCallback, this);
        prop_pub_ = nh_.advertise<navigation_pkg::PropArray>("/prop_array", 1);
        
    }

    void spin() {
        ros::Rate rate(10); // 10 Hz
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:

    void propCallback(const navigation_pkg::Prop::ConstPtr& msg)
    {
        // get prop info
        navigation_pkg::Prop prop;
        prop.prop_type = msg->prop_type;
        prop.prop_coords.latitude = msg->prop_coords.latitude;
        prop.prop_coords.longitude = msg->prop_coords.longitude;
        prop.prop_coords.altitude = msg->prop_coords.altitude;
        prop.prop_coord_range.min_latitude = msg->prop_coord_range.min_latitude;
        prop.prop_coord_range.max_latitude = msg->prop_coord_range.max_latitude;
        prop.prop_coord_range.min_longitude = msg->prop_coord_range.min_longitude;
        prop.prop_coord_range.max_longitude = msg->prop_coord_range.max_longitude;
        prop.prop_coord_range.min_altitude = msg->prop_coord_range.min_altitude;
        prop.prop_coord_range.max_altitude = msg->prop_coord_range.max_altitude;



        //make sure prop is not already in array
            
        if (isPropInArray(prop) == false){
            // add prop to array if not already there
            prop_array.props.push_back(prop);
            ROS_INFO_STREAM("Prop is not in the array, adding it");
        }


        //publish array       
        prop_pub_.publish(prop_array);
    }

    ros::NodeHandle nh_;
    ros::Subscriber prop_sub_;
    ros::Publisher prop_pub_;
    navigation_pkg::PropArray prop_array;

    bool isPropInArray(navigation_pkg::Prop prop){
        //use safety ranges to decide if prop is already in array
        for (int i = 0; i < prop_array.props.size(); i++) {
            navigation_pkg::Prop checkprop = prop_array.props[i];
            
            if ( prop.prop_coords.latitude <= checkprop.prop_coord_range.max_latitude && prop.prop_coords.latitude >= checkprop.prop_coord_range.min_latitude
            && prop.prop_coords.longitude <= checkprop.prop_coord_range.max_longitude && prop.prop_coords.longitude >= checkprop.prop_coord_range.min_longitude) {
                //prop is already in array, don't add it to the array
                return true;
                ROS_INFO_STREAM("Prop is already in the array");
            }

            
        }
        return false;       

    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "prop_mapping_node");
    PropMapping prop_mapper;
    prop_mapper.spin();
    return 0;
}