#include <ros/ros.h>
#include <lidar_calculations.h>
#include <lidar_point.h>

class lidarCalcTest {
public:
    lidarCalcTest(): nh_(""), private_nh_("~")
    { 
        
    }

    void spin() {
        ros::Rate rate(10);
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

    double calcRadius() {
        std::vector<lidarPoint> points;

        // POINT SET 1
        lidarPoint point1 = {1, 1.5708};
        lidarPoint point2 = {5.385, 1.1903};
        lidarPoint point3 = {6.083, 1.4056};
        lidarPoint point4 = {9.22, 0.7086};

        // POINT SET 2 (example B from website)
        // lidarPoint point1 = {38.63, 0.3709};
        // lidarPoint point2 = {37.36, 0.27095};
        // lidarPoint point3 = {33.84, 0.9746};
        // lidarPoint point4 = {35.85, 1.0447};
        // lidarPoint point5 = {37.59, 0.4993};
        // lidarPoint point6 = {36.77, 0.7854};

        points.push_back(point1);
        points.push_back(point2);
        points.push_back(point3);
        points.push_back(point4);

        // points.push_back(point5);
        // points.push_back(point6);

        lidarCalculations lidarCalc;

        double rad = lidarCalc.calculateRadius(points, 1);
        return rad;
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

};


int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_calc_test");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
        ros::console::notifyLoggerLevelsChanged();

    lidarCalcTest lidarCalcTest;

    double rad = lidarCalcTest.calcRadius();
    ROS_INFO_STREAM("Radius = " << rad);

    lidarCalcTest.spin();

    return 0;
}