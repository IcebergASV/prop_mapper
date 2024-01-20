#include <iostream>
#include <vector>
#include <ros.h>

// Default constructor
lidarCalculations::lidarCalculations() : {}

double calculateRadius(const std::vector<lidarPoint>& points, int min_pts) {
    if (points.size() < min_pts) {
        ROS_WARN("At least 6 points are required to calculate the radius of a cylinder.");
    }

    // convert polar coordinates to cartesian coordinates
    std::vector<double> x_coords(points.size());
    std::vector<double> y_coords(points.size());

    for (size_t i = 0; i < points.size(); i++) {
        x_coords[i] = points[i].getDistance() * std::cos(points[i].getAngle());
        y_coords[i] = points[i].getDistance() * std::sin(points[i].getAngle());
    }

    double radius;
    // follow gpt's solution for solving the matrices using Eigen
    // break it down into smaller helper functions

    // look into being behind on version for prop_mapper

    return radius;
}