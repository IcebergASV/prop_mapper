#include <iostream>
#include <vector>
#include <ros/ros.h>
#include "lidar_calculations.h"
#include "lidar_point.h"
#include <eigen3/Eigen/Dense>

lidarCalculations::lidarCalculations() {}

double lidarCalculations::calculateRadius(const std::vector<lidarPoint>& points, int min_pts) const {
    if (points.size() < min_pts) {
        ROS_WARN_STREAM("At least " << min_pts << " points are required to calculate the radius of a cylinder.");
    }

    // convert polar coordinates to cartesian coordinates
    std::vector<double> x_coords(points.size());
    std::vector<double> y_coords(points.size());

    for (size_t i = 0; i < points.size(); i++) {
        x_coords[i] = points[i].getDistance() * std::cos(points[i].getAngle());
        y_coords[i] = points[i].getDistance() * std::sin(points[i].getAngle());
    }

    double radius;
    
    // calculate matrix A
    Eigen::Matrix<double, 3, 3> matrixA;
    double mA_11, mA_12, mA_13, mA_21, mA_22, mA_23, mA_31, mA_32, mA_33 = 0;
    for (size_t i = 0; i < points.size(); i++) {
        mA_11 = mA_11 + pow(x_coords[i], 2);
        mA_12 = mA_12 + (x_coords[i]*y_coords[i]);
        mA_13 = mA_13 + x_coords[i];
        mA_22 = mA_22 + pow(y_coords[i], 2);
        mA_23 = mA_23 + y_coords[i];
    }
    mA_21 = mA_12;
    mA_31 = mA_13;
    mA_32 = mA_23;
    mA_33 = points.size();

    matrixA << mA_11, mA_12, mA_13, mA_21, mA_22, mA_23, mA_31, mA_32, mA_33;

    // calculate vector X
    Eigen::Matrix<double, 3, 1> vectorX;
    double vX_1, vX_2, vX_3 = 0;
    for (size_t i = 0; i < points.size(); i++) {
        vX_1 = vX_1 + x_coords[i]*(pow(x_coords[i], 2) + pow(y_coords[i], 2));
        vX_2 = vX_2 + y_coords[i]*(pow(x_coords[i], 2) + pow(y_coords[i], 2));
        vX_3 = vX_3 + (pow(x_coords[i], 2) + pow(y_coords[i], 2));
    }

    vectorX << vX_1, vX_2, vX_3;

    Eigen::Matrix<double, 3, 1> result = matrixA.colPivHouseholderQr().solve(vectorX.cast<double>());

    ROS_DEBUG_STREAM("A: " << result[0]);
    ROS_DEBUG_STREAM("B: " << result[1]);
    ROS_DEBUG_STREAM("C: " << result[2]);

    radius = (sqrt(4*result[2] + pow(result[0], 2) + pow(result[1], 2))) / 2;

    return radius;
}