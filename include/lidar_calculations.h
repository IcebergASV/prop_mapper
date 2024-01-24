#ifndef LIDARCALCULATIONS_H
#define LIDARCALCULATIONS_H

#include "lidar_point.h"
#include <vector>
#include <eigen3/Eigen/Dense>

class lidarCalculations {
public:
    // Constructors
    lidarCalculations();

    // Functions
    double calculateRadius(const std::vector<lidarPoint>& points, int min_pts) const;

private:
};


#endif // LIDARCALCULATIONS_H