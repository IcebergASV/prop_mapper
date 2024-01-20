#ifndef LIDARCALCULATIONS_H
#define LIDARCALCULATIONS_H

class lidarCalculations {
public:
    // Constructors
    lidarCalculations();

    // Functions
    double calculateRadius(const std::vector<lidarPoint>& points, int min_pts);

private:
}


#endif // LIDARCALCULATIONS_H