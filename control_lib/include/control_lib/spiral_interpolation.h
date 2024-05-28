#include <iostream>
#include <cmath>
#include <vector>
#include <array>

class SpiralInterpolation {
private: 
    // arguments in interpolateSpiral function
    double angleIncrement, trajectoryDistance, radiusSearch;
    std::vector<double> pose;

public:
    public:
    SpiralInterpolation();

    std::vector<std::vector<double>> interpolateSpiral(double angleIncrement, double trajectoryDistance, double radiusSearch, std::vector<double> pose);
};

