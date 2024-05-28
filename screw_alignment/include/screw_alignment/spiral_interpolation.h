#ifndef SPIRAL_INTERPOLATION_H
#define SPIRAL_INTERPOLATION_H

#include <cmath>
#include <vector>
#include <array>
#include "spiral_interpolation.h"

class SpiralInterpolation {
public:
    // Member function for spiral interpolation
    std::vector<std::vector<double>> interpolateSpiral(double angleIncrement, double trajectoryDistance, double radiusSearch, std::vector<double> pose);
};

#endif // SPIRAL_INTERPOLATION_H
