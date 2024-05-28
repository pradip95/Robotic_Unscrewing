#include "control_lib/spiral_interpolation.h"

//constructor of SpiralInterpolation

SpiralInterpolation::SpiralInterpolation() {

}


// Member function for spiral interpolation
std::vector<std::vector<double>> SpiralInterpolation::interpolateSpiral(double angleIncrement, double trajectoryDistance, double radiusSearch, std::vector<double> pose) {
    double phi1 = 0.0;
    double phi2 = 0.0;
    double a = trajectoryDistance / (2 * M_PI);
    std::vector<std::vector<double>> poses;

    while (phi2 <= 2 * 360) {
        double phi = phi2 - phi1;
        double distanceV = sqrt(pow((tan(phi) * cos(phi) * phi1 * a), 2) + pow((a * (phi2 - phi1 * cos(phi))), 2));
            
        if (distanceV > trajectoryDistance) {
            double x1 = a * phi1 * cos(phi1);
            double y1 = a * phi1 * sin(phi1);
            double x2 = a * phi2 * cos(phi2);
            double y2 = a * phi2 * sin(phi2);
            double x = x2 - x1;
            double y = y2 - y1;

            std::vector<double> intermediatePose = pose;
            poses.push_back(intermediatePose);

            pose[0] += x;
            pose[1] += y;

            phi1 = phi2;

            if (radiusSearch < fabs(x2) || radiusSearch< fabs(y2)) {
                break;
            }
        }

        phi2 += angleIncrement;
    }
    std::cout << "Printing Interpolated Points of Spiral Trajectory" << std::endl;
    for (const auto& pose : poses) {
        std::cout << pose[0] << " " << pose[1] << " " << pose[2] << " " << pose[3] << " " << pose[4] << " " << pose[5] <<std::endl;
    };
    return poses;
}


/* Main function
int main() {
    // Create an instance of the SpiralInterpolation class
    SpiralInterpolation spiral;
    
    // Parameters for spiral interpolation
    double angleIncrement = 0.001;
    double trajectoryDistance = 0.0005;
    double radiusSearch = 0.004;

    std::vector<double> initialPose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    // Compute the interpolated poses using the SpiralInterpolation class
    std::vector<std::vector<double>> interpolatedPoses = spiral.interpolateSpiral(angleIncrement, trajectoryDistance, radiusSearch, initialPose);
    
    // Printing the interpolated poses
    for (const auto& pose : interpolatedPoses) {
        std::cout << "Pose: ";
        for (const auto& value : pose) {
            std::cout << value << " ";
        }
        std::cout << std::endl;
    }
    
    return 0;
}*/

