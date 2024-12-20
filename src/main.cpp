#include <iostream>
#include "angular_velocity_interpolator.h"

const double QC_DIAG = 1e6;

int main(int argc, char** argv)
{
    std::vector<double> measurementStamps = {0.0, 1.0, 2.0, 3.0};
    std::vector<double> velocityStamps = {0.0, 1.0, 3.0};
    std::vector<Eigen::Vector3d> angularVelocities = {Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 1)};
    std::vector<Eigen::Matrix3d> informationMatrices = {Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity()};
    auto result = interpolateAngularVelocities(measurementStamps, velocityStamps, angularVelocities, informationMatrices, QC_DIAG);
    std::cout << result.first.size() << std::endl;
    std::cout << result.second.size() << std::endl;
    return 0;
}