#ifndef ANGULAR_VELOCITY_INTERPOLATOR_ANGULAR_VELOCITY_INTERPOLATOR_H
#define ANGULAR_VELOCITY_INTERPOLATOR_ANGULAR_VELOCITY_INTERPOLATOR_H

#include <Eigen/Core>

std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Matrix3d>>
interpolateAngularVelocities(const std::vector<double>& measurementStamps, const std::vector<double>& velocityStamps, const std::vector<Eigen::Vector3d>& angularVelocities,
                             const std::vector<Eigen::Matrix<double, 3, 3>>& informationMatrices, const double& qcDiag);

#endif //ANGULAR_VELOCITY_INTERPOLATOR_ANGULAR_VELOCITY_INTERPOLATOR_H
