#include "angular_velocity_interpolator.h"
#include <steam.hpp>

std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Matrix3d>>
interpolateAngularVelocities(const std::vector<double>& measurementStamps, const std::vector<double>& velocityStamps, const std::vector<Eigen::Vector3d>& angularVelocities,
                             const std::vector<Eigen::Matrix<double, 3, 3>>& informationMatrices, const double& qcDiag)
{
    Eigen::Matrix<double, 6, 1> jerkNoise = Eigen::Matrix<double, 6, 1>::Ones();
    jerkNoise.bottomRows<3>() *= qcDiag;
    steam::traj::const_acc::Interface traj(jerkNoise);
    steam::OptimizationProblem problem;
    for(size_t i = 0; i < velocityStamps.size(); ++i)
    {
        Eigen::Matrix<double, 4, 4> pose = Eigen::Matrix<double, 4, 4>::Identity();
        Eigen::Matrix<double, 6, 1> velocity = Eigen::Matrix<double, 6, 1>::Zero();
        velocity.bottomRows(3) = angularVelocities[i];
        Eigen::Matrix<double, 6, 6> informationMatrix = Eigen::Matrix<double, 6, 6>::Identity() * 1e6;
        informationMatrix.bottomRightCorner<3, 3>() = informationMatrices[i];

        std::shared_ptr<steam::se3::SE3StateVar> T_vi = steam::se3::SE3StateVar::MakeShared(lgmath::se3::Transformation(pose));
        std::shared_ptr<steam::vspace::VSpaceStateVar<6>> w_iv_inv = steam::vspace::VSpaceStateVar<6>::MakeShared(velocity);
        std::shared_ptr<steam::vspace::VSpaceStateVar<6>> dw_iv_inv = steam::vspace::VSpaceStateVar<6>::MakeShared(Eigen::Matrix<double, 6, 1>::Zero());
        if(i == 0)
        {
            T_vi->locked() = true;
        }
        traj.add(steam::traj::Time(velocityStamps[i] - velocityStamps[0]), T_vi, w_iv_inv, dw_iv_inv);
        problem.addStateVariable(T_vi);
        problem.addStateVariable(w_iv_inv);
        problem.addStateVariable(dw_iv_inv);

        std::shared_ptr<steam::vspace::VSpaceErrorEvaluator<6>> errorFunc = steam::vspace::VSpaceErrorEvaluator<6>::MakeShared(w_iv_inv, velocity);
        std::shared_ptr<steam::StaticNoiseModel<6>> noiseModel = steam::StaticNoiseModel<6>::MakeShared(informationMatrix, steam::NoiseType::INFORMATION);
        std::shared_ptr<steam::L2LossFunc> lossFunc = steam::L2LossFunc::MakeShared();
        std::shared_ptr<steam::WeightedLeastSqCostTerm<6>> costTerm = steam::WeightedLeastSqCostTerm<6>::MakeShared(errorFunc, noiseModel, lossFunc);
        problem.addCostTerm(costTerm);
    }
    traj.addPriorCostTerms(problem);
    steam::GaussNewtonSolver::Params params;
    params.verbose = false;
    steam::GaussNewtonSolver solver(problem, params);
    solver.optimize();

    std::vector<Eigen::Vector3d> interpolatedAngularVelocities;
    steam::Covariance covariance(problem);
    std::vector<Eigen::Matrix3d> interpolatedAngularVelocityCovariances;
    for(size_t i = 0; i < measurementStamps.size(); ++i)
    {
        interpolatedAngularVelocities.emplace_back(traj.getVelocityInterpolator(measurementStamps[i] - measurementStamps[0])->value().matrix().bottomRows<3>());
        if(i == 0)
        {
            interpolatedAngularVelocityCovariances.emplace_back(informationMatrices[i].inverse());
        }
        else
        {
            interpolatedAngularVelocityCovariances.emplace_back(traj.getCovariance(covariance, measurementStamps[i] - measurementStamps[0]).block<3, 3>(9, 9));
        }
    }
    return std::make_pair(interpolatedAngularVelocities, interpolatedAngularVelocityCovariances);
}
