#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd>& estimations, const vector<VectorXd>& ground_truth)
{
    // Calculate the RMSE here.
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    if (estimations.size() != ground_truth.size() || estimations.empty())
    {
        std::cout << "ERROR - CalculateRMSE() - Arguments invalid." << std::endl;
        return rmse;
    }

    // accumulate squared errors
    for (unsigned int i = 0; i < estimations.size(); ++i)
    {
        VectorXd err = estimations[i] - ground_truth[i];
        err = err.array() * err.array();
        rmse += err;
    }

    // calculate the RMSE
    rmse = rmse / estimations.size();
    rmse = rmse.array().sqrt();
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state)
{
    // Calculate a Jacobian here.
    MatrixXd Hj(3, 4);

    if (x_state.size() != 4)
    {
        std::cout << "ERROR - CalculateJacobian () - Argument invalid." << std::endl;
        return Hj;
    }

    // state parameters
    auto px = x_state(0);
    auto py = x_state(1);
    auto vx = x_state(2);
    auto vy = x_state(3);

    // calculate some terms that are needed multiple times
    auto t1 = px * px + py * py;
    auto t2 = sqrt(t1);
    auto t3 = (t1 * t2);

    // avoid division by zero
    if (fabs(t1) < 0.0001)
    {
        std::cout << "ERROR - CalculateJacobian () - Division by zero" << std::endl;
        return Hj;
    }

    // compute the Jacobian matrix
    Hj << (px / t2),                   (py / t2),                      0,       0,
         -(py / t1),                   (px / t1),                      0,       0,
         py * (vx * py - vy * px) / t3, px * (px * vy - py * vx) / t3, px / t2, py / t2;

    return Hj;
}
