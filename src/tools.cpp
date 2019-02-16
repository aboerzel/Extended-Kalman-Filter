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

    // check non empty and same size
    if (estimations.size() != ground_truth.size() || estimations.empty())
    {
        std::cout << "ERROR - CalculateRMSE() - Arguments invalid." << std::endl;
        return rmse;
    }

    // accumulate squared residuals
    for (unsigned int i = 0; i < estimations.size(); ++i)
    {
        VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array() * residual.array();
        rmse += residual;
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
    auto c1 = px * px + py * py;
    auto c2 = sqrt(c1);
    auto c3 = (c1 * c2);

    // check division by zero
    if (fabs(c1) < 0.0001)
    {
        std::cout << "ERROR - CalculateJacobian () - Division by Zero" << std::endl;
        return Hj;
    }

    // compute the Jacobian matrix
    Hj << (px / c2),                   (py / c2),                      0,       0,
         -(py / c1),                   (px / c1),                      0,       0,
         py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;

    return Hj;
}
