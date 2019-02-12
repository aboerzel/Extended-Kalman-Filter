#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd>& estimations, const vector<VectorXd>& ground_truth)
{
    /**
     * TODO: Calculate the RMSE here.
     */
    VectorXd r;
    return r;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x)
{
    // Calculate a Jacobian here.
    MatrixXd Hj(3, 4);

    if (x.size() != 4)
    {
        std::cout << "ERROR - CalculateJacobian () - The state vector must have size 4." << std::endl;
        return Hj;
    }

    //recover state parameters
    auto px = x(0);
    auto py = x(1);
    auto vx = x(2);
    auto vy = x(3);

    //pre-compute a set of terms to avoid repeated calculation
    auto c1 = px * px + py * py;
    auto c2 = sqrt(c1);
    auto c3 = (c1 * c2);

    //check division by zero
    if (fabs(c1) < 0.0001)
    {
        std::cout << "ERROR - CalculateJacobian () - Division by Zero" << std::endl;
        return Hj;
    }

    //compute the Jacobian matrix
    Hj << (px / c2), (py / c2), 0, 0,
          -(py / c1), (px / c1), 0, 0,
          py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;

    return Hj;
}
