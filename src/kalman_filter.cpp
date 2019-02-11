#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd& x_in, MatrixXd& P_in)
{
	x_ = x_in;
	P_ = P_in;

	// create identity matrix
	I_ = MatrixXd::Identity(x_.size(), x_.size());
}

const VectorXd& KalmanFilter::Predict(const MatrixXd &F, const MatrixXd &Q)
{
	// predict the state
	x_ = (F * x_);
	P_ = F * P_ * F.transpose() + Q;
	return x_;
}

void KalmanFilter::Update(const VectorXd& z, const MatrixXd &R, const MatrixXd &H)
{
	/**
	 * TODO: update the state by using Kalman Filter equations
	 */

	VectorXd y = z - (H * x_);
	MatrixXd S = H * P_ * H.transpose() + R;
	MatrixXd K = P_ * H.transpose() * S.inverse();

	x_ = x_ + (K * y);
	P_ = (I_ - (K * H)) * P_;
}

Eigen::VectorXd KalmanFilter::GetState()
{
	return x_;
}

Eigen::MatrixXd KalmanFilter::GetP()
{
	return P_;
}

void KalmanFilter::UpdateEKF(const VectorXd& z)
{
	/**
	 * TODO: update the state by using Extended Kalman Filter equations
	 */
}
