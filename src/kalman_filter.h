#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "tools.h"

class KalmanFilter
{
public:
	/**
	 * Constructor
	 */
	KalmanFilter();

	/**
	 * Destructor
	 */
	virtual ~KalmanFilter();

	/**
	 * Init Initializes Kalman filter
	 * @param x_in Initial state
	 * @param P_in Initial state covariance matrix
	 */
	void Init(Eigen::VectorXd& x_in, Eigen::MatrixXd& P_in);

	/**
	 * Prediction Predicts the state and the state covariance using the process model
	 * @param F Transition matrix
	 * @param Q Process covariance matrix
	 * @return predicted state
	 */
	const Eigen::VectorXd& Predict(const Eigen::MatrixXd& F, const Eigen::MatrixXd& Q);

	/**
	 * Updates the state by using standard Kalman Filter equations
	 * @param z The measurement at k+1
	 * @param R Measurement covariance matrix at k+1
	 * @param H Measurement matrix at k+1
	 */
	void Update(const Eigen::VectorXd& z, const Eigen::MatrixXd& R, const Eigen::MatrixXd& H);

	/**
	 * Updates the state by using Extended Kalman Filter equations
	 * @param z The measurement at k+1
	 * @param R Measurement covariance matrix at k+1
	 * @param H Measurement matrix at k+1
	 */
	void UpdateEKF(const Eigen::VectorXd& z, const Eigen::MatrixXd& R);

	/**
	* @return current state
	*/
	Eigen::VectorXd GetState();

	/**
	* @return current state covariance matrix
	*/
	Eigen::MatrixXd GetP();

private:

	void UpdateInnovation(const Eigen::VectorXd& y, const Eigen::MatrixXd& R, const Eigen::MatrixXd& H);

	// state vector
	Eigen::VectorXd x_;

	// state covariance matrix
	Eigen::MatrixXd P_;

	// identity matrix
	Eigen::MatrixXd I_;

    // tool object used to compute Jacobian and RMSE
    Tools tools;
};

#endif // KALMAN_FILTER_H_
