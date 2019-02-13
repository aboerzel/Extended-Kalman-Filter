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
	 * @param F State transition matrix
	 * @param Q Process noise covariance matrix
	 */
	void Predict(const Eigen::MatrixXd& F, const Eigen::MatrixXd& Q);

	/**
	 * Updates the state by using standard Kalman Filter equations
	 * @param z The measurement at k+1
	 * @param R Measurement noise covariance matrix
	 * @param H Measurement matrix
	 */
	void Update(const Eigen::VectorXd& z, const Eigen::MatrixXd& R, const Eigen::MatrixXd& H);

	/**
	 * Updates the state by using Extended Kalman Filter equations
	 * @param z The measurement at k+1
	 * @param R Measurement noise covariance matrix
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

    /**
     * Normalizes the given angle between PI and -PI
     * @param angle Angle to be normalized
     * @return Normalized angle
     */
    static double NormalizeAngle(double angle);

    /**
     * Updates the state and the state covariance matrix
	 * @param y The innovation at k+1
	 * @param R Measurement noise covariance matrix
	 * @param H Measurement matrix
     */
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
