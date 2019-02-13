#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <fstream>
#include "kalman_filter.h"
#include "measurement_package.h"

class FusionEKF
{
public:
    /**
     * Constructor.
     */
    FusionEKF();

    /**
     * Destructor.
     */
    virtual ~FusionEKF();

    /**
     * Run the whole flow of the Kalman Filter from here.
     */
    void ProcessMeasurement(const MeasurementPackage& measurement_pack);

    /**
     * Kalman Filter update and prediction math lives in here.
     */
    KalmanFilter ekf_;

private:
    // check whether the tracking toolbox was initialized or not (first measurement)
    bool is_initialized_;

    // previous timestamp
    long long previous_timestamp_;

    // Measurement noise covariance matrix - laser
    Eigen::MatrixXd R_laser_;

    // Measurement noise covariance matrix - radar
    Eigen::MatrixXd R_radar_;

    // Measurement matrix - laser
    Eigen::MatrixXd H_laser_;
};

#endif // FusionEKF_H_
