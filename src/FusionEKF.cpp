#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF()
{
    is_initialized_ = false;

    previous_timestamp_ = 0;

    //measurement noise covariance matrix - laser
    R_laser_ = MatrixXd(2, 2);
    R_laser_ << 0.0225, 0,
                0, 0.0225;

    //measurement noise covariance matrix - radar
    R_radar_ = MatrixXd(3, 3);
    R_radar_ << 0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;

    // measurement matrix - laser
    H_laser_ = MatrixXd(2, 4);
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage& measurement_pack)
{
    /**
     * Initialization
     */
    if (!is_initialized_)
    {
        // initial state covariance matrix P
        auto P = MatrixXd(4, 4);
        P << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;

        // first measurement
        cout << "Initialization" << endl;
        auto state = VectorXd(4);

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
        {
            // Convert radar from polar to cartesian coordinates and initialize state.
            float ro = measurement_pack.raw_measurements_(0);
            float phi = measurement_pack.raw_measurements_(1);
            float ro_dot = measurement_pack.raw_measurements_(2);

            state << ro * cos(phi), ro * sin(phi), ro_dot * cos(phi), ro_dot * sin(phi);
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
        {
            // Initialize state.
            state << measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1), 0, 0;
        }
        else
        {
            return;
        }

        ekf_.Init(state, P);

        previous_timestamp_ = measurement_pack.timestamp_;

        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }

    /**
     * Prediction
     */

    // Calculate time between two calls in seconds.
    auto dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;

    // State transition matrix computation
    auto F = MatrixXd(4, 4);
    F << 1, 0, dt, 0,
         0, 1, 0, dt,
         0, 0, 1, 0,
         0, 0, 0, 1;

    // Noise covariance matrix computation
    auto noise_ax = 9.0;
    auto noise_ay = 9.0;

    auto dt_2 = dt * dt; //dt^2
    auto dt_3 = dt_2 * dt; //dt^3
    auto dt_4 = dt_3 * dt; //dt^4
    auto dt_4_4 = dt_4 / 4; //dt^4/4
    auto dt_3_2 = dt_3 / 2; //dt^3/2

    auto Q = MatrixXd(4, 4);
    Q << dt_4_4 * noise_ax, 0, dt_3_2 * noise_ax, 0,
         0, dt_4_4 * noise_ay, 0, dt_3_2 * noise_ay,
         dt_3_2 * noise_ax, 0, dt_2 * noise_ax, 0,
         0, dt_3_2 * noise_ay, 0, dt_2 * noise_ay;

    ekf_.Predict(F, Q);

    /**
     * Update
     */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
        // Radar updates
        ekf_.UpdateEKF(measurement_pack.raw_measurements_, R_radar_);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
        // Laser updates
        ekf_.Update(measurement_pack.raw_measurements_, R_laser_, H_laser_);
    }
    else
    {
        return;
    }

    // print the output
    cout << "x = " << ekf_.GetState() << endl;
    cout << "P = " << ekf_.GetP() << endl;
}
