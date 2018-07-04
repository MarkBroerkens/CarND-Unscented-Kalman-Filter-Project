#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

#define EPS 0.001

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;
  
  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;
  
  n_x_ = 5;
  n_aug_ = n_x_ + 2;
  n_sigma_points_ = n_aug_ * 2 + 1 ;
  lambda_ = 3 - n_aug_;
  
  // initial state vector
  // p_x
  // p_y
  // v
  // yaw
  // yawd
  x_ = VectorXd( n_x_);
  
  // initial covariance matrix
  P_ = MatrixXd( n_x_,  n_x_);
  
  // Process noise standard deviation longitudinal acceleration in m/s^2
  // **** can be tuned ****
  std_a_ = 1.5;  // -3m/s^2 to 3m/s^2 in 95%
  
  // Process noise standard deviation yaw acceleration in rad/s^2
  // **** can be tuned ****
  std_yawdd_ = 0.5;  // -1rad/s^2 to 1rad/s^2 in 95%
  
  // DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;
  
  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;
  
  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;
  
  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;
  
  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  // DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

  Xsig_pred_ = MatrixXd(n_x_, n_sigma_points_);
  Xsig_pred_.setZero();

  is_initialized_ = false;

  // TODO: describe why we use these covariance matrixes
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << std_radr_ * std_radr_, 0, 0, 0, std_radphi_ * std_radphi_, 0, 0, 0, std_radrd_ * std_radrd_;

  R_laser_ = MatrixXd(2, 2);
  R_laser_ << std_laspx_ * std_laspx_, 0, 0, std_laspy_ * std_laspy_;

  // set weights
  weights_ = VectorXd(n_sigma_points_);
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights_(0) = weight_0;
  for (int i=1; i < n_sigma_points_; i++) {
    double weight = 0.5/(n_aug_+lambda_);
    weights_(i) = weight;
  }

  // the current NIS for radar
  NIS_radar_ = 0.0;

  // the current NIS for laser
  NIS_laser_ = 0.0;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The first measurement data of
 * either radar or laser.
 */
void UKF::Init(MeasurementPackage meas_package) {
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    // convert from polar to cartesian coordinates
    float ro =  meas_package.raw_measurements_[0];
    float theta =  meas_package.raw_measurements_[1];
    float px = cos(theta) * ro;
    float py = sin(theta) * ro;

    P_ <<   MatrixXd::Identity(n_x_, n_x_);
    P_(0, 0) = std_radr_*std_radr_;
    P_(1, 1) = std_radr_*std_radr_;
    x_ << px, py, 4, 0, 0;

  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    float px = meas_package.raw_measurements_[0];
    float py = meas_package.raw_measurements_[1];

    P_ <<  MatrixXd::Identity(n_x_, n_x_);
    P_(0, 0) = std_laspx_ * std_laspx_;
    P_(1, 1) = std_laspy_ * std_laspy_;

    x_ << px, py, 0, 0, 0;
  }
  // Deal with the special case initialisation problems
  if (fabs(x_(0)) < EPS && fabs(x_(1)) < EPS) {
    x_(0) = EPS;
    x_(1) = EPS;
  }


  previous_timestamp_ = meas_package.timestamp_;

  // done initializing, no need to predict or update
  is_initialized_ = true;
}

/**
 *  Angle normalization to [-Pi, Pi]
 */
void UKF::NormAng(double *ang) {
  while (*ang > M_PI) *ang -= 2. * M_PI;
  while (*ang < -M_PI) *ang += 2. * M_PI;
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    Init(meas_package);
  } else {
    // compute the time elapsed between the current and previous measurements
    float dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;    // dt - expressed in seconds
    previous_timestamp_ = meas_package.timestamp_;

    /*****************************************************************************
     *  Prediction
     ****************************************************************************/
    Prediction(dt);

    /*****************************************************************************
     *  Update
     ****************************************************************************/
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
      UpdateRadar(meas_package);
    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
      UpdateLidar(meas_package);
    }
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  // create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sigma_points_);
  AugmentedSigmaPoints(Xsig_aug);

  // Predict Sigma Points
  PredictSigmaPoints(Xsig_aug, delta_t);

  // Predict Mean and Covariance
  PredictMeanAndCovariance();
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  // set measurement dimension, lidar can measure px, py
  int n_z = 2;

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = Xsig_pred_.block(0, 0, n_z, n_sigma_points_);

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  // innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);

  MeasurementMeanAndCovariance(Zsig, z_pred, S);

  // add measurement noise covariance matrix
  S = S + R_laser_;

  UpdateState(Zsig, S, z_pred, meas_package, n_z);
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  // set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, n_sigma_points_);

  // transform sigma points into measurement space
  for (int i = 0; i < n_sigma_points_; i++) {
    // extract values for better readibility
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);
    double v  = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0, i) = sqrt(p_x*p_x + p_y*p_y);                        // r
    Zsig(1, i) = atan2(p_y, p_x);                                // phi
    Zsig(2, i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);    // r_dot
  }

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  // innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);

  MeasurementMeanAndCovariance(Zsig, z_pred, S);

  // add measurement noise covariance matrix
  S = S + R_radar_;

  UpdateState(Zsig, S, z_pred, meas_package, n_z);
}


void UKF::PredictMeanAndCovariance() {
  // predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < n_sigma_points_; i++) {  // iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  // predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < n_sigma_points_; i++) {  // iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    NormAng(&x_diff(3));

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
}

void UKF::MeasurementMeanAndCovariance(const MatrixXd &Zsig,
                                       VectorXd &z_pred, MatrixXd &S) {
  // mean predicted measurement

  z_pred.fill(0.0);
  for (int i=0; i < n_sigma_points_; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // innovation covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < n_sigma_points_; i++) {
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    NormAng(&z_diff(1));

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }
}

void UKF::AugmentedSigmaPoints(MatrixXd &Xsig_aug) {
  // create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  // create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);


  // create augmented mean state
  x_aug.head(n_x_) = x_;
  x_aug(5) = 0.0;
  x_aug(6) = 0.0;

  // create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5, 5) = std_a_*std_a_;
  P_aug(6, 6) = std_yawdd_*std_yawdd_;

  // create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  // create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++) {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }
}

void UKF::PredictSigmaPoints(MatrixXd &Xsig_aug, double delta_t) {
  for (int i = 0; i< n_sigma_points_; i++) {
    // extract values for better readability
    double p_x = Xsig_aug(0, i);
    double p_y = Xsig_aug(1, i);
    double v = Xsig_aug(2, i);
    double yaw = Xsig_aug(3, i);
    double yawd = Xsig_aug(4, i);
    double nu_a = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);

    // predicted state values
    double px_p, py_p;

    // avoid division by zero
    if (fabs(yawd) > EPS) {
      px_p = p_x + v/yawd * (sin(yaw + yawd*delta_t) - sin(yaw));
      py_p = p_y + v/yawd * (cos(yaw) - cos(yaw+yawd*delta_t));
    } else {
      px_p = p_x + v*delta_t*cos(yaw);
      py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    // add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    // write predicted sigma point into right column
    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }
}

void UKF::UpdateState(MatrixXd &Zsig, MatrixXd &S, VectorXd &z_pred,
                      MeasurementPackage meas_package, int n_z) {
  // Mean predicted measurement

  VectorXd z = VectorXd(n_z);
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    z <<
      meas_package.raw_measurements_(0),  // r
      meas_package.raw_measurements_(1),  // phi
      meas_package.raw_measurements_(2);  // r_dot
  } else {
    z <<
      meas_package.raw_measurements_(0),  // px
      meas_package.raw_measurements_(1);  // py
  }

  // create matrix for cross correlation Tc

  MatrixXd Tc = MatrixXd(n_x_, n_z);

  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < n_sigma_points_; i++) {
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      NormAng(&z_diff(1));
    }

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    NormAng(&x_diff(3));

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // residual
  VectorXd z_diff = z - z_pred;

  // angle normalization
  NormAng(&z_diff(1));

  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    NIS_radar_ = z.transpose() * S.inverse() * z;
  } else {
    NIS_laser_ = z.transpose() * S.inverse() * z;
  }
}
