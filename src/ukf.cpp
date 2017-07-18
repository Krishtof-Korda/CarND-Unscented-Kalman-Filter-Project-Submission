#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = .8;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = .2;

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

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  
  ///* initialize the previous time stamp
  time_us_ = 0;
  
  ///* initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;
  
  ///* State dimension
  n_x_ = 5;
  
  ///* Augmented state dimension
  n_aug_ = 7;
  
  ///* set radar measurement dimension
  n_z_r_ = 3;
  
  ///* set laser measurement dimension
  n_z_l_ = 2;
  
  ///* Sigma point spreading parameter
  lambda_ = 3 - n_aug_;
  
  ///* create sigma point matrix
  Xsig_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  Xsig_.fill(0.0);
  
  // initial state vector
  x_ = VectorXd(n_x_);
  
  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);
  
  ///* Weights of sigma points
  weights_ = VectorXd (2*n_aug_+1);
  weights_(0) = lambda_/(lambda_ + n_aug_);
  for (int i = 1; i<2*n_aug_+1; i++) {
    weights_(i) = .5/(lambda_ + n_aug_);
  }
  
  // set Radar measurement covariance matrix
  R_radar_ = MatrixXd(n_z_r_, n_z_r_);
  R_radar_ << std_radr_ * std_radr_, 0.0, 0.0,
  0.0, std_radphi_ * std_radphi_, 0.0,
  0.0, 0.0, std_radrd_ * std_radrd_;
  
  // set Laser measurement covariance matrix
  R_laser_ = MatrixXd(n_z_l_,n_z_l_);
  R_laser_ << std_laspx_ * std_laspx_, 0,
              0, std_laspy_ * std_laspy_;
  
  Chi_ = 7.8;
  
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    
    cout << "Initializing UKF...: " << endl;
    
    
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
       DONE: Convert radar from polar to cartesian coordinates and initialize state.
       */
      
      cout << "UKF Initialization using RADAR... " << endl;
      
      // Initialize measurement covariance
      P_ << .6, 0, 0, 0, 0,
            0, .6, 0, 0, 0,
            0, 0, .6, 0, 0,
            0, 0, 0, .5, 0,
            0, 0, 0, 0, .5;
      
      // extract measurement parts for readability
      const double px = meas_package.raw_measurements_[0] * cos(meas_package.raw_measurements_[1]);
      const double py = meas_package.raw_measurements_[0] * sin(meas_package.raw_measurements_[1]);
      const double v = meas_package.raw_measurements_[2];
      const double phi = meas_package.raw_measurements_[1];
      
      /*set the state with the initial location, velocity, radar angle phi 
       between car and object (this not the yaw angle of the object), and zero object yaw rate */
      x_ << px, py, v, phi, 0;
      time_us_ = meas_package.timestamp_; std::cout << "||||||||||||||||||||time = " << time_us_ << "\n\n";
      is_initialized_ = true;
      
      cout << "Initialization of UKF complete!" << endl;
      
      return;
    }
    
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
       DONE: Initialize state.
       */
      
      cout << "UKF Initialization using LASER... " << endl;
      
      // Initialize measurement covariance
      P_ << .3, 0, 0, 0, 0,
            0, .3, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;
      
      // extract measurement parts for readability
      const double px = meas_package.raw_measurements_[0];
      const double py = meas_package.raw_measurements_[1];
      //set the state with the initial location, zero velocity, zero yaw, and zero yaw rate
      x_ << px, py, 0, 0, 0;
      
      time_us_ = meas_package.timestamp_; std::cout << "||||||||||||||||||time = " << time_us_ << "\n\n";
      is_initialized_ = true;
      
      cout << "Initialization of UKF complete!" << endl;
      
      return;
    }
    
    // done initializing, no need to predict or update
    is_initialized_ = true;
    
    return;
  }
  
  //compute the time elapsed between the current and previous measurements
  const double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;	//dt - expressed in seconds
  std::cout << "|||||||||||||||||||||dt = " << dt << "\n\n";
  if (dt<.05) {
    std::cout << "DELTA TIME IS ZERO!!!!!!!!!!!!!!!!!!!!!! \n\n";
    throw std::exception();
  }
  
  time_us_ = meas_package.timestamp_; std::cout << "||||||||||||||||||||||time = " << time_us_ << "\n\n";
    
  
  ///*** Call Prediction
  Prediction(dt);
  
  ///*** Decide whether to call UpdateLidar
  if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_==true) {
  
    // DONE: Call UpdateLidar
    UpdateLidar(meas_package);
  }
  
  ///***Decide whether to call UpdateRadar
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_==true) {
    // DONE: Call UpdateRadar
    UpdateRadar(meas_package);
  }
  
  // print the output
  cout << "x_ = " << x_ << endl;
  cout << "P_ = " << P_ << endl;

  
}


/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  
  // Generate Sigma Points
  
  // create augmented mean vector
  VectorXd x_aug(n_aug_);
  x_aug.fill(0.0);
  
  // create augmented state covariance
  MatrixXd P_aug(n_aug_, n_aug_);
  P_aug.fill(0.0);
  
  // create augmented sigma point matrix
  MatrixXd Xsig_aug(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug.fill(0.0);
  
  // create augmented mean state
  x_aug.head(n_x_) = x_;
  x_aug.segment(n_x_,2) << 0,0;
  //std::cout << "x_aug = \n" << x_aug << "\n\n";
  
  //create augmented covariance matrix
  MatrixXd Q_(2,2);
  Q_ << std_a_ * std_a_, 0,
        0, std_yawdd_ * std_yawdd_;

  // Fill augmented state covariance
  P_aug.topLeftCorner(n_x_,n_x_) = P_;
  P_aug.bottomRightCorner(n_aug_ - n_x_, n_aug_ - n_x_) = Q_;
  //std::cout << "P_aug = \n" << P_aug << "\n\n";
  
  // create square root matrix
  MatrixXd A_aug = P_aug.llt().matrixL();
  //std::cout << "A = \n" << A_aug << "\n\n";
  
  // Fill augmented sigma points
  // set first column of sigma point matrix
  Xsig_aug.col(0)  = x_aug;
  
  //set remaining sigma points
  for (int i = 0; i < n_aug_; i++)
  {
    Xsig_aug.col(i+1)     = x_aug + sqrt(lambda_ + n_aug_) * A_aug.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * A_aug.col(i);
    //std::cout << "Xsig_aug = \n" << Xsig_aug << "\n\n";
  }
  
  // TODO: Predict Sigma Points
  
  VectorXd f(n_x_);
  VectorXd nu(n_x_);
  for (int i=0; i<2 * n_aug_ + 1; i++){
    double eps = .001;
    //double p_x = Xsig_aug_(0,i);
    //double p_y = Xsig_aug_(1,i);
    double v = Xsig_aug(2,i);
    double psi = Xsig_aug(3,i);
    double psi_dot = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_acc = Xsig_aug(6,i);
    double dt2 = delta_t*delta_t;
    
    //  normalize angle
    psi = atan2( sin(psi), cos(psi) );
    
    // extract current sigma point for use later
    VectorXd xsig = Xsig_aug.col(i).segment(0,5);
    //std::cout << "xsig = \n" << x << "\n\n" << "Xsig_aug.col(i).segment(0,5) = \n" << Xsig_aug.col(i).segment(0,5) << "\n\n" << "Xsig_aug.col(i) = \n" << Xsig_aug.col(i) << "\n\n";
    
    // Create and fill state transition matrix whislt avoiding divide by zero
    if (fabs(psi_dot) < eps){
      f <<    v*cos(psi)*delta_t,
              v*sin(psi)*delta_t,
              0,
              psi_dot*delta_t,
              0;
    }
    
    else {
      f <<    (v/psi_dot)*(sin(psi + psi_dot*delta_t) - sin(psi)),
              (v/psi_dot)*(-cos(psi + psi_dot*delta_t) + cos(psi)),
              0,
              psi_dot*delta_t,
              0;
    }
    
    nu <<   .5*dt2*cos(psi)*nu_a,
            .5*dt2*sin(psi)*nu_a,
            delta_t*nu_a,
            .5*dt2*nu_acc,
            delta_t*nu_acc;
    /*std::cout << "x = \n" << x << "\n\n" << "f =\n" << f << "\n\n" << "nu =\n" << nu << "\n\n";
    std::cout << "Xsig_aug.col(i) before = \n" << Xsig_aug.col(i) << "\n\n";
    std::cout << "Xsig_.col(i) before = \n" << Xsig_.col(i) << "\n\n";*/
    
    Xsig_.col(i) = xsig + f + nu;
    //std::cout << "Xsig_.col(i) after = \n" << Xsig_.col(i) << "\n\n";
    
  }
  
  // TODO: Predict Mean and assign to x_
  //create vector for predicted state
  VectorXd x(n_x_);
  x.fill(0.0);
  for (int i=0; i<2*n_aug_+1; i++) {
    x += weights_(i)*Xsig_.col(i);
    //std::cout << "x in predict\n" << x << "\n\n" << "Xsig.col(i)\n" << Xsig_.col(i) << "\n\n";
  }
  
  // Store predicted mean in state x_
  x_ = x;
  //std::cout << "x_ in predict\n" << x_ << "\n\n";
  
  // TODO: Predict Covariance and assign to P_
  MatrixXd P(n_x_,n_x_);
  P.fill(0.0);
  
  VectorXd x_diff(n_x_);
  x_diff.fill(0.0);
  
  for (int i=0; i<2*n_aug_+1; i++) {
    // state difference
    x_diff = Xsig_.col(i) - x_;
    //std::cout << "x_diff in predict\n" << x_diff << "\n\n";
    
    // normalize angle
    Normalize(x_diff(3));
    
    // summing covariances at each sigma point
    P += weights_(i)*(x_diff)*(x_diff).transpose();
  }
  
  // Store predicted covariance in P_
  P_ = P;
  //std::cout << "P_ in predict\n" << P_ << "\n\n";
  
}



/*******************************************************************************
*********************** UPDATE LIDAR *******************************************
********************************************************************************
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  
  //create matrix for sigma points in measurement space
  MatrixXd Zsig(n_z_l_, 2 * n_aug_ + 1);
  Zsig.fill(0.0);
  
  //mean predicted measurement
  VectorXd z_pred(n_z_l_);
  z_pred.fill(0.0);
  
  //create matrix for cross correlation Tc
  MatrixXd Tc(n_x_, n_z_l_);
  Tc.fill(0.0);
  
  //measurement covariance matrix S
  MatrixXd S(n_z_l_, n_z_l_);
  S.fill(0.0);
  
  //Kalman gain
  MatrixXd K(n_x_, n_z_l_);
  K.fill(0.0);
  
  //transform sigma points into measurement space
  for (int i=0; i<2 * n_aug_ + 1; i++){
    double eps = .001;
    double p_x = Xsig_(0,i);
    double p_y = Xsig_(1,i);
    
    // avoid division by zero
    p_x = std::max(p_x,eps);
    p_y = std::max(p_y,eps);
    
    // Measurement model
    Zsig(0,i) = p_x;
    Zsig(1,i) = p_y;

  }
  
  // Call Estimator()
  Estimator(Zsig, z_pred, Tc, S, K, meas_package);
  
  
}

/*******************************************************************************
*********************** UPDATE RADAR *******************************************
********************************************************************************
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  
  // TODO: Predict Radar Measurement
  
  //create matrix for sigma points in measurement space
  MatrixXd Zsig(n_z_r_, 2 * n_aug_ + 1);
  Zsig.fill(0.0);
  
  //mean predicted measurement
  VectorXd z_pred(n_z_r_);
  z_pred.fill(0.0);
  
  //measurement covariance matrix S
  MatrixXd S(n_z_r_, n_z_r_);
  S.fill(0.0);
  
  //create matrix for cross correlation Tc
  MatrixXd Tc(n_x_, n_z_r_);
  Tc.fill(0.0);
  
  //create Kalman gain
  MatrixXd K(n_x_, n_z_r_);
  K.fill(0.0);
  
  //transform sigma points into measurement space
  for (int i=0; i<2 * n_aug_ + 1; i++){
    double eps = .001;
    double p_x = Xsig_(0,i);
    double p_y = Xsig_(1,i);
    double v = Xsig_(2,i);
    double psi = Xsig_(3,i);
    //double psi_dot = Xsig_(4,i);
    double rho = sqrt(p_x*p_x + p_y*p_y);
    /*std::cout << "i =\n" << i << "\n\n p_x in radar\n\n" << p_x << "\n\n";
    std::cout << "i =\n" << i << "\n\n p_y in radar\n\n" << p_y << "\n\n";
    std::cout << "i =\n" << i << "\n\n v in radar\n\n" << v << "\n\n";
    std::cout << "i =\n" << i << "\n\n psi in radar\n\n" << psi << "\n\n";*/
    
    //std::cout << "i =\n" << i << "\n\n rho before std::max() in radar\n\n" << rho << "\n\n";
    // avoid division by zero
    rho = std::max(rho, eps);
    //std::cout << "i =\n" << i << "\n\n rho in radar\n\n" << rho << "\n\n";

    double phi = atan2(p_y,p_x);
    double rho_dot = (p_x*cos(psi)*v + p_y*sin(psi)*v)/rho;
    
    // Measurement model
    Zsig(0,i) = rho;
    Zsig(1,i) = phi;
    Zsig(2,i) = rho_dot;
    
    //std::cout << "i =\n" << i << "\n\n Zsig in radar\n\n" << Zsig << "\n\n";
  }
  
  // Call Estimator()
  Estimator(Zsig, z_pred, Tc, S, K, meas_package);
 
}


//Normalize angle function
void UKF::Normalize(double& angle) {
  // angle normalization
  angle = atan2( sin(angle), cos(angle) );
}


// Function for reused esitimation and update math
void UKF::Estimator(MatrixXd& Zsig, VectorXd& z_pred, MatrixXd& Tc, MatrixXd& S, MatrixXd& K, MeasurementPackage meas_package) {
  
  // Pull latest measurement from the pack
  VectorXd z = meas_package.raw_measurements_;
  //std::cout << "z in estimator\n" << z << "\n\n";
  
  //calculate mean predicted measurement
  for (int i=0; i<2*n_aug_+1; i++) {
    z_pred += weights_(i)*Zsig.col(i);
    //std::cout << "z_pred = in estimator\n" << z_pred << "\n\n";
  }
  
  VectorXd z_diff;
  z_diff.fill(0.0);
  
  VectorXd x_diff(n_x_);
  x_diff.fill(0.0);
  
  //calculate measurement covariance matrix S
  for (int i=0; i<2*n_aug_+1; i++) {
    // sigma point difference
    z_diff = Zsig.col(i)-z_pred;
    //std::cout << "z_diff in estimator\n" << z_diff << "\n\n";
    
    // Normalize angle if radar
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) Normalize(z_diff(1));
    
    // state difference
    x_diff = Xsig_.col(i) - x_;
    //std::cout << "x_diff in estimator\n" << x_diff << "\n\n";
    
    // normalize angle
    Normalize(x_diff(3));
    
    //std::cout << "z_diff in estimator\n" << z_diff << "\n\n";
    //std::cout << "x_diff in estimator\n" << x_diff << "\n\n";
    
    //calculate cross correlation matrix
    Tc += weights_(i)*(x_diff)*(z_diff).transpose();
    
    // calculate measurement covariance matrix S
    S += weights_(i)*(z_diff)*(z_diff).transpose();
    
  }
  
  // add measurement noise to covariance
  if (meas_package.sensor_type_ == MeasurementPackage::LASER) S += R_laser_;
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) S += R_radar_;
  //std::cout << "S in estimator\n" << S << "\n\n";
  //std::cout << "Tc in estimator\n" << Tc << "\n\n";
  
  //calculate Kalman gain K;
  K = Tc*S.inverse();
  //std::cout << "K in estimator = \n" << K << "\n\n";
  
  // difference between measurement and predicted
  VectorXd y = z - z_pred;
  //std::cout << "y in estimator = \n" << y << "\n\n";
  
  // angle normalization
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) Normalize(y(1));
  
  ///* Update state mean and covariance matrix
  x_ += K*y;
  //std::cout << "x_ in estimator\n" << x_ << "\n\n";
  P_ -= K*S*K.transpose();
  //std::cout << "P_ in laser\n" << P_ << "\n\n";
  
  ///* Calculate the NIS parameter
  if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    
    double NIS_laser = y.transpose() * S.inverse() * y;

    NIS_laser_.push_back(NIS_laser);
    std::cout << "NIS_laser_ = " << NIS_laser << "\n\n";
  }
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    double NIS_radar = y.transpose() * S.inverse() * y;

    NIS_radar_.push_back(NIS_radar);
    std::cout << "NIS_radar_ = " << NIS_radar << "\n\n";
    
  }
  
}
//********************************************************
