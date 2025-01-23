/*
 * ViEkf.cpp
 *
 *  Created on: 20 Nov 2015
 *      Author: sleutene
 */

#include <opencv2/highgui/highgui.hpp>

#include <arp/ViEkf.hpp>
#include <arp/kinematics/Imu.hpp>

namespace arp {

ViEkf::ViEkf()
    : cameraModel_(0, 0, 0, 0, 0, 0,
                   arp::cameras::RadialTangentialDistortion(0,0,0,0))
{
  x_.t_WS.setZero();
  x_.q_WS.setIdentity();
  x_.v_W.setZero();
  x_.b_g.setZero();
  x_.b_a.setZero();
  P_.setZero();
}

// Set the pose of the camera relative to the IMU.
bool ViEkf::setCameraExtrinsics(const arp::kinematics::Transformation & T_SC)
{
  T_SC_ = T_SC;
  return true;
}

// Set the intrinsics of the camera, i.e. the camera model
bool ViEkf::setCameraIntrinsics(
    const arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> & cameraModel)
{
  cameraModel_ = cameraModel;
  return true;
}

// Set IMU noise parameters
void ViEkf::setImuNoiseParameters(double sigma_c_gyr, double sigma_c_acc,
                                  double sigma_c_gw, double sigma_c_aw)
{
  sigma_c_gyr_ = sigma_c_gyr;
  sigma_c_acc_ = sigma_c_acc;
  sigma_c_gw_ = sigma_c_gw;
  sigma_c_aw_ = sigma_c_aw;
}

// Set measurement noise parameter.
void ViEkf::setDetectorNoiseParameter(double sigma_imagePoint)
{
  sigma_imagePoint_ = sigma_imagePoint;
}

// Initialise the states
bool ViEkf::initialiseState(uint64_t timestampMicroseconds,
                            const arp::kinematics::RobotState & x,
                            const Eigen::Matrix<double, 15, 15> & P)
{
  timestampLastUpdateMicrosec_ = timestampMicroseconds;
  x_ = x;
  P_ = P;

  // reset propagation for publishing
  x_propagated_ = x_;
  timestampPropagatedMicrosec_ = timestampLastUpdateMicrosec_;

  return true;
}

// Has this EKF been initialised?
bool ViEkf::isInitialised() const
{
  return 0 != timestampLastUpdateMicrosec_;
}

// Get the states.
bool ViEkf::getState(uint64_t timestampMicroseconds,
                     arp::kinematics::RobotState & x,
                     Eigen::Matrix<double, 15, 15> * P)
{
  if (timestampPropagatedMicrosec_ == 0) {
    x = x_;
    if (P) {
      *P = P_;
    }
    return false;
  }

  if (timestampPropagatedMicrosec_ > timestampLastUpdateMicrosec_) {
    if (timestampPropagatedMicrosec_ - timestampLastUpdateMicrosec_ > 300000) {
      // stop propagation, this will just diverge
      // assign output
      x = x_propagated_;
      if (P) {
        *P = P_;  // Not 100% correct, we should have also propagated  P_...
      }
      return false;
    }
  }

  // run prediction as far as possible
  for (auto it_k_minus_1 = imuMeasurementDeque_.begin();
      it_k_minus_1 != imuMeasurementDeque_.end(); ++it_k_minus_1) {
    auto it_k = it_k_minus_1;
    it_k++;
    if (it_k == imuMeasurementDeque_.end()) {
      return false;  // we reached the buffer end...
    }

    // ensure we're in the right segment
    if (it_k->timestampMicroseconds < timestampPropagatedMicrosec_) {
      continue;
    }

    if (it_k->timestampMicroseconds >= timestampMicroseconds) {
      break;
    }

    // propagate and get state transition matrix
    arp::kinematics::ImuKinematicsJacobian F =
        arp::kinematics::ImuKinematicsJacobian::Identity();
    arp::kinematics::RobotState x_start = x_propagated_;
    arp::kinematics::Imu::stateTransition(x_start, *it_k_minus_1, *it_k,
                                          x_propagated_, &F);
  }
  // assign output
  x = x_propagated_;
  if (P) {
    *P = P_;  // Not 100% correct, we should have also propagated  P_...
  }

  // remember
  timestampPropagatedMicrosec_ = timestampMicroseconds;
  return true;
}

bool ViEkf::addImuMeasurement(uint64_t timestampMicroseconds,
                              const Eigen::Vector3d & gyroscopeMeasurements,
                              const Eigen::Vector3d & accelerometerMeasurements)
{
  imuMeasurementDeque_.push_back(
      arp::kinematics::ImuMeasurement { timestampMicroseconds,
          gyroscopeMeasurements, accelerometerMeasurements });
  return true;
}

// The EKF prediction.
bool ViEkf::predict(uint64_t from_timestampMicroseconds,
                    uint64_t to_timestampMicroseconds)
{
  for (auto it_k_minus_1 = imuMeasurementDeque_.begin();
      it_k_minus_1 != imuMeasurementDeque_.end(); ++it_k_minus_1) {
    auto it_k = it_k_minus_1;
    it_k++;
    if (it_k == imuMeasurementDeque_.end()) {
      break;  // we reached the buffer end...
    }

    // ensure we're in the right segment
    if (it_k->timestampMicroseconds < from_timestampMicroseconds
        || it_k->timestampMicroseconds >= to_timestampMicroseconds) {
      continue;
    }

    // access the IMU measurements:
    kinematics::ImuMeasurement z_k_minus_1 = *it_k_minus_1; // IMU meas. at last time step
    kinematics::ImuMeasurement z_k = *it_k; // IMU measurements at current time step

    // get the time delta
    const double delta_t = double(z_k.timestampMicroseconds - z_k_minus_1.timestampMicroseconds) * 1.0e-6;

    // propagate robot state x_ using IMU measurements
    // i.e. we do x_k = f(x_k_minus_1).
    // Also, we compute the matrix F (linearisation of f()) related to
    // delta_chi_k = F * delta_chi_k_minus_1.
    kinematics::ImuKinematicsJacobian F = kinematics::ImuKinematicsJacobian::Identity();
    if (!arp::kinematics::Imu::stateTransition(x_, z_k_minus_1, z_k, x_propagated_, &F)) {
      return false;  // failed to propagate state
    }
    x_propagated_.q_WS.normalize();
    x_ = x_propagated_; 


    // process noise matrix:
    Eigen::Matrix<double, 3, 3> sig_cg_squared_delta_t = pow(sigma_c_gyr_, 2) * delta_t * Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 3, 3> sig_ca_squared_delta_t = pow(sigma_c_acc_, 2) * delta_t * Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 3, 3> sig_cbg_squared_delta_t = pow(sigma_c_gw_, 2) * delta_t * Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 3, 3> sig_cba_squared_delta_t = pow(sigma_c_aw_, 2) * delta_t * Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 15, 15> LQLt = Eigen::Matrix<double, 15, 15>::Zero();
    LQLt.block<3,3>(3, 3) = sig_cg_squared_delta_t;
    LQLt.block<3,3>(6, 6) = sig_ca_squared_delta_t;
    LQLt.block<3,3>(9, 9) = sig_cbg_squared_delta_t;
    LQLt.block<3,3>(12, 12) = sig_cba_squared_delta_t;

    // propagate covariance matrix P_
    P_ = F * P_ * F.transpose() + LQLt;
  }

  return true;
}

// Pass a set of keypoint measurements to trigger an update.
bool ViEkf::addKeypointMeasurements(uint64_t timestampMicroseconds,
                  const DetectionVec & detectionVec)
{
  // let's do the propagation from last time to now:
  predict(timestampLastUpdateMicrosec_, timestampMicroseconds);

  // now we are ready to do the actual update
  int successes = 0;
  for(size_t k=0; k<detectionVec.size(); ++k){
    if(update(detectionVec[k])) {
      successes++;
    }
  }

  // monitor update history
  if(successes<4) {
    lastTimeReject_ = true;
    if(lastTimeReject_) {
      rejections_++;
    }
  } else {
    // reset monitoring
    lastTimeReject_ = false;
    rejections_ = 0;
  }

  // remember update
  timestampLastUpdateMicrosec_ = timestampMicroseconds;
  auto it = imuMeasurementDeque_.begin();

  // also delete stuff in the queue since not needed anymore.
  while(it != imuMeasurementDeque_.end()){
    auto it_p1 = it;
    it_p1++;
    if(it_p1 == imuMeasurementDeque_.end()){
      break;
    }
    if(it->timestampMicroseconds < timestampMicroseconds
        && it_p1->timestampMicroseconds >= timestampMicroseconds) {
      break;
    }
    it++;
  }
  imuMeasurementDeque_.erase(imuMeasurementDeque_.begin(),it);

  // reset propagation for publishing
  x_propagated_ = x_;
  timestampPropagatedMicrosec_ = timestampLastUpdateMicrosec_;

  if(rejections_>3) {
    std::cout << "REINITIALISE" << std::endl;
    timestampLastUpdateMicrosec_ = 0; // not initialised.
    rejections_ = 0;
    lastTimeReject_ = false;
  }
  return true;
}

// The EKF update.
bool ViEkf::update(const Detection & detection){

  // We avoid the use of kinematics::Transformation here due to quaternion normalization and so forth.
  // This only matters in order to be able to check Jacobians with numeric differentiation chained,
  // first w.r.t. q and then d_alpha.

  // pose: world to sensor transformation
  kinematics::Transformation T_WS(x_.t_WS,x_.q_WS);

  // landmark in World frame
  Eigen::Vector4d hp_W(0,0,0,1);
  hp_W.head<3>() = detection.landmark;

  // transform the corner point from world frame into the camera frame
  // (remember the camera projection will assume the point is represented
  // in camera coordinates):
  Eigen::Vector4d hp_C = T_SC_.inverse() * T_WS.inverse() * hp_W;

  // calculate the reprojection error y (residual)
  // using the PinholeCamera::project
  Eigen::Vector2d reprojection_point;
  Eigen::Matrix<double, 2, 3> U;

  // check validity of projection -- return false if not successful!
  if (cameraModel_.project(hp_C.head<3>(), &reprojection_point, &U) != cameras::ProjectionStatus::Successful){
    return false;
  }
  
  const Eigen::Vector2d y = detection.keypoint - reprojection_point;

  // calculate measurement Jacobian H
  Eigen::Matrix<double, 2, 15> H = Eigen::Matrix<double, 2, 15>::Zero();
  H.block<2,3>(0, 0) = U * T_SC_.R().transpose() * (-T_WS.R().transpose());
  H.block<2,3>(0, 3) = U * T_SC_.R().transpose() * T_WS.R().transpose() * arp::kinematics::crossMx(hp_W.head<3>() - x_.t_WS);

  // Obtain the measurement covariance form parameters:
  const double r = sigma_imagePoint_ * sigma_imagePoint_;
  Eigen::Matrix2d R = Eigen::Vector2d(r, r).asDiagonal();  // the measurement covariance

  // compute residual covariance S
  Eigen::Matrix2d S = H * P_ * H.transpose() + R;

  // chi2 test
  if(y.transpose() * S.inverse() * y > 40.0){
    std::cout << "Rejecting measurement " << std::endl;
    return false;
  }

  // compute Kalman gain K 15x2
  Eigen::Matrix<double, 15, 2> K = P_ * H.transpose() * S.inverse();

  // compute increment Delta_chi
  Eigen::Matrix<double, 15, 1> delta_chi = K * y; 
  Eigen::Vector3d dAlpha = delta_chi.segment<3>(3);

  // perform update. Note: multiplicative for the quaternion!!
  x_.t_WS += delta_chi.segment<3>(0);
  x_.q_WS = kinematics::deltaQ(dAlpha) * x_.q_WS;
  x_.q_WS.normalize();
  x_.v_W += delta_chi.segment<3>(6);
  x_.b_g += delta_chi.segment<3>(9);
  x_.b_a += delta_chi.segment<3>(12);

  // update to covariance matrix:
  P_ = (Eigen::Matrix<double, 15, 15>::Identity() - K * H) * P_;

  return true;
}

}  // namespace arp
