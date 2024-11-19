/*
 * Imu.cpp
 *
 *  Created on: 8 Feb 2017
 *      Author: sleutene
 */

#include <arp/kinematics/Imu.hpp>
#include <iostream>

namespace arp {
namespace kinematics {

Eigen::Matrix<double, 15, 15> calculateFc(const RobotState & state_k)
{
  Eigen::Matrix<double, 15, 15> Fc;

  Fc.block<3,3>(0, 6) = Eigen::Matrix::Identity();  

  Eigen::Matrix<double, 3, 3> R_WS = state_k.q_WS.toRotationMatrix();
  Fc.block<3,3>(3, 9) = -R_WS;
  
  return Fc;
}

bool Imu::stateTransition(const RobotState & state_k_minus_1,
                          const ImuMeasurement & z_k_minus_1,
                          const ImuMeasurement & z_k, RobotState & state_k,
                          ImuKinematicsJacobian* jacobian)
{
  // get the time delta
  const double dt = double(
      z_k.timestampMicroseconds - z_k_minus_1.timestampMicroseconds) * 1.0e-6;
  if (dt < 1.0e-12 || dt > 0.05) {
    // for safety, we assign reasonable values here.
    state_k = state_k_minus_1;
    if(jacobian) {
      jacobian->setIdentity();
    }
    return false;  // negative, no or too large time increments not permitted
  }

  // TODO: implement trapezoidal integration
  auto fc_1 = calculateFc(state_k_minus_1);

  std::cout << fc_1 << std::endl;

  if (jacobian) {
    // TODO: if requested, impement jacobian of trapezoidal integration with chain rule
  }
  return true;
}

}
}  // namespace arp

