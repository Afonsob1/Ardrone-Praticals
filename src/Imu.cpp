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

Eigen::Matrix<double, 15, 15> calculate_fc(const RobotState & state_k, const ImuMeasurement & z_k){
  Eigen::Matrix<double, 3, 3> R_WS = state_k.q_WS.toRotationMatrix();

  Eigen::Matrix<double, 15, 15> Fc = Eigen::Matrix<double, 15, 15>::Zero();
  Fc.block<3,3>(0, 6) = Eigen::Matrix3d::Identity();  
  Fc.block<3,3>(3, 9) = -R_WS;
  Fc.block<3,3>(6, 3) = -arp::kinematics::crossMx(R_WS * (z_k.acc_S - state_k.b_a));
  Fc.block<3,3>(6, 12) = -R_WS;

  return Fc;
}

RobotState calculate_d_chi(const double dt,
                            const RobotState & state_k, 
                            const ImuMeasurement & z_k)
{
  Eigen::Matrix<double, 15, 15> Fc = calculate_fc(state_k, z_k);

  auto R = state_k.q_WS.toRotationMatrix();
  RobotState result;
  result.t_WS = dt*state_k.v_W;
  result.q_WS = arp::kinematics::deltaQ(dt*R*(z_k.omega_S - state_k.b_g));
  result.v_W = dt*(R * (z_k.acc_S - state_k.b_a) + Eigen::Vector3d(0,0,-9.81));
  result.b_g = Eigen::Vector3d::Zero();
  result.b_a = Eigen::Vector3d::Zero();

  return result;
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

  auto d_chi_1 = calculate_d_chi(dt, state_k_minus_1, z_k_minus_1);

  RobotState sum_state_k_minus_1_chi_1 = state_k_minus_1 + d_chi_1;
  auto d_chi_2 = calculate_d_chi(dt, sum_state_k_minus_1_chi_1, z_k);

  state_k.t_WS = state_k_minus_1.t_WS + (d_chi_1.t_WS + d_chi_2.t_WS) * 0.5;
  state_k.q_WS = Eigen::Quaterniond( (d_chi_1.q_WS.coeffs() + d_chi_2.q_WS.coeffs()) * 0.5 ) * state_k_minus_1.q_WS ;
  state_k.v_W = state_k_minus_1.v_W + (d_chi_1.v_W + d_chi_2.v_W) * 0.5;
  state_k.b_g = state_k_minus_1.b_g;
  state_k.b_a = state_k_minus_1.b_a;

  if (jacobian) {
    Eigen::Matrix<double, 15, 15> I = Eigen::Matrix<double, 15, 15>::Identity();
    Eigen::Matrix<double, 15, 15> Fc_minus_1 = calculate_fc(state_k_minus_1, z_k_minus_1);
    Eigen::Matrix<double, 15, 15> Fc_minus_1_d_chi_1 = calculate_fc(sum_state_k_minus_1_chi_1, z_k);
    *jacobian = I + 0.5 * dt * Fc_minus_1 + 0.5 * dt * (Fc_minus_1_d_chi_1 * (I + dt * Fc_minus_1));
  }

  return true;
}

}
}  // namespace arp
