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

  Eigen::Matrix<double, 15, 15> Fc;
  Fc.block<3,3>(0, 6) = Eigen::Matrix3d::Identity();  
  Fc.block<3,3>(3, 9) = -R_WS;
  Fc.block<3,3>(6, 3) = -arp::kinematics::crossMx(R_WS * (z_k.acc_S - state_k.b_a));
  Fc.block<3,3>(6, 12) = -R_WS;

  return Fc;
}

RobotState calculate_d_chi( const double dt,
                            const RobotState & state_k, 
                            const ImuMeasurement & z_k)
{
  Eigen::Matrix<double, 15, 15> Fc = calculate_fc(state_k, z_k);

  Eigen::VectorXd x(15);
  x.segment<3>(0) = state_k.t_WS;
  x.segment<3>(3) = Eigen::Vector3d::Zero();
  x.segment<3>(6) = state_k.v_W;
  x.segment<3>(9) = state_k.b_g;
  x.segment<3>(12) = state_k.b_a;

  auto d_chi = dt * Fc * x;

  RobotState state;
  state.t_WS = d_chi.segment<3>(0);
  state.q_WS = arp::kinematics::deltaQ(d_chi.segment<3>(3));
  state.v_W = d_chi.segment<3>(6);
  state.b_g = d_chi.segment<3>(9);
  state.b_a = d_chi.segment<3>(12);

  return state;
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
  auto d_chi_1 = calculate_d_chi(dt, state_k_minus_1, z_k_minus_1);
  auto sum_state_k_minus_1_chi_1 = state_k_minus_1 + d_chi_1;
  auto d_chi_2 = calculate_d_chi(dt, sum_state_k_minus_1_chi_1, z_k);
  state_k = state_k_minus_1 + 1/2 * (d_chi_1 + d_chi_2);
  
  if (jacobian) {
    Eigen::Matrix<double, 15, 15> I = Eigen::Matrix<double, 15, 15>::Identity();
    Eigen::Matrix<double, 15, 15> Fc_minus_1 = calculate_fc(state_k_minus_1, z_k_minus_1);
    Eigen::Matrix<double, 15, 15> Fc_minus_1_d_chi_1 = calculate_fc(sum_state_k_minus_1_chi_1, z_k);

    *jacobian = I + dt/2.0 * Fc_minus_1 + dt/2.0 * (Fc_minus_1_d_chi_1 * (I + dt * Fc_minus_1));
  }
  return true;
}

}
}  // namespace arp

