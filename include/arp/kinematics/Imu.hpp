#ifndef ARP_KINEMATICS_TRANSFORMATION_HPP
#define ARP_KINEMATICS_TRANSFORMATION_HPP

#include <Eigen/Core>
#include <arp/kinematics/Transformation.hpp>
#include <arp/kinematics/operators.hpp>

namespace arp {
namespace kinematics {

struct RobotState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d t_WS;  ///< The position relative to the W frame.
  Eigen::Quaterniond q_WS;  ///< The quaternion of rotation W-S.
  Eigen::Vector3d v_W;  ///< The velocity expressed in W frame.
  Eigen::Vector3d b_g;  ///< The gyro bias.
  Eigen::Vector3d b_a;  ///< The accelerometer bias.

  RobotState(){}
  RobotState(Eigen::Vector3d t_WS, Eigen::Quaterniond q_WS, Eigen::Vector3d v_W, Eigen::Vector3d b_g, Eigen::Vector3d b_a) : 
                        t_WS(t_WS), q_WS(q_WS), v_W(v_W), b_g(b_g), b_a(b_a) {}

  RobotState operator+(const RobotState& d_chi) const {
    RobotState r;
    r.t_WS = t_WS + d_chi.t_WS;
    r.q_WS = d_chi.q_WS * q_WS;
    r.q_WS.normalize();
    r.v_W = v_W + d_chi.v_W;
    r.b_g = b_g + d_chi.b_g;
    r.b_a = b_a + d_chi.b_a;
    return r;
  }

};

typedef Eigen::Matrix<double,15,15> ImuKinematicsJacobian;

struct ImuMeasurement {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  uint64_t timestampMicroseconds; ///< Timestamp of the measurement in [usec].
  Eigen::Vector3d omega_S; ///< The gyro reading omega_tilde_S [rad/s].
  Eigen::Vector3d acc_S; ///< The accelerometer reading a_tilde_S [rm/s^2/s].
};

class Imu
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /// \brief Default constructor, does nothing
  Imu()
  {
  }

  /// \brief Implements the discrete-time nonlinear IMU kinematics
  ///        x_k = f(x_k_minus_1,z_imu). It will also compute the
  ///        linearised kinematics in the sense
  ///        delta_chi_k \approx F_k * delta_chi_k_minus_1.
  /// @param[in]  state_k_minus_1  Last state.
  /// @param[in]  z_k_minus_1      Last IMU measurement.
  /// @param[in]  z_k              Current IMU measurement.
  /// @param[out] state_k          Current state computed.
  /// @param[out] jacobian         state transition matrix F_k.
  static bool stateTransition(const RobotState & state_k_minus_1,
                              const ImuMeasurement & z_k_minus_1,
                              const ImuMeasurement & z_k, RobotState & state_k,
                              ImuKinematicsJacobian* jacobian = nullptr);

};

}
} // namespace arp

#endif // ARP_KINEMATICS_TRANSFORMATION_HPP
