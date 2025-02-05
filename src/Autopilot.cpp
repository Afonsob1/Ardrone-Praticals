/*
 * Autopilot.cpp
 *
 *  Created on: 10 Jan 2017
 *      Author: sleutene
 */

#include <arp/Autopilot.hpp>
#include <arp/kinematics/operators.hpp>
#include <math.h>

namespace arp {

Autopilot::Autopilot(ros::NodeHandle& nh)
    : nh_(&nh)
{
  isAutomatic_ = false; // always start in manual mode  

  // receive navdata
  subNavdata_ = nh.subscribe("ardrone/navdata", 50, &Autopilot::navdataCallback,
                             this);

  // commands
  pubReset_ = nh_->advertise<std_msgs::Empty>("/ardrone/reset", 1);
  pubTakeoff_ = nh_->advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
  pubLand_ = nh_->advertise<std_msgs::Empty>("/ardrone/land", 1);
  pubMove_ = nh_->advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  // flattrim service
  srvFlattrim_ = nh_->serviceClient<std_srvs::Empty>(
      nh_->resolveName("ardrone/flattrim"), 1);

  // get max values for PID controllers from ros parameter
  double max_angle, max_vel_mm, max_yaw;
  if(!nh_->getParam("/ardrone_driver/euler_angle_max", max_angle)) {
    std::cout << "Error reading parameter euler_angle_max" << std::endl;
  }

  if (!nh_->getParam("/ardrone_driver/control_vz_max", max_vel_mm)) {
    std::cout << "Error reading parameter control_vz_max" << std::endl;
  }

  if (!nh_->getParam("/ardrone_driver/control_yaw", max_yaw)) {
    std::cout << "Error reading parameter control_yaw" << std::endl;
  }

  // get values for PID X controllers from ros parameter
  double pid_X_k_p, pid_X_k_i, pid_X_k_d;
  if(!nh_->getParam("/arp_node/pid_X_k_p", pid_X_k_p)) {
    std::cout << "Error reading parameter pid_X_k_p" << std::endl;
  }

  if (!nh_->getParam("/arp_node/pid_X_k_i", pid_X_k_i)) {
    std::cout << "Error reading parameter pid_X_k_i" << std::endl;
  }

  if (!nh_->getParam("/arp_node/pid_X_k_d", pid_X_k_d)) {
    std::cout << "Error reading parameter pid_X_k_d" << std::endl;
  }

  // get values for PID Y controllers from ros parameter
  double pid_Y_k_p, pid_Y_k_i, pid_Y_k_d;
  if(!nh_->getParam("/arp_node/pid_Y_k_p", pid_Y_k_p)) {
    std::cout << "Error reading parameter pid_Y_k_p" << std::endl;
  }

  if (!nh_->getParam("/arp_node/pid_Y_k_i", pid_Y_k_i)) {
    std::cout << "Error reading parameter pid_Y_k_i" << std::endl;
  }

  if (!nh_->getParam("/arp_node/pid_Y_k_d", pid_Y_k_d)) {
    std::cout << "Error reading parameter pid_Y_k_d" << std::endl;
  }

  // get values for PID Z controllers from ros parameter
  double pid_Z_k_p, pid_Z_k_i, pid_Z_k_d;
  if(!nh_->getParam("/arp_node/pid_Z_k_p", pid_Z_k_p)) {
    std::cout << "Error reading parameter pid_Z_k_p" << std::endl;
  }

  if (!nh_->getParam("/arp_node/pid_Z_k_i", pid_Z_k_i)) {
    std::cout << "Error reading parameter pid_Z_k_i" << std::endl;
  }

  if (!nh_->getParam("/arp_node/pid_Z_k_d", pid_Z_k_d)) {
    std::cout << "Error reading parameter pid_Z_k_d" << std::endl;
  }

  // get values for PID Yaw controllers from ros parameter
  double pid_Yaw_k_p, pid_Yaw_k_i, pid_Yaw_k_d;
  if(!nh_->getParam("/arp_node/pid_Yaw_k_p", pid_Yaw_k_p)) {
    std::cout << "Error reading parameter pid_Yaw_k_p" << std::endl;
    return;
  }

  if (!nh_->getParam("/arp_node/pid_Yaw_k_i", pid_Yaw_k_i)) {
    std::cout << "Error reading parameter pid_Yaw_k_i" << std::endl;
    return;
  }

  if (!nh_->getParam("/arp_node/pid_Yaw_k_d", pid_Yaw_k_d)) {
    std::cout << "Error reading parameter pid_Yaw_k_d" << std::endl;
    return;
  }

  PidController::Parameters p_X;
  p_X.k_p = pid_X_k_p;
  p_X.k_i = pid_X_k_i;
  p_X.k_d = pid_X_k_d;
  pidX.setParameters(p_X);

  PidController::Parameters p_Y;
  p_Y.k_p = pid_Y_k_p;
  p_Y.k_i = pid_Y_k_i;
  p_Y.k_d = pid_Y_k_d;
  pidY.setParameters(p_Y);

  PidController::Parameters p_Z;
  p_Z.k_p = pid_Z_k_p;
  p_Z.k_i = pid_Z_k_i;
  p_Z.k_d = pid_Z_k_d;
  pidZ.setParameters(p_Z);

  PidController::Parameters p_Yaw;
  p_Yaw.k_p = pid_Yaw_k_p;
  p_Yaw.k_i = pid_Yaw_k_i;
  p_Yaw.k_d = pid_Yaw_k_d;
  pidYaw.setParameters(p_Yaw);

  double max_vel = max_vel_mm / 1000.0;

  // set pid limits
  pidX.setOutputLimits(-max_angle, max_angle);
  pidY.setOutputLimits(-max_angle, max_angle);
  pidZ.setOutputLimits(-max_vel, max_vel);
  pidYaw.setOutputLimits(-max_yaw, max_yaw);
}

void Autopilot::navdataCallback(const ardrone_autonomy::NavdataConstPtr& msg)
{
  std::lock_guard<std::mutex> l(navdataMutex_);
  lastNavdata_ = *msg;
}

// Get the drone status.
Autopilot::DroneStatus Autopilot::droneStatus()
{
  ardrone_autonomy::Navdata navdata;
  {
    std::lock_guard<std::mutex> l(navdataMutex_);
    navdata = lastNavdata_;
  }
  return DroneStatus(navdata.state);
}

// Get the drone battery.
float Autopilot::droneBattery()
{
  ardrone_autonomy::Navdata navdata;
  {
    std::lock_guard<std::mutex> l(navdataMutex_);
    navdata = lastNavdata_;
  }
  return navdata.batteryPercent;
}

// Request flattrim calibration.
bool Autopilot::flattrimCalibrate()
{
  DroneStatus status = droneStatus();
  if (status != DroneStatus::Landed) {
    return false;
  }
  // ARdrone -> flattrim calibrate
  std_srvs::Empty flattrimServiceRequest;
  srvFlattrim_.call(flattrimServiceRequest);
  return true;
}

// Takeoff.
bool Autopilot::takeoff()
{
  DroneStatus status = droneStatus();
  if (status != DroneStatus::Landed) {
    return false;
  }
  // ARdrone -> take off
  std_msgs::Empty takeoffMsg;
  pubTakeoff_.publish(takeoffMsg);
  return true;
}

// Land.
bool Autopilot::land()
{
  DroneStatus status = droneStatus();
  if (status != DroneStatus::Landed && status != DroneStatus::Landing
      && status != DroneStatus::Looping) {
    // ARdrone -> land
    std_msgs::Empty landMsg;
    pubLand_.publish(landMsg);
    return true;
  }
  return false;
}

// Turn off all motors and reboot.
bool Autopilot::estopReset()
{
  // ARdrone -> Emergency mode
  std_msgs::Empty resetMsg;
  pubReset_.publish(resetMsg);
  return true;
}

// Move the drone manually.
bool Autopilot::manualMove(double forward, double left, double up,
                           double rotateLeft)
{
  return move(forward, left, up, rotateLeft);
}

// Move the drone.
bool Autopilot::move(double forward, double left, double up,
                           double rotateLeft)
{
  DroneStatus status = droneStatus();
  if (status == DroneStatus::Flying || status == DroneStatus::Flying2 || status == DroneStatus::Hovering) {
    geometry_msgs::Twist moveMsg;
    moveMsg.linear.x = forward;
    moveMsg.linear.y = left;
    moveMsg.linear.z = up;
    moveMsg.angular.z = rotateLeft;
    pubMove_.publish(moveMsg);

    return true;
  }

  return false;
}

// Set to automatic control mode.
void Autopilot::setManual()
{
  isAutomatic_ = false;
}

// Set to manual control mode.
void Autopilot::setAutomatic()
{
  isAutomatic_ = true;
}

// Move the drone automatically.
bool Autopilot::setPoseReference(double x, double y, double z, double yaw)
{
  std::lock_guard<std::mutex> l(refMutex_);
  ref_x_ = x;
  ref_y_ = y;
  ref_z_ = z;
  ref_yaw_ = yaw;
  return true;
}

bool Autopilot::getPoseReference(double& x, double& y, double& z, double& yaw) {
  std::lock_guard<std::mutex> l(refMutex_);
  x = ref_x_;
  y = ref_y_;
  z = ref_z_;
  yaw = ref_yaw_;
  return true;
}

void Autopilot::resetIntegrators()
{
  pidX.resetIntegrator();
  pidY.resetIntegrator();
  pidZ.resetIntegrator();
  pidYaw.resetIntegrator();
}

/// The callback from the estimator that sends control outputs to the drone
void Autopilot::controllerCallback(uint64_t timeMicroseconds,
                                  const arp::kinematics::RobotState& x)
{

  // return if not in automatic mode
  if (!isAutomatic_) {
    // keep resetting this to make sure we use the current state as reference as soon as sent to automatic mode
    const double yaw = kinematics::yawAngle(x.q_WS);
    setPoseReference(x.t_WS[0], x.t_WS[1], x.t_WS[2], yaw);
    resetIntegrators();
    return;
  }

  checkLandedCallback(timeMicroseconds);

  std::lock_guard<std::mutex> l(waypointMutex_);
  if(!waypoints_.empty()) {
    
    // get the current waypoint from begining of the list
    auto currentWaypoint = waypoints_.front();

    // set pose reference from current waypoint
    setPoseReference(currentWaypoint.x, currentWaypoint.y, currentWaypoint.z, currentWaypoint.yaw);

    // check if current waypoint is reached
    double curr_x, curr_y, curr_z, curr_yaw;
    curr_x = x.t_WS[0];
    curr_y = x.t_WS[1];
    curr_z = x.t_WS[2];
    curr_yaw = kinematics::yawAngle(x.q_WS);

    //std::cout << "Current: " << curr_x << " " << curr_y << " " << curr_z << " " << curr_yaw
    //          << "Waypoint: " << currentWaypoint.x << " " << currentWaypoint.y << " " << currentWaypoint.z << " " << currentWaypoint.yaw
    //           << std::endl;
    // check if we reached an further waypoint before of the current one
    double smallest_distance = sqrt( pow(curr_x - currentWaypoint.x,2) + pow(curr_y - currentWaypoint.y,2) + pow(curr_z - currentWaypoint.z, 2));
    auto best_wp = waypoints_.begin();
    for (auto it = waypoints_.begin(); it != waypoints_.end(); it++) {
      auto wp = *it;
      if (wp.land) {
        break;
      }

      if (sqrt( pow(curr_x - wp.x,2) + pow(curr_y - wp.y,2) + pow(curr_z - wp.z, 2)) <= smallest_distance) {
        best_wp = it;
        smallest_distance = sqrt( pow(curr_x - wp.x,2) + pow(curr_y - wp.y,2) + pow(curr_z - wp.z, 2));
      }
    }

    // remove waypoints before the best one
    waypoints_.erase(waypoints_.begin(), best_wp);
    currentWaypoint = waypoints_.front();


    if (sqrt( pow(curr_x - currentWaypoint.x,2) + pow(curr_y - currentWaypoint.y,2) + pow(curr_z - currentWaypoint.z, 2)) < currentWaypoint.posTolerance)
    {
      // if reached, remove current waypoint
      waypoints_.pop_front();

      // if last waypoint, land
      if (currentWaypoint.land) {
        
        land();
        Autopilot::last_landed_time_ = ros::Time::now().toSec();
        resetIntegrators();
        return;
      }
    }
  } 

  // compute error signals
  double x_ref, y_ref, z_ref, yaw_ref;
  getPoseReference(x_ref, y_ref, z_ref, yaw_ref);
  auto pos_ref = Eigen::Vector3d(x_ref, y_ref, z_ref);

  auto R = x.q_WS.toRotationMatrix();
  auto pos_error = R.transpose() * (pos_ref - x.t_WS);

  double yaw_estimated = arp::kinematics::yawAngle(x.q_WS);
  double yaw_error = yaw_ref - yaw_estimated;
  
  if (yaw_error > M_PI) {
    yaw_error -= 2*M_PI;
  } else if (yaw_error < -M_PI) {
    yaw_error += 2*M_PI;
  }

  // compute error signal time derivatives
  auto pos_error_dot = - R.transpose() * x.v_W;
  auto yaw_error_dot = 0;

  // compute control output
  double u_x = pidX.control(timeMicroseconds, pos_error.x(), pos_error_dot.x());
  double u_y = pidY.control(timeMicroseconds, pos_error.y(), pos_error_dot.y());
  double u_z = pidZ.control(timeMicroseconds, pos_error.z(), pos_error_dot.z());
  double u_yaw = pidYaw.control(timeMicroseconds, yaw_error, yaw_error_dot);

  // send to move
  move(u_x, u_y, u_z, u_yaw);
}

void Autopilot::checkLandedCallback(uint64_t timeMicroseconds)
  {
    auto status = droneStatus();
    // if landed and waypoints are available, takeoff
    if (!isAutomatic_ ) return;
    if (status == DroneStatus::Landed && !waypoints_.empty()) {
      auto current_time = ros::Time::now().toSec();
      auto time_passed = current_time - Autopilot::last_landed_time_;
      if (time_passed > 3) {
        // if landed for more than 3 second, takeoff
        takeoff();
        resetIntegrators();
        return;
      }
    } else if (status != DroneStatus::Flying && status != DroneStatus::Flying2 && status != DroneStatus::Hovering) {
      return;
    }

}

}  // namespace arp

