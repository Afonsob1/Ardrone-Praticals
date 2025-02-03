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

  PidController::Parameters p;
  p.k_p = 0.5; // 0.1 - 0.05
  p.k_i = 0.02; // 0.00
  p.k_d = 0.1; // 0.05
  pidX.setParameters(p);
  pidY.setParameters(p);

  p.k_p = 1; //0.5 - 1.0
  p.k_i = 0.02; // 0.0
  p.k_d = 0.1; // 0.0
  pidZ.setParameters(p);

  p.k_p = 0.8; // 1.5
  p.k_i = 0.03; // 0.0
  p.k_d = 0.15; // 0.0
  pidYaw.setParameters(p);

  // get ros parameter
  double max_angle, max_vel_mm, max_yaw;
  if(!nh_->getParam("/ardrone_driver/euler_angle_max", max_angle)) {
    std::cout << "Error reading parameter euler_angle_max" << std::endl;
    return;
  }

  if (!nh_->getParam("/ardrone_driver/control_vz_max", max_vel_mm)) {
    std::cout << "Error reading parameter control_vz_max" << std::endl;
    return;
  }

  if (!nh_->getParam("/ardrone_driver/control_yaw", max_yaw)) {
    std::cout << "Error reading parameter control_yaw" << std::endl;
    return;
  }

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
  static auto last_landed_time = std::chrono::steady_clock::now();

  // return if not in automatic mode
  if (!isAutomatic_) {
    // keep resetting this to make sure we use the current state as reference as soon as sent to automatic mode
    const double yaw = kinematics::yawAngle(x.q_WS);
    setPoseReference(x.t_WS[0], x.t_WS[1], x.t_WS[2], yaw);
    resetIntegrators();
    return;
  }

  auto status = droneStatus();

  // if landed and waypoints are available, takeoff
  if (status == DroneStatus::Landed && !waypoints_.empty() ) {

    // check if is landed for more than 1 second
    auto current_time = std::chrono::steady_clock::now();
    auto time_passed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_landed_time);
    if (time_passed.count() > 2000) {
      // if landed for more than 1 second, takeoff
      takeoff();
      resetIntegrators();
      return;
    }
  } else if (status != DroneStatus::Flying && status != DroneStatus::Flying2 && status != DroneStatus::Hovering) {
    return;
  }

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

    std::cout << "Current: " << curr_x << " " << curr_y << " " << curr_z << " " << curr_yaw
              << "Waypoint: " << currentWaypoint.x << " " << currentWaypoint.y << " " << currentWaypoint.z << " " << currentWaypoint.yaw
               << std::endl;


    if (sqrt( pow(curr_x - currentWaypoint.x,2) + pow(curr_y - currentWaypoint.y,2) + pow(curr_z - currentWaypoint.z, 2)) < currentWaypoint.posTolerance)
    {
      // if reached, remove current waypoint
      waypoints_.pop_front();

      // if last waypoint, land
      if (currentWaypoint.land) {
        
        land();
        last_landed_time = std::chrono::steady_clock::now();
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

}  // namespace arp

