/*
 * PidController.cpp
 *
 *  Created on: 23 Feb 2017
 *      Author: sleutene
 */

#include <arp/PidController.hpp>
#include <stdexcept>
#include <algorithm>

namespace arp {

// Set the controller parameters
void PidController::setParameters(const Parameters & parameters)
{
  parameters_ = parameters;

}

// Implements the controller as u(t)=c(e(t))
double PidController::control(uint64_t timestampMicroseconds, double e,
                              double e_dot)
{
  // if it just started, we cannot compute the derivative
  if (lastTimestampMicroseconds_ == 0) {
    lastTimestampMicroseconds_ = timestampMicroseconds;
    return 0.0;
  }
  
  double timestampSeconds = (double)(timestampMicroseconds - lastTimestampMicroseconds_) / 1E6;
  timestampSeconds = std::min<double>(0.1, timestampSeconds); // make sure it is at most 0.1 seconds

  double output = parameters_.k_p * e + parameters_.k_i * integratedError_ + parameters_.k_d * e_dot;
  // saturate:
  if (output < minOutput_) {
    output = minOutput_; // clamp -- and DO NOT INTEGRATE ERROR (anti-reset windup)
  } else if (output > maxOutput_) {
    output = maxOutput_; // clamp -- and DO NOT INTEGRATE ERROR (anti-reset windup)
  } else {
    integratedError_ += e * timestampSeconds; // safe to keep integrating
  }
  
  lastTimestampMicroseconds_ = timestampMicroseconds;
  return output;
}

// Reset the integrator to zero again.
void PidController::setOutputLimits(double minOutput, double maxOutput)
{
  minOutput_ = minOutput;
  maxOutput_ = maxOutput;
}

/// \brief
void PidController::resetIntegrator()
{
  integratedError_ = 0.0;
}

}  // namespace arp
