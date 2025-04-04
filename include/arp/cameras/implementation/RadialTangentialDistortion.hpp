/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * 
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Feb 3, 2015
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file implementation/RadialTangentialDistortion.hpp
 * @brief Header implementation file for the RadialTangentialDistortion class.
 * @author Stefan Leutenegger
 */


#include <Eigen/LU>
#include <iostream>
#include <stdexcept>
#include <math.h>

/// \brief arp Main namespace of this package.
namespace arp {
/// \brief cameras Namespace for camera-related functionality.
namespace cameras {

// The default constructor with all zero ki
RadialTangentialDistortion::RadialTangentialDistortion()
    : k1_(0.0),
      k2_(0.0),
      p1_(0.0),
      p2_(0.0)
{
}

// Constructor initialising ki
RadialTangentialDistortion::RadialTangentialDistortion(double k1, double k2,
                                                       double p1, double p2)
{
  k1_ = k1;
  k2_ = k2;
  p1_ = p1;
  p2_ = p2;
}

bool RadialTangentialDistortion::distort(
    const Eigen::Vector2d & pointUndistorted,
    Eigen::Vector2d * pointDistorted) const
{
  double r = sqrt(pow(pointUndistorted.x(), 2) + pow(pointUndistorted.y(), 2));

  double x = pointUndistorted.x();
  double y = pointUndistorted.y();

  pointDistorted->x() =  (1 + k1_*pow(r,2) + k2_*pow(r,4)) * x + 2*this->p1_*x*y + this->p2_*(pow(r,2) + 2*pow(x,2));
  pointDistorted->y() =  (1 + k1_*pow(r,2) + k2_*pow(r,4)) * y + this->p1_*(pow(r,2) + 2*pow(y,2)) + 2*this->p2_*x*y;

  return true;
}

bool RadialTangentialDistortion::distort(
    const Eigen::Vector2d & pointUndistorted, Eigen::Vector2d * pointDistorted,
    Eigen::Matrix2d * pointJacobian) const
{
  double x = pointUndistorted.x();
  double y = pointUndistorted.y();

  this->distort(pointUndistorted, pointDistorted);

  *pointJacobian << 5*k2_*pow(x,4) + 3 * k1_*pow(x,2) + 6*pow(y,2)*k2_*pow(x,2) + k1_*pow(y, 2) + k2_*pow(y, 4) + 1 + 2*y*p1_ + 6*p2_*x,
                  x*(2*k1_*y + 4*k2_*y*(pow(x,2) + pow(y,2))) + 2*x*p1_ + 2*p2_*y,
                 y * (2 * k1_ * x + 4 * k2_ * x * (pow(x, 2) + pow(y, 2))) + 2 * p1_ * x + 2 * p2_ * y,
                   5 * k2_ * pow(y, 4) + 3 * k1_ * pow(y, 2) + 6 * pow(x, 2) * k2_ * pow(y, 2) + k1_ * pow(x, 2) + pow(x, 4) * k2_ + 1 + 6 * p1_ * y + 2 * p2_ * x;
  
  return true;
}

bool RadialTangentialDistortion::undistort(
    const Eigen::Vector2d & pointDistorted,
    Eigen::Vector2d * pointUndistorted) const
{
  // this is expensive: we solve with Gauss-Newton...
  Eigen::Vector2d x_bar = pointDistorted; // initialise at distorted point
  const int n = 5;  // just 5 iterations max.
  Eigen::Matrix2d E;  // error Jacobian

  bool success = false;
  for (int i = 0; i < n; i++) {

    Eigen::Vector2d x_tmp;

    distort(x_bar, &x_tmp, &E);

    Eigen::Vector2d e(pointDistorted - x_tmp);
    Eigen::Matrix2d E2 = (E.transpose() * E);
    Eigen::Vector2d du = E2.inverse() * E.transpose() * e;

    x_bar += du;

    const double chi2 = e.dot(e);
    if (chi2 < 1e-4) {
      success = true;
    }
    if (chi2 < 1e-15) {
      success = true;
      break;
    }

  }
  *pointUndistorted = x_bar;

  return success;
}

}  // namespace cameras
}  // namespace arp
