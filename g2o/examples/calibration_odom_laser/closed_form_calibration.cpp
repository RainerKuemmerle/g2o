// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G.Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "closed_form_calibration.h"

#include "g2o/types/sclam2d/odometry_measurement.h"

#include <iostream>
#include <limits>

#include <Eigen/SVD>

#define SQR(X)		( std::pow(X,2) )
#define CUBE(X)		( std::pow(X,3) )

namespace g2o {

bool ClosedFormCalibration::calibrate(const MotionInformationVector& measurements, SE2& laserOffset, Eigen::Vector3d& odomParams)
{
  std::vector<VelocityMeasurement, Eigen::aligned_allocator<VelocityMeasurement> > velMeasurements;
  for (size_t i = 0; i < measurements.size(); ++i) {
    const SE2& odomMotion = measurements[i].odomMotion;
    const double& timeInterval = measurements[i].timeInterval;
    MotionMeasurement mm(odomMotion.translation().x(), odomMotion.translation().y(), odomMotion.rotation().angle(), timeInterval);
    VelocityMeasurement velMeas = OdomConvert::convertToVelocity(mm);
    velMeasurements.push_back(velMeas);
  }

  double J_21, J_22;
  {
    Eigen::MatrixXd A(measurements.size(), 2);
    Eigen::VectorXd x(measurements.size());
    for (size_t i = 0; i < measurements.size(); ++i) {
      const SE2& laserMotion = measurements[i].laserMotion;
      const double& timeInterval = measurements[i].timeInterval;
      const VelocityMeasurement& velMeas = velMeasurements[i];
      A(i, 0) = velMeas.vl() * timeInterval;
      A(i, 1) = velMeas.vr() * timeInterval;
      x(i) = laserMotion.rotation().angle();
    }
    // (J_21, J_22) = (-r_l / b, r_r / b)
    Eigen::Vector2d linearSolution = (A.transpose() * A).inverse() * A.transpose() * x;
    //std::cout << linearSolution.transpose() << std::endl;
    J_21 = linearSolution(0);
    J_22 = linearSolution(1);
  }

  // construct M
  Eigen::Matrix<double, 5, 5> M;
  M.setZero();
  for (size_t i = 0; i < measurements.size(); ++i) {
    const SE2& laserMotion = measurements[i].laserMotion;
    const double& timeInterval = measurements[i].timeInterval;
    const VelocityMeasurement& velMeas = velMeasurements[i];
    Eigen::Matrix<double, 2, 5> L;
    double omega_o_k = J_21 * velMeas.vl() + J_22 * velMeas.vr();
    double o_theta_k = omega_o_k * timeInterval;
    double sx = 1.;
    double sy = 0.;
    if (fabs(o_theta_k) > std::numeric_limits<double>::epsilon()) {
      sx = sin(o_theta_k)       / o_theta_k;
      sy = (1 - cos(o_theta_k)) / o_theta_k;
    }
    double c_x = 0.5 * timeInterval * (-J_21 * velMeas.vl() + J_22 * velMeas.vr()) * sx;
    double c_y = 0.5 * timeInterval * (-J_21 * velMeas.vl() + J_22 * velMeas.vr()) * sy;
    L(0, 0) = -c_x;
    L(0, 1) = 1 - cos(o_theta_k);
    L(0, 2) = sin(o_theta_k);
    L(0, 3) = laserMotion.translation().x(); 
    L(0, 4) = -laserMotion.translation().y();
    L(1, 0) = -c_y;
    L(1, 1) = - sin(o_theta_k);
    L(1, 2) = 1 - cos(o_theta_k);
    L(1, 3) = laserMotion.translation().y(); 
    L(1, 4) = laserMotion.translation().x();
    M.noalias() += L.transpose() * L;
  }
  //std::cout << M << std::endl;

  // compute lagrange multiplier
  // and solve the constrained least squares problem
  double m11 = M(0,0);
  double m13 = M(0,2);
  double m14 = M(0,3);
  double m15 = M(0,4);
  double m22 = M(1,1);
  double m34 = M(2,3);
  double m35 = M(2,4);
  double m44 = M(3,3); 

  double a = m11 * SQR(m22) - m22 * SQR(m13);
  double b = 2*m11 * SQR(m22) * m44 - SQR(m22) * SQR(m14) - 2*m22 * SQR(m13) * m44 - 2*m11 * m22 * SQR(m34) 
    - 2*m11 * m22 * SQR(m35) - SQR(m22) * SQR(m15) + 2*m13 * m22 * m34 * m14 + SQR(m13) * SQR(m34) 
    + 2*m13 * m22 * m35 * m15 + SQR(m13) * SQR(m35);
  double c = - 2*m13 * CUBE(m35) * m15 - m22 * SQR(m13) * SQR(m44) + m11 * SQR(m22) * SQR(m44) + SQR(m13) * SQR(m35) * m44
    + 2*m13 * m22 * m34 * m14 * m44 + SQR(m13) * SQR(m34) * m44 - 2*m11 * m22 * SQR(m34) * m44 - 2 * m13 * CUBE(m34) * m14
    - 2*m11 * m22 * SQR(m35) * m44 + 2*m11 * SQR(m35) * SQR(m34) + m22 * SQR(m14) * SQR(m35) - 2*m13 * SQR(m35) * m34 * m14
    - 2*m13 * SQR(m34) * m35 * m15 + m11 * std::pow(m34, 4) + m22 * SQR(m15) * SQR(m34) + m22 * SQR(m35) * SQR(m15)
    + m11 * std::pow(m35, 4) - SQR(m22) * SQR(m14) * m44 + 2*m13 * m22 * m35 * m15 * m44 + m22 * SQR(m34) * SQR(m14)
    - SQR(m22) * SQR(m15) * m44;

  // solve the quadratic equation
  double lambda1, lambda2;
  if(a < std::numeric_limits<double>::epsilon()) {
    if(b <= std::numeric_limits<double>::epsilon())
      return false;
    lambda1 = lambda2 = -c/b;
  } else {
    double delta = b*b - 4*a*c;
    if (delta < 0)
      return false;
    lambda1 = 0.5 * (-b-sqrt(delta)) / a;
    lambda2 = 0.5 * (-b+sqrt(delta)) / a;
  }

  Eigen::VectorXd x1 = solveLagrange(M, lambda1);
  Eigen::VectorXd x2 = solveLagrange(M, lambda2);
  double err1 = x1.dot(M * x1);
  double err2 = x2.dot(M * x2);

  const Eigen::VectorXd& calibrationResult = err1 < err2 ? x1 : x2;
  odomParams(0) = - calibrationResult(0) * J_21;
  odomParams(1) = calibrationResult(0) * J_22;
  odomParams(2) = calibrationResult(0);

  laserOffset = SE2(calibrationResult(1), calibrationResult(2), atan2(calibrationResult(4), calibrationResult(3)));

  return true;
}

Eigen::VectorXd ClosedFormCalibration::solveLagrange(const Eigen::Matrix<double,5,5>& M, double lambda)
{
  // A = M * lambda*W (see paper)
  Eigen::Matrix<double,5,5> A;
  A.setZero();
  A(3,3) = A(4,4) = lambda;
  A.noalias() += M;

  // compute the kernel of A by SVD
  Eigen::JacobiSVD< Eigen::Matrix<double,5,5> > svd(A, ComputeFullV);
  Eigen::VectorXd result = svd.matrixV().col(4);
  //for (int i = 0; i < 5; ++i)
  //std::cout << "singular value " << i << " "  << svd.singularValues()(i) << std::endl;
  //std::cout << "kernel base " << result << std::endl;

  // enforce the conditions
  // x_1 > 0
  if (result(0) < 0.)
    result *= -1;
  // x_4^2 + x_5^2 = 1
  double scale = sqrt(pow(result(3), 2) + pow(result(4), 2));
  result /= scale;

  return result;
}

} // end namespace
