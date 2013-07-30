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

#ifndef G2O_CLOSED_FORM_CALIBRATION_H
#define G2O_CLOSED_FORM_CALIBRATION_H

#include <Eigen/Core>
#include "motion_information.h"
#include "g2o_calibration_odom_laser_api.h"

namespace g2o {

  /**
   * \brief Simultaneous calibration of the laser offest and the parameters of a differential drive
   *
   * Approach described by Censi et al.
   * Andrea Censi, Luca Marchionni, and Giuseppe Oriolo.
   * Simultaneous maximum-likelihood calibration of robot and sensor parameters.
   * In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA). Pasadena, CA, May 2008.
   */
  class G2O_CALIBRATION_ODOM_LASER_API ClosedFormCalibration
  {
    public:
      static bool calibrate(const MotionInformationVector& measurements, SE2& laserOffset, Eigen::Vector3d& odomParams);

    protected:
      static Eigen::VectorXd solveLagrange(const Eigen::Matrix<double,5,5>& M, double lambda);
  };

} // end namespace

#endif
