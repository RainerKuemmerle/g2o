// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
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

#ifndef EDGE_SE2_PURE_CALIB_H
#define EDGE_SE2_PURE_CALIB_H

#include "g2o/types/sclam2d/odometry_measurement.h"
#include "g2o/types/sclam2d/vertex_odom_differential_params.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o_calibration_odom_laser_api.h"

namespace g2o {

  struct G2O_CALIBRATION_ODOM_LASER_API OdomAndLaserMotion
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VelocityMeasurement velocityMeasurement;
    SE2 laserMotion;
  };

  /**
   * \brief calibrate odometry and laser based on a set of measurements
   */
  class G2O_CALIBRATION_ODOM_LASER_API EdgeSE2PureCalib : public BaseBinaryEdge<3, OdomAndLaserMotion, VertexSE2, VertexOdomDifferentialParams>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      EdgeSE2PureCalib();

      void computeError();

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;
  };

} // end namespace

#endif
