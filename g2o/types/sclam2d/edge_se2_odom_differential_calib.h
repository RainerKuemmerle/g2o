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

#ifndef G2O_EDGE_SE2_ODOM_CALIB_DIFFERENTIAL_H
#define G2O_EDGE_SE2_ODOM_CALIB_DIFFERENTIAL_H

#include "g2o/core/base_fixed_sized_edge.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o_types_sclam2d_api.h"
#include "odometry_measurement.h"
#include "vertex_odom_differential_params.h"

namespace g2o {

class G2O_TYPES_SCLAM2D_API EdgeSE2OdomDifferentialCalib
    : public BaseFixedSizedEdge<3, VelocityMeasurement, VertexSE2, VertexSE2,
                                VertexOdomDifferentialParams> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  void computeError() override {
    const VertexSE2* v1 = vertexXnRaw<0>();
    const VertexSE2* v2 = vertexXnRaw<1>();
    const VertexOdomDifferentialParams* params = vertexXnRaw<2>();
    const SE2& x1 = v1->estimate();
    const SE2& x2 = v2->estimate();

    // get the calibrated motion given by the odometry
    VelocityMeasurement calibratedVelocityMeasurement(
        measurement().vl() * params->estimate()(0),
        measurement().vr() * params->estimate()(1), measurement().dt());
    MotionMeasurement mm = OdomConvert::convertToMotion(
        calibratedVelocityMeasurement, params->estimate()(2));
    SE2 Ku_ij;
    Ku_ij.fromVector(mm.measurement());

    SE2 delta = Ku_ij.inverse() * x1.inverse() * x2;
    error_ = delta.toVector();
  }

  bool read(std::istream& is) override;
  bool write(std::ostream& os) const override;
};

#ifdef G2O_HAVE_OPENGL
class G2O_TYPES_SCLAM2D_API EdgeSE2OdomDifferentialCalibDrawAction
    : public DrawAction {
 public:
  EdgeSE2OdomDifferentialCalibDrawAction();
  bool operator()(HyperGraph::HyperGraphElement& element,
                  const std::shared_ptr<HyperGraphElementAction::Parameters>&
                      params_) override;
};
#endif

}  // namespace g2o

#endif
