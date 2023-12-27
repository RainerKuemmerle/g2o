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

#include "edge_se2_offset.h"

#include <Eigen/Core>

#include "g2o/core/eigen_types.h"
#include "g2o/types/slam2d/se2.h"
#include "parameter_se2_offset.h"

namespace g2o {

EdgeSE2Offset::EdgeSE2Offset() {
  information().setIdentity();
  resizeParameters(2);
  installParameter<CacheSE2Offset::ParameterType>(0);
  installParameter<CacheSE2Offset::ParameterType>(1);
}

bool EdgeSE2Offset::resolveCaches() {
  ParameterVector pv(1);
  pv[0] = parameters_[0];
  cacheFrom_ =
      resolveCache<CacheSE2Offset>(vertexXn<0>(), "CACHE_SE2_OFFSET", pv);
  pv[0] = parameters_[1];
  cacheTo_ =
      resolveCache<CacheSE2Offset>(vertexXn<1>(), "CACHE_SE2_OFFSET", pv);
  return (cacheFrom_ && cacheTo_);
}

void EdgeSE2Offset::computeError() {
  SE2 delta = inverseMeasurement_ * cacheFrom_->w2n() * cacheTo_->n2w();
  error_.head<2>() = delta.translation();
  error_(2) = delta.rotation().angle();
}

bool EdgeSE2Offset::setMeasurementFromState() {
  SE2 delta = cacheFrom_->w2n() * cacheTo_->n2w();
  setMeasurement(delta);
  return true;
}

void EdgeSE2Offset::initialEstimate(const OptimizableGraph::VertexSet& from_,
                                    OptimizableGraph::Vertex* /*to_*/) {
  auto from = vertexXn<0>();
  auto to = vertexXn<1>();

  SE2 virtualMeasurement = cacheFrom_->offsetParam()->param() * measurement() *
                           cacheTo_->offsetParam()->param().inverse();

  if (from_.count(from) > 0)
    to->setEstimate(from->estimate() * virtualMeasurement);
  else
    from->setEstimate(to->estimate() * virtualMeasurement.inverse());
}

}  // namespace g2o
