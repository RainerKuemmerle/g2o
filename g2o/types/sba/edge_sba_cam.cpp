// g2o - General Graph Optimization
// Copyright (C) 2011 Kurt Konolige
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

#include "edge_sba_cam.h"

namespace g2o {

bool EdgeSBACam::read(std::istream& is) {
  Vector7 meas;
  internal::readVector(is, meas);
  setMeasurement(SE3Quat(meas));
  return readInformationMatrix(is);
}

bool EdgeSBACam::write(std::ostream& os) const {
  internal::writeVector(os, measurement().toVector());
  return writeInformationMatrix(os);
}

void EdgeSBACam::initialEstimate(const OptimizableGraph::VertexSet& from_,
                                 OptimizableGraph::Vertex* /*to_*/) {
  auto from = vertexXn<0>();
  auto to = vertexXn<1>();
  if (from_.count(from) > 0)
    to->setEstimate(SBACam(SE3Quat(from->estimate()) * measurement_));
  else
    from->setEstimate(SBACam(SE3Quat(to->estimate()) * inverseMeasurement_));
}

bool EdgeSBACam::setMeasurementFromState() {
  const VertexCam* v1 = vertexXnRaw<0>();
  const VertexCam* v2 = vertexXnRaw<1>();
  measurement_ = (v1->estimate().inverse() * v2->estimate());
  inverseMeasurement_ = measurement_.inverse();
  return true;
}

void EdgeSBACam::setMeasurement(const SE3Quat& meas) {
  measurement_ = meas;
  inverseMeasurement_ = meas.inverse();
}

bool EdgeSBACam::setMeasurementData(const number_t* d) {
  Eigen::Map<const Vector7> v(d);
  measurement_.fromVector(v);
  inverseMeasurement_ = measurement_.inverse();
  return true;
}

bool EdgeSBACam::getMeasurementData(number_t* d) const {
  Eigen::Map<Vector7> v(d);
  v = measurement_.toVector();
  return true;
}

void EdgeSBACam::computeError() {
  const VertexCam* v1 = vertexXnRaw<0>();
  const VertexCam* v2 = vertexXnRaw<1>();
  SE3Quat delta =
      inverseMeasurement_ * (v1->estimate().inverse() * v2->estimate());
  error_[0] = delta.translation().x();
  error_[1] = delta.translation().y();
  error_[2] = delta.translation().z();
  error_[3] = delta.rotation().x();
  error_[4] = delta.rotation().y();
  error_[5] = delta.rotation().z();
}

}  // namespace g2o
