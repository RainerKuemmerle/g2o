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

EdgeSBACam::EdgeSBACam() : BaseBinaryEdge<6, SE3Quat, VertexCam, VertexCam>() {}

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
  VertexCam* from = static_cast<VertexCam*>(_vertices[0]);
  VertexCam* to = static_cast<VertexCam*>(_vertices[1]);
  if (from_.count(from) > 0)
    to->setEstimate((SE3Quat)from->estimate() * _measurement);
  else
    from->setEstimate((SE3Quat)to->estimate() * _inverseMeasurement);
}

bool EdgeSBACam::setMeasurementFromState() {
  const VertexCam* v1 = dynamic_cast<const VertexCam*>(_vertices[0]);
  const VertexCam* v2 = dynamic_cast<const VertexCam*>(_vertices[1]);
  _measurement = (v1->estimate().inverse() * v2->estimate());
  _inverseMeasurement = _measurement.inverse();
  return true;
}

void EdgeSBACam::setMeasurement(const SE3Quat& meas) {
  _measurement = meas;
  _inverseMeasurement = meas.inverse();
}

bool EdgeSBACam::setMeasurementData(const number_t* d) {
  Eigen::Map<const Vector7> v(d);
  _measurement.fromVector(v);
  _inverseMeasurement = _measurement.inverse();
  return true;
}

bool EdgeSBACam::getMeasurementData(number_t* d) const {
  Eigen::Map<Vector7> v(d);
  v = _measurement.toVector();
  return true;
}

void EdgeSBACam::computeError() {
  const VertexCam* v1 = dynamic_cast<const VertexCam*>(_vertices[0]);
  const VertexCam* v2 = dynamic_cast<const VertexCam*>(_vertices[1]);
  SE3Quat delta = _inverseMeasurement * (v1->estimate().inverse() * v2->estimate());
  _error[0] = delta.translation().x();
  _error[1] = delta.translation().y();
  _error[2] = delta.translation().z();
  _error[3] = delta.rotation().x();
  _error[4] = delta.rotation().y();
  _error[5] = delta.rotation().z();
}

}  // namespace g2o
