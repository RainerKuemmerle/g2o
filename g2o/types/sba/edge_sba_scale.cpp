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

#include "edge_sba_scale.h"

namespace g2o {

// point to camera projection, stereo
EdgeSBAScale::EdgeSBAScale()
    : BaseBinaryEdge<1, number_t, VertexCam, VertexCam>() {}

bool EdgeSBAScale::read(std::istream& is) {
  number_t meas;
  is >> meas;
  setMeasurement(meas);
  information().setIdentity();
  is >> information()(0, 0);
  return true;
}

bool EdgeSBAScale::write(std::ostream& os) const {
  os << measurement() << " " << information()(0, 0);
  return os.good();
}

void EdgeSBAScale::initialEstimate(const OptimizableGraph::VertexSet& from_,
                                   OptimizableGraph::Vertex* /*to_*/) {
  VertexCam* v1 = dynamic_cast<VertexCam*>(_vertices[0]);
  VertexCam* v2 = dynamic_cast<VertexCam*>(_vertices[1]);
  // compute the translation vector of v1 w.r.t v2
  if (from_.count(v1) == 1) {
    SE3Quat delta = (v1->estimate().inverse() * v2->estimate());
    number_t norm = delta.translation().norm();
    number_t alpha = _measurement / norm;
    delta.setTranslation(delta.translation() * alpha);
    v2->setEstimate(v1->estimate() * delta);
  } else {
    SE3Quat delta = (v2->estimate().inverse() * v1->estimate());
    number_t norm = delta.translation().norm();
    number_t alpha = _measurement / norm;
    delta.setTranslation(delta.translation() * alpha);
    v1->setEstimate(v2->estimate() * delta);
  }
}

void EdgeSBAScale::computeError() {
  const VertexCam* v1 = dynamic_cast<const VertexCam*>(_vertices[0]);
  const VertexCam* v2 = dynamic_cast<const VertexCam*>(_vertices[1]);
  Vector3 dt = v2->estimate().translation() - v1->estimate().translation();
  _error[0] = _measurement - dt.norm();
}

}  // namespace g2o
