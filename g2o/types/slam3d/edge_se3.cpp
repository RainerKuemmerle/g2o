// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, H. Strasdat, W. Burgard
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

#include "edge_se3.h"

#include <iostream>

#include "isometry3d_gradients.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_primitives.h"
#include "g2o/stuff/opengl_wrapper.h"
#endif

namespace g2o {

EdgeSE3::EdgeSE3() { information().setIdentity(); }

bool EdgeSE3::read(std::istream& is) {
  Vector7 meas;
  internal::readVector(is, meas);
  // normalize the quaternion to recover numerical precision lost by storing as
  // human readable text
  Vector4::MapType(meas.data() + 3).normalize();
  setMeasurement(internal::fromVectorQT(meas));
  if (is.bad()) return false;
  readInformationMatrix(is);
  return is.good() || is.eof();
}

bool EdgeSE3::write(std::ostream& os) const {
  internal::writeVector(os, internal::toVectorQT(measurement()));
  return writeInformationMatrix(os);
}

void EdgeSE3::computeError() {
  VertexSE3* from = vertexXnRaw<0>();
  VertexSE3* to = vertexXnRaw<1>();
  Isometry3 delta =
      inverseMeasurement_ * from->estimate().inverse() * to->estimate();
  error_ = internal::toVectorMQT(delta);
}

bool EdgeSE3::setMeasurementFromState() {
  VertexSE3* from = vertexXnRaw<0>();
  VertexSE3* to = vertexXnRaw<1>();
  Isometry3 delta = from->estimate().inverse() * to->estimate();
  setMeasurement(delta);
  return true;
}

void EdgeSE3::linearizeOplus() {
  VertexSE3* from = vertexXnRaw<0>();
  VertexSE3* to = vertexXnRaw<1>();
  Isometry3 E;
  const Isometry3& Xi = from->estimate();
  const Isometry3& Xj = to->estimate();
  const Isometry3& Z = measurement_;
  internal::computeEdgeSE3Gradient(E, jacobianOplusXi_, jacobianOplusXj_, Z, Xi,
                                   Xj);
}

void EdgeSE3::initialEstimate(const OptimizableGraph::VertexSet& from_,
                              OptimizableGraph::Vertex* /*to_*/) {
  VertexSE3* from = vertexXnRaw<0>();
  VertexSE3* to = vertexXnRaw<1>();

  if (from_.count(vertexXn<0>()) > 0) {
    to->setEstimate(from->estimate() * measurement_);
  } else
    from->setEstimate(to->estimate() * measurement_.inverse());
  // cerr << "IE" << endl;
}

EdgeSE3WriteGnuplotAction::EdgeSE3WriteGnuplotAction()
    : WriteGnuplotAction(typeid(EdgeSE3).name()) {}

bool EdgeSE3WriteGnuplotAction::operator()(
    HyperGraph::HyperGraphElement& element,
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params_) {
  if (typeid(element).name() != typeName_) return false;
  auto* params = static_cast<WriteGnuplotAction::Parameters*>(params_.get());
  if (!params->os) {
    std::cerr << __PRETTY_FUNCTION__ << ": warning, on valid os specified"
              << std::endl;
    return false;
  }

  auto* e = static_cast<EdgeSE3*>(&element);
  auto* fromEdge = static_cast<VertexSE3*>(e->vertices()[0].get());
  auto* toEdge = static_cast<VertexSE3*>(e->vertices()[1].get());
  Vector6 fromV;
  Vector6 toV;
  fromV = internal::toVectorMQT(fromEdge->estimate());
  toV = internal::toVectorMQT(toEdge->estimate());
  for (int i = 0; i < 6; i++) {
    *(params->os) << fromV[i] << " ";
  }
  for (int i = 0; i < 6; i++) {
    *(params->os) << toV[i] << " ";
  }
  *(params->os) << std::endl;
  return true;
}

#ifdef G2O_HAVE_OPENGL
EdgeSE3DrawAction::EdgeSE3DrawAction() : DrawAction(typeid(EdgeSE3).name()) {}

bool EdgeSE3DrawAction::operator()(
    HyperGraph::HyperGraphElement& element,
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params_) {
  if (typeid(element).name() != typeName_) return false;
  refreshPropertyPtrs(params_);
  if (!previousParams_) return true;

  if (show_ && !show_->value()) return true;

  auto* e = static_cast<EdgeSE3*>(&element);
  auto* fromEdge = static_cast<VertexSE3*>(e->vertices()[0].get());
  auto* toEdge = static_cast<VertexSE3*>(e->vertices()[1].get());
  if (!fromEdge || !toEdge) return true;
  glColor3f(POSE_EDGE_COLOR);
  glPushAttrib(GL_ENABLE_BIT);
  glDisable(GL_LIGHTING);
  glBegin(GL_LINES);
  glVertex3f(static_cast<float>(fromEdge->estimate().translation().x()),
             static_cast<float>(fromEdge->estimate().translation().y()),
             static_cast<float>(fromEdge->estimate().translation().z()));
  glVertex3f(static_cast<float>(toEdge->estimate().translation().x()),
             static_cast<float>(toEdge->estimate().translation().y()),
             static_cast<float>(toEdge->estimate().translation().z()));
  glEnd();
  glPopAttrib();
  return true;
}
#endif

}  // namespace g2o
