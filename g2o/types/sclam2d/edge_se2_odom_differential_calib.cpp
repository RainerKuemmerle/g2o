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

#include "edge_se2_odom_differential_calib.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#endif

namespace g2o {

bool EdgeSE2OdomDifferentialCalib::read(std::istream& is) {
  number_t vl;
  number_t vr;
  number_t dt;
  is >> vl >> vr >> dt;
  VelocityMeasurement vm(vl, vr, dt);
  setMeasurement(vm);
  return readInformationMatrix(is);
}

bool EdgeSE2OdomDifferentialCalib::write(std::ostream& os) const {
  os << measurement().vl() << " " << measurement().vr() << " "
     << measurement().dt() << " ";
  return writeInformationMatrix(os);
}

#ifdef G2O_HAVE_OPENGL
EdgeSE2OdomDifferentialCalibDrawAction::EdgeSE2OdomDifferentialCalibDrawAction()
    : DrawAction(typeid(EdgeSE2OdomDifferentialCalib).name()) {}

bool EdgeSE2OdomDifferentialCalibDrawAction::operator()(
    HyperGraph::HyperGraphElement& element,
    const std::shared_ptr<HyperGraphElementAction::Parameters>&) {
  if (typeid(element).name() != typeName_) return false;
  auto* e = static_cast<EdgeSE2OdomDifferentialCalib*>(&element);
  auto fromEdge = e->vertexXn<0>();
  auto toEdge = e->vertexXn<1>();
  if (!fromEdge || !toEdge) return true;
  glColor3f(0.5F, 0.5F, 0.5F);
  glPushAttrib(GL_ENABLE_BIT);
  glDisable(GL_LIGHTING);
  glBegin(GL_LINES);
  glVertex3f(static_cast<float>(fromEdge->estimate().translation().x()),
             static_cast<float>(fromEdge->estimate().translation().y()), 0.F);
  glVertex3f(static_cast<float>(toEdge->estimate().translation().x()),
             static_cast<float>(toEdge->estimate().translation().y()), 0.F);
  glEnd();
  glPopAttrib();
  return true;
}
#endif

}  // namespace g2o
