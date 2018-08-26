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

  EdgeSE2OdomDifferentialCalib::EdgeSE2OdomDifferentialCalib() :
    BaseMultiEdge<3, VelocityMeasurement>()
  {
    resize(3);
  }

  bool EdgeSE2OdomDifferentialCalib::read(std::istream& is)
  {
    number_t vl, vr, dt;
    is >> vl >> vr >> dt;
    VelocityMeasurement vm(vl, vr, dt);
    setMeasurement(vm);
    for (int i = 0; i < information().rows(); ++i)
      for (int j = i; j < information().cols(); ++j) {
        is >> information()(i, j);
        if (i != j)
          information()(j, i) = information()(i, j);
      }
    return true;
  }

  bool EdgeSE2OdomDifferentialCalib::write(std::ostream& os) const
  {
    os << measurement().vl() << " " << measurement().vr() << " " << measurement().dt();
    for (int i = 0; i < information().rows(); ++i)
      for (int j = i; j < information().cols(); ++j)
        os << " " << information()(i, j);
    return os.good();
  }

#ifdef G2O_HAVE_OPENGL
  EdgeSE2OdomDifferentialCalibDrawAction::EdgeSE2OdomDifferentialCalibDrawAction() :
    DrawAction(typeid(EdgeSE2OdomDifferentialCalib).name())
  {
  }

  HyperGraphElementAction* EdgeSE2OdomDifferentialCalibDrawAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* )
  {
    if (typeid(*element).name()!=_typeName)
      return nullptr;
    EdgeSE2OdomDifferentialCalib* e = static_cast<EdgeSE2OdomDifferentialCalib*>(element);
    VertexSE2* fromEdge = static_cast<VertexSE2*>(e->vertex(0));
    VertexSE2* toEdge   = static_cast<VertexSE2*>(e->vertex(1));
    glColor3f(0.5f,0.5f,0.5f);
    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glVertex3f((float)fromEdge->estimate().translation().x(),(float)fromEdge->estimate().translation().y(),0.f);
    glVertex3f((float)toEdge->estimate().translation().x(),(float)toEdge->estimate().translation().y(),0.f);
    glEnd();
    glPopAttrib();
    return this;
  }
#endif

} // end namespace
