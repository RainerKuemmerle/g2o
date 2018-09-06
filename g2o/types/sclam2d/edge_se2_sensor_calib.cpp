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

#include "edge_se2_sensor_calib.h"
#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#endif
namespace g2o {

  EdgeSE2SensorCalib::EdgeSE2SensorCalib() :
    BaseMultiEdge<3, SE2>()
  {
    resize(3);
  }

  void EdgeSE2SensorCalib::initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to)
  {
    (void) to;
    VertexSE2* vi = static_cast<VertexSE2*>(_vertices[0]);
    VertexSE2* vj = static_cast<VertexSE2*>(_vertices[1]);
    VertexSE2* l  = static_cast<VertexSE2*>(_vertices[2]);
    if (from.count(l) == 0)
      return;
    if (from.count(vi) == 1) {
      vj->setEstimate(vi->estimate() * l->estimate() * measurement() * l->estimate().inverse());
    } else {
      vi->setEstimate(vj->estimate() * l->estimate() * _inverseMeasurement * l->estimate().inverse());
    }
  }

  bool EdgeSE2SensorCalib::read(std::istream& is)
  {
    Vector3 p;
    is >> p(0) >> p(1) >> p(2);
    _measurement.fromVector(p);
    _inverseMeasurement=measurement().inverse();
    for (int i = 0; i < information().rows(); ++i)
      for (int j = i; j < information().cols(); ++j) {
        is >> information()(i, j);
        if (i != j)
          information()(j, i) = information()(i, j);
      }
    return true;
  }

  bool EdgeSE2SensorCalib::write(std::ostream& os) const
  {
    Vector3 p = measurement().toVector();
    os << p(0) << " " << p(1) << " " << p(2);
    for (int i = 0; i < information().rows(); ++i)
      for (int j = i; j < information().cols(); ++j)
        os << " " << information()(i, j);
    return os.good();
  }

#ifdef G2O_HAVE_OPENGL
  EdgeSE2SensorCalibDrawAction::EdgeSE2SensorCalibDrawAction() : 
    DrawAction(typeid(EdgeSE2SensorCalib).name())
  {
  }

  HyperGraphElementAction* EdgeSE2SensorCalibDrawAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* )
  {
    if (typeid(*element).name()!=_typeName)
      return nullptr;
    EdgeSE2SensorCalib* e = static_cast<EdgeSE2SensorCalib*>(element);
    VertexSE2* fromEdge = static_cast<VertexSE2*>(e->vertex(0));
    VertexSE2* toEdge   = static_cast<VertexSE2*>(e->vertex(1));
    glColor3f(0.5,0.5,1.0);
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
