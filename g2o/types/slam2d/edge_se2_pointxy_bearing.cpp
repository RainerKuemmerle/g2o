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

#include "edge_se2_pointxy_bearing.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#endif

namespace g2o {

  EdgeSE2PointXYBearing::EdgeSE2PointXYBearing()
  {
  }


  void EdgeSE2PointXYBearing::initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* /*to*/)
  {
    assert(from.size() == 1 && from.count(_vertices[0]) == 1 && "Can not initialize VertexSE2 position by VertexPointXY");

    if (from.count(_vertices[0]) != 1)
      return;
    double r=2.;
    const VertexSE2* v1 = static_cast<const VertexSE2*>(_vertices[0]);
    VertexPointXY* l2 = static_cast<VertexPointXY*>(_vertices[1]);
    SE2 t=v1->estimate();
    t.setRotation(t.rotation().angle()+_measurement);
    Vector2d vr;
    vr[0]=r; vr[1]=0;
    l2->setEstimate(t*vr);
  }

  bool EdgeSE2PointXYBearing::read(std::istream& is)
  {
    is >> _measurement >> information()(0,0);
    return true;
  }

  bool EdgeSE2PointXYBearing::write(std::ostream& os) const
  {
    os << measurement() << " " << information()(0,0);
    return os.good();
  }


  EdgeSE2PointXYBearingWriteGnuplotAction::EdgeSE2PointXYBearingWriteGnuplotAction(): WriteGnuplotAction(typeid(EdgeSE2PointXYBearing).name()){}

  HyperGraphElementAction* EdgeSE2PointXYBearingWriteGnuplotAction::operator()(HyperGraph::HyperGraphElement* element,
                         HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
      return 0;
    WriteGnuplotAction::Parameters* params=static_cast<WriteGnuplotAction::Parameters*>(params_);
    if (!params->os){
      std::cerr << __PRETTY_FUNCTION__ << ": warning, on valid os specified" << std::endl;
      return 0;
    }

    EdgeSE2PointXYBearing* e =  static_cast<EdgeSE2PointXYBearing*>(element);
    VertexSE2* fromEdge = static_cast<VertexSE2*>(e->vertex(0));
    VertexPointXY* toEdge   = static_cast<VertexPointXY*>(e->vertex(1));
    *(params->os) << fromEdge->estimate().translation().x() << " " << fromEdge->estimate().translation().y()
      << " " << fromEdge->estimate().rotation().angle() << std::endl;
    *(params->os) << toEdge->estimate().x() << " " << toEdge->estimate().y() << std::endl;
    *(params->os) << std::endl;
    return this;
  }

#ifdef G2O_HAVE_OPENGL
  EdgeSE2PointXYBearingDrawAction::EdgeSE2PointXYBearingDrawAction(): DrawAction(typeid(EdgeSE2PointXYBearing).name()){}

  HyperGraphElementAction* EdgeSE2PointXYBearingDrawAction::operator()(HyperGraph::HyperGraphElement* element,  HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
      return 0;

    refreshPropertyPtrs(params_);
    if (! _previousParams)
      return this;
    
    if (_show && !_show->value())
      return this;

    EdgeSE2PointXYBearing* e =  static_cast<EdgeSE2PointXYBearing*>(element);
    VertexSE2* fromEdge = static_cast<VertexSE2*>(e->vertex(0));
    VertexPointXY* toEdge   = static_cast<VertexPointXY*>(e->vertex(1));
    glColor3f(0.3f,0.3f,0.1f);
    glBegin(GL_LINES);
    glVertex3f((float)fromEdge->estimate().translation().x(),(float)fromEdge->estimate().translation().y(),0.f);
    glVertex3f((float)toEdge->estimate().x(),(float)toEdge->estimate().y(),0.f);
    glEnd();
    return this;
  }
#endif

} // end namespace
