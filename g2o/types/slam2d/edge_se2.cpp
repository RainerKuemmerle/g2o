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

#include "edge_se2.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#include "g2o/stuff/opengl_primitives.h"
#endif

namespace g2o {

  EdgeSE2::EdgeSE2() :
    BaseBinaryEdge<3, SE2, VertexSE2, VertexSE2>()
  {
  }

  bool EdgeSE2::read(std::istream& is)
  {
    Vector3D p;
    is >> p[0] >> p[1] >> p[2];
    setMeasurement(SE2(p));
    _inverseMeasurement = measurement().inverse();
    for (int i = 0; i < 3; ++i)
      for (int j = i; j < 3; ++j) {
        is >> information()(i, j);
        if (i != j)
          information()(j, i) = information()(i, j);
      }
    return true;
  }

  bool EdgeSE2::write(std::ostream& os) const
  {
    Vector3D p = measurement().toVector();
    os << p.x() << " " << p.y() << " " << p.z();
    for (int i = 0; i < 3; ++i)
      for (int j = i; j < 3; ++j)
        os << " " << information()(i, j);
    return os.good();
  }

  void EdgeSE2::initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* /* to */)
  {
    VertexSE2* fromEdge = static_cast<VertexSE2*>(_vertices[0]);
    VertexSE2* toEdge   = static_cast<VertexSE2*>(_vertices[1]);
    if (from.count(fromEdge) > 0)
      toEdge->setEstimate(fromEdge->estimate() * _measurement);
    else
      fromEdge->setEstimate(toEdge->estimate() * _inverseMeasurement);
  }

#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
  void EdgeSE2::linearizeOplus()
  {
    const VertexSE2* vi = static_cast<const VertexSE2*>(_vertices[0]);
    const VertexSE2* vj = static_cast<const VertexSE2*>(_vertices[1]);
    double thetai = vi->estimate().rotation().angle();

    Vector2D dt = vj->estimate().translation() - vi->estimate().translation();
    double si=sin(thetai), ci=cos(thetai);

    _jacobianOplusXi(0, 0) = -ci; _jacobianOplusXi(0, 1) = -si; _jacobianOplusXi(0, 2) = -si*dt.x()+ci*dt.y();
    _jacobianOplusXi(1, 0) =  si; _jacobianOplusXi(1, 1) = -ci; _jacobianOplusXi(1, 2) = -ci*dt.x()-si*dt.y();
    _jacobianOplusXi(2, 0) =  0;  _jacobianOplusXi(2, 1) = 0;   _jacobianOplusXi(2, 2) = -1;

    _jacobianOplusXj(0, 0) = ci; _jacobianOplusXj(0, 1)= si; _jacobianOplusXj(0, 2)= 0;
    _jacobianOplusXj(1, 0) =-si; _jacobianOplusXj(1, 1)= ci; _jacobianOplusXj(1, 2)= 0;
    _jacobianOplusXj(2, 0) = 0;  _jacobianOplusXj(2, 1)= 0;  _jacobianOplusXj(2, 2)= 1;

    const SE2& rmean = _inverseMeasurement;
    Matrix3D z = Matrix3D::Zero();
    z.block<2, 2>(0, 0) = rmean.rotation().toRotationMatrix();
    z(2, 2) = 1.;
    _jacobianOplusXi = z * _jacobianOplusXi;
    _jacobianOplusXj = z * _jacobianOplusXj;
  }
#endif

  EdgeSE2WriteGnuplotAction::EdgeSE2WriteGnuplotAction(): WriteGnuplotAction(typeid(EdgeSE2).name()){}

  HyperGraphElementAction* EdgeSE2WriteGnuplotAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
      return 0;
    WriteGnuplotAction::Parameters* params=static_cast<WriteGnuplotAction::Parameters*>(params_);
    if (!params->os){
      std::cerr << __PRETTY_FUNCTION__ << ": warning, on valid os specified" << std::endl;
      return 0;
    }

    EdgeSE2* e =  static_cast<EdgeSE2*>(element);
    VertexSE2* fromEdge = static_cast<VertexSE2*>(e->vertex(0));
    VertexSE2* toEdge   = static_cast<VertexSE2*>(e->vertex(1));
    *(params->os) << fromEdge->estimate().translation().x() << " " << fromEdge->estimate().translation().y()
      << " " << fromEdge->estimate().rotation().angle() << std::endl;
    *(params->os) << toEdge->estimate().translation().x() << " " << toEdge->estimate().translation().y()
      << " " << toEdge->estimate().rotation().angle() << std::endl;
    *(params->os) << std::endl;
    return this;
  }

#ifdef G2O_HAVE_OPENGL
  EdgeSE2DrawAction::EdgeSE2DrawAction(): DrawAction(typeid(EdgeSE2).name()){}

  bool EdgeSE2DrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_){
    if (!DrawAction::refreshPropertyPtrs(params_))
      return false;
    if (_previousParams){
      _triangleX = _previousParams->makeProperty<FloatProperty>(_typeName + "::GHOST_TRIANGLE_X", .2f);
      _triangleY = _previousParams->makeProperty<FloatProperty>(_typeName + "::GHOST_TRIANGLE_Y", .05f);
    } else {
      _triangleX = 0;
      _triangleY = 0;
    }
    return true;
  }

  HyperGraphElementAction* EdgeSE2DrawAction::operator()(HyperGraph::HyperGraphElement* element, 
               HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
      return 0;

    refreshPropertyPtrs(params_);
    if (! _previousParams)
      return this;
    
    if (_show && !_show->value())
      return this;

    EdgeSE2* e =  static_cast<EdgeSE2*>(element);
    VertexSE2* from = static_cast<VertexSE2*>(e->vertex(0));
    VertexSE2* to   = static_cast<VertexSE2*>(e->vertex(1));
    if (! from && ! to)
      return this;
    SE2 fromTransform;
    SE2 toTransform;
    glPushAttrib(GL_ENABLE_BIT | GL_LIGHTING | GL_COLOR);
    glDisable(GL_LIGHTING);
    if (! from) {
      glColor3f(POSE_EDGE_GHOST_COLOR);
      toTransform = to->estimate();
      fromTransform = to->estimate()*e->measurement().inverse();
      // DRAW THE FROM EDGE AS AN ARROW
      glPushMatrix();
      glTranslatef((float)fromTransform.translation().x(), (float)fromTransform.translation().y(),0.f);
      glRotatef((float)RAD2DEG(fromTransform.rotation().angle()),0.f,0.f,1.f);
      opengl::drawArrow2D((float)_triangleX->value(), (float)_triangleY->value(), (float)_triangleX->value()*.3f);
      glPopMatrix();
    } else if (! to){
      glColor3f(POSE_EDGE_GHOST_COLOR);
      fromTransform = from->estimate();
      toTransform = from->estimate()*e->measurement();
      // DRAW THE TO EDGE AS AN ARROW
      glPushMatrix();
      glTranslatef(toTransform.translation().x(),toTransform.translation().y(),0.f);
      glRotatef((float)RAD2DEG(toTransform.rotation().angle()),0.f,0.f,1.f);
      opengl::drawArrow2D((float)_triangleX->value(), (float)_triangleY->value(), (float)_triangleX->value()*.3f);
      glPopMatrix();
    } else {
      glColor3f(POSE_EDGE_COLOR);
      fromTransform = from->estimate();
      toTransform = to->estimate();
    }
    glBegin(GL_LINES);
    glVertex3f((float)fromTransform.translation().x(),(float)fromTransform.translation().y(),0.f);
    glVertex3f((float)toTransform.translation().x(),(float)toTransform.translation().y(),0.f);
    glEnd();
    glPopAttrib();
    return this;
  }
#endif

} // end namespace
