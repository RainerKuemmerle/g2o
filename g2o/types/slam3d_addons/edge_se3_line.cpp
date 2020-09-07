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

#include "edge_se3_line.h"

#include "g2o/stuff/opengl_wrapper.h"

namespace g2o {

  EdgeSE3Line3D::EdgeSE3Line3D() {
    information().setIdentity();
    cache = 0;
    offsetParam = 0;
    resizeParameters(1);
    installParameter(offsetParam, 0);
    color << 0.0, 0.5, 1.0;
  }

  bool EdgeSE3Line3D::read(std::istream& is) {
    bool state = readParamIds(is);
    state &= internal::readVector(is, _measurement);
    state &= readInformationMatrix(is);
    return state;
  }

  bool EdgeSE3Line3D::write(std::ostream& os) const {
    writeParamIds(os);
    internal::writeVector(os, measurement());
    return writeInformationMatrix(os);
  }

  void EdgeSE3Line3D::computeError() {
    const VertexSE3* se3Vertex = static_cast<const VertexSE3*>(_vertices[0]);
    const VertexLine3D* lineVertex = static_cast<const VertexLine3D*>(_vertices[1]);
    const Line3D& line = lineVertex->estimate();
    Line3D localLine = se3Vertex->estimate().inverse() * line;
    _error = localLine.ominus(_measurement);
  }

  bool EdgeSE3Line3D::resolveCaches() {
    ParameterVector pv(1);
    pv[0] = offsetParam;
    resolveCache(cache, (OptimizableGraph::Vertex*)_vertices[0], "CACHE_SE3_OFFSET", pv);
    return cache != 0;
  }

#ifdef G2O_HAVE_OPENGL
  EdgeSE3Line3DDrawAction::EdgeSE3Line3DDrawAction()
      : DrawAction(typeid(EdgeSE3Line3D).name()), _lineLength(nullptr), _lineWidth(nullptr) {}

  bool EdgeSE3Line3DDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_) {
    if(!DrawAction::refreshPropertyPtrs(params_)) {
      return false;
    }
    if(_previousParams) {
      _lineLength = _previousParams->makeProperty<FloatProperty>(_typeName + "::LINE_LENGTH", 4.0f);
      _lineWidth = _previousParams->makeProperty<FloatProperty>(_typeName + "::LINE_WIDTH", 2.0f);
    }
    else {
      _lineLength = 0;
      _lineWidth = 0;
    }
    return true;
  }

  HyperGraphElementAction* EdgeSE3Line3DDrawAction::operator()(HyperGraph::HyperGraphElement* element,
							       HyperGraphElementAction::Parameters* params_) {
    if(typeid(*element).name() != _typeName)
      return nullptr;

    refreshPropertyPtrs(params_);
    if(!_previousParams)
      return this;


    if(_show && !_show->value())
      return this;


    EdgeSE3Line3D* that = dynamic_cast<EdgeSE3Line3D*>(element);

    if(!that)
      return this;


    const VertexSE3* robot  = dynamic_cast<const VertexSE3*>(that->vertex(0));
    const VertexLine3D* landmark = dynamic_cast<const VertexLine3D*>(that->vertex(1));

    if(!robot || !landmark)
      return nullptr;

    if (_lineLength && _lineWidth) {
      Line3D line = that->measurement();
      line.normalize();
      Vector3 direction = line.d();
      Vector3 npoint = line.d().cross(line.w());

      glPushMatrix();
      glMultMatrixd(robot->estimate().matrix().cast<double>().eval().data());
      glColor3f(float(that->color(0)), float(that->color(1)), float(that->color(2)));
      glLineWidth(float(_lineWidth->value()));
      glBegin(GL_LINES);
      glNormal3f(float(npoint.x()), float(npoint.y()), float(npoint.z()));
      glVertex3f(float(npoint.x() - direction.x() * _lineLength->value() / 2),
                 float(npoint.y() - direction.y() * _lineLength->value() / 2),
                 float(npoint.z() - direction.z() * _lineLength->value() / 2));
      glVertex3f(float(npoint.x() + direction.x() * _lineLength->value() / 2),
                 float(npoint.y() + direction.y() * _lineLength->value() / 2),
                 float(npoint.z() + direction.z() * _lineLength->value() / 2));
      glEnd();
      glPopMatrix();
    }

    return this;
  }
#endif

}
