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

#include "vertex_line3d.h"

#include "g2o/stuff/misc.h"
#include "g2o/stuff/opengl_wrapper.h"

namespace g2o {

  VertexLine3D::VertexLine3D() {
    color << cst(1.0), cst(0.5), cst(0.0);
  }

  bool VertexLine3D::read(std::istream& is) {
    Vector6 lv;
    for(int i = 0; i < 6; ++i) {
      is >> lv[i];
    }
    setEstimate(Line3D(lv));
    return true;
  }

  bool VertexLine3D::write(std::ostream& os) const {
    Vector6 lv = _estimate;
    for(int i = 0; i < 6; ++i) {
      os << lv[i] << " ";
    }
    return os.good();
  }

#ifdef G2O_HAVE_OPENGL
  VertexLine3DDrawAction::VertexLine3DDrawAction()
      : DrawAction(typeid(VertexLine3D).name()), _lineLength(nullptr), _lineWidth(nullptr) {}

  bool VertexLine3DDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_) {
    if(!DrawAction::refreshPropertyPtrs(params_)) {
      return false;
    }
    if(_previousParams) {
      _lineLength = _previousParams->makeProperty<FloatProperty>(_typeName + "::LINE_LENGTH", 15);
      _lineWidth = _previousParams->makeProperty<FloatProperty>(_typeName + "::LINE_WIDTH", 5);
    }
    else {
      _lineLength = 0;
      _lineWidth = 0;
    }
    return true;
  }

  HyperGraphElementAction* VertexLine3DDrawAction::operator()(HyperGraph::HyperGraphElement* element,
							     HyperGraphElementAction::Parameters* params_) {
    if(typeid(*element).name() != _typeName) {
      return nullptr;
    }

    refreshPropertyPtrs(params_);
    if(!_previousParams) {
      return this;
    }

    if(_show && !_show->value()) {
      return this;
    }

    VertexLine3D* that = static_cast<VertexLine3D*>(element);
    Line3D line = that->estimate();
    line.normalize();
    Vector3 direction = line.d();
    Vector3 npoint = line.d().cross(line.w());
    glPushMatrix();
    glColor3f(float(that->color(0)), float(that->color(1)), float(that->color(2)));
    if(_lineLength && _lineWidth) {
      glLineWidth(float(_lineWidth->value()));
      glBegin(GL_LINES);
      glNormal3f(float(npoint.x()), float(npoint.y()), float(npoint.z()));
      glVertex3f(float(npoint.x() - direction.x() * _lineLength->value()/2),
		 float(npoint.y() - direction.y() * _lineLength->value()/2),
		 float(npoint.z() - direction.z() * _lineLength->value()/2));
      glVertex3f(float(npoint.x() + direction.x() * _lineLength->value()/2),
		 float(npoint.y() + direction.y() * _lineLength->value()/2),
		 float(npoint.z() + direction.z() * _lineLength->value()/2));
      glEnd();
    }
    glPopMatrix();

    return this;
  }
#endif

}
