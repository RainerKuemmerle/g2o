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

#include "vertex_line2d.h"

#ifdef WINDOWS
#include <windows.h>
#endif

#ifdef G2O_HAVE_OPENGL
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#endif

#include <typeinfo>

#include "g2o/stuff/macros.h"

namespace g2o {

  VertexLine2D::VertexLine2D() :
    BaseVertex<2, Line2D>()
  {
    _estimate.setZero();
    p1Id=p2Id=-1;
  }

  bool VertexLine2D::read(std::istream& is)
  {
    is >> _estimate[0] >> _estimate[1] >> p1Id >> p2Id;
    return true;
  }

  bool VertexLine2D::write(std::ostream& os) const
  {
    os << estimate()(0) << " " << estimate()(1) << " " << p1Id << " " << p2Id;
    return os.good();
  }

//   VertexLine2DWriteGnuplotAction::VertexLine2DWriteGnuplotAction(): WriteGnuplotAction(typeid(VertexLine2D).name()){}

//   HyperGraphElementAction* VertexLine2DWriteGnuplotAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_){
//     if (typeid(*element).name()!=_typeName)
//       return 0;

//     WriteGnuplotAction::Parameters* params=static_cast<WriteGnuplotAction::Parameters*>(params_);
//     if (!params->os){
//       std::cerr << __PRETTY_FUNCTION__ << ": warning, on valid os specified" << std::endl;
//       return false;
//     }

//     VertexLine2D* v =  static_cast<VertexLine2D*>(element);
//     *(params->os) << v->estimate().x() << " " << v->estimate().y() << std::endl;
//     return this;
//   }

#ifdef G2O_HAVE_OPENGL
  VertexLine2DDrawAction::VertexLine2DDrawAction(): DrawAction(typeid(VertexLine2D).name()){}

  bool VertexLine2DDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_){
    if (! DrawAction::refreshPropertyPtrs(params_))
      return false;
    if (_previousParams){
      _pointSize = _previousParams->makeProperty<FloatProperty>(_typeName + "::POINT_SIZE", 1.);
    } else {
      _pointSize = 0;
    }
    return true;
  }

  HyperGraphElementAction* VertexLine2DDrawAction::operator()(HyperGraph::HyperGraphElement* element,
                     HyperGraphElementAction::Parameters* params_ ){

    if (typeid(*element).name()!=_typeName)
      return 0;

    refreshPropertyPtrs(params_);
    if (! _previousParams)
      return this;

    if (_show && !_show->value())
      return this;


    VertexLine2D* that = static_cast<VertexLine2D*>(element);
    glPushAttrib(GL_CURRENT_BIT | GL_BLEND);
    if (_pointSize) {
      glPointSize(_pointSize->value());
    }
    Vector2D n(cos(that->theta()), sin(that->theta()));
    Vector2D pmiddle=n*that->rho();
    Vector2D t(-n.y(), n.x());
    double l1,l2 = 10;
    VertexPointXY *vp1=0, *vp2=0;
    vp1=dynamic_cast<VertexPointXY*> (that->graph()->vertex(that->p1Id));
    vp2=dynamic_cast<VertexPointXY*> (that->graph()->vertex(that->p2Id));

    glColor4f(0.8f,0.5f,0.3f,0.3f);
    if (vp1 && vp2) {
      glColor4f(0.8f,0.5f,0.3f,0.7f);
    } else if (vp1 || vp2){
      glColor4f(0.8f,0.5f,0.3f,0.5f);
    }

    if (vp1) {
      glColor4f(0.8f,0.5f,0.3f,0.7f);
      l1 = t.dot(vp1->estimate()-pmiddle);
    }
    if (vp2) {
      glColor4f(0.8f,0.5f,0.3f,0.7f);
      l2 = t.dot(vp2->estimate()-pmiddle);
    }
    Vector2D p1=pmiddle+t*l1;
    Vector2D p2=pmiddle+t*l2;
    glBegin(GL_LINES);
    glVertex3f((float)p1.x(),p1.y(),0.f);
    glVertex3f((float)p2.x(),p2.y(),0.f);
    glEnd();
    glPopAttrib();
    return this;
  }
#endif

} // end namespace
