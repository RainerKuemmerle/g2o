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

#include "vertex_segment2d.h"

#ifdef WINDOWS
#include <windows.h>
#endif

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#endif

#include <typeinfo>

#include "g2o/stuff/macros.h"

namespace g2o {

  VertexSegment2D::VertexSegment2D() :
    BaseVertex<4, Vector4>()
  {
    _estimate.setZero();
  }

  bool VertexSegment2D::read(std::istream& is)
  {
    for (size_t i=0; i<4; i++)
      is >> _estimate[i];
    return true;
  }

  bool VertexSegment2D::write(std::ostream& os) const
  {
    for (size_t i=0; i<4; i++)
      os << _estimate[i] << " ";
    return os.good();
  }

  VertexSegment2DWriteGnuplotAction::VertexSegment2DWriteGnuplotAction(): WriteGnuplotAction(typeid(VertexSegment2D).name()){}

  HyperGraphElementAction* VertexSegment2DWriteGnuplotAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
      return nullptr;

    WriteGnuplotAction::Parameters* params=static_cast<WriteGnuplotAction::Parameters*>(params_);
    if (!params->os){
      std::cerr << __PRETTY_FUNCTION__ << ": warning, on valid os specified" << std::endl;
      return nullptr;
    }

    VertexSegment2D* v =  static_cast<VertexSegment2D*>(element);
    *(params->os) << v->estimateP1().x() << " " << v->estimateP1().y() << std::endl;
    *(params->os) << v->estimateP2().x() << " " << v->estimateP2().y() << std::endl;
    *(params->os) << std::endl;
    return this;
  }

#ifdef G2O_HAVE_OPENGL
  VertexSegment2DDrawAction::VertexSegment2DDrawAction(): DrawAction(typeid(VertexSegment2D).name()){}

  bool VertexSegment2DDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_){
    if (! DrawAction::refreshPropertyPtrs(params_))
      return false;
    if (_previousParams){
      _pointSize = _previousParams->makeProperty<FloatProperty>(_typeName + "::POINT_SIZE", 1.);
    } else {
      _pointSize = 0;
    }
    return true;
  }

  HyperGraphElementAction* VertexSegment2DDrawAction::operator()(HyperGraph::HyperGraphElement* element,
                     HyperGraphElementAction::Parameters* params_ ){

    if (typeid(*element).name()!=_typeName)
      return nullptr;

    refreshPropertyPtrs(params_);
    if (! _previousParams)
      return this;

    if (_show && !_show->value())
      return this;


    VertexSegment2D* that = static_cast<VertexSegment2D*>(element);
    glColor3f(0.8f,0.5f,0.3f);
    if (_pointSize) {
      glPointSize(_pointSize->value());
    }
    glBegin(GL_LINES);
    glVertex3f((float)that->estimateP1().x(),(float)that->estimateP1().y(),0.f);
    glVertex3f((float)that->estimateP2().x(),(float)that->estimateP2().y(),0.f);
    glEnd();
    return this;
  }
#endif

} // end namespace
