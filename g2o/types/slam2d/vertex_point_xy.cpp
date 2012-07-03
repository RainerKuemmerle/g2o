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

#include "vertex_point_xy.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#endif

#include <typeinfo>

#include "g2o/stuff/macros.h"

namespace g2o {

  VertexPointXY::VertexPointXY() :
    BaseVertex<2, Vector2d>()
  {
    _estimate.setZero();
  }

  bool VertexPointXY::read(std::istream& is)
  {
    is >> _estimate[0] >> _estimate[1];
    return true;
  }

  bool VertexPointXY::write(std::ostream& os) const
  {
    os << estimate()(0) << " " << estimate()(1);
    return os.good();
  }

  VertexPointXYWriteGnuplotAction::VertexPointXYWriteGnuplotAction(): WriteGnuplotAction(typeid(VertexPointXY).name()){}

  HyperGraphElementAction* VertexPointXYWriteGnuplotAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
      return 0;

    WriteGnuplotAction::Parameters* params=static_cast<WriteGnuplotAction::Parameters*>(params_);
    if (!params->os){
      std::cerr << __PRETTY_FUNCTION__ << ": warning, on valid os specified" << std::endl;
      return 0;
    }
         
    VertexPointXY* v =  static_cast<VertexPointXY*>(element);
    *(params->os) << v->estimate().x() << " " << v->estimate().y() << std::endl;
    return this;
  }

#ifdef G2O_HAVE_OPENGL
  VertexPointXYDrawAction::VertexPointXYDrawAction(): DrawAction(typeid(VertexPointXY).name()){}

  bool VertexPointXYDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_){
    if (! DrawAction::refreshPropertyPtrs(params_))
      return false;
    if (_previousParams){
      _pointSize = _previousParams->makeProperty<FloatProperty>(_typeName + "::POINT_SIZE", 1.);
    } else {
      _pointSize = 0;
    }
    return true;
  }

  HyperGraphElementAction* VertexPointXYDrawAction::operator()(HyperGraph::HyperGraphElement* element, 
                     HyperGraphElementAction::Parameters* params_ ){

    if (typeid(*element).name()!=_typeName)
      return 0;

    refreshPropertyPtrs(params_);
    if (! _previousParams)
      return this;

    if (_show && !_show->value())
      return this;
    

    VertexPointXY* that = static_cast<VertexPointXY*>(element);
    glColor3f(0.8f,0.5f,0.3f);
    if (_pointSize) {
      glPointSize(_pointSize->value());
    }
    glBegin(GL_POINTS);
    glVertex3f((float)that->estimate().x(),(float)that->estimate().y(),0.f);
    glEnd();
    return this;
  }
#endif

} // end namespace
