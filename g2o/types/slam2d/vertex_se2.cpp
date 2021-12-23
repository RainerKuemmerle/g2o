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

#include "vertex_se2.h"
#include <typeinfo>

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#include "g2o/stuff/opengl_primitives.h"
#endif

namespace g2o {

  VertexSE2::VertexSE2() :
    BaseVertex<3, SE2>()
  {
  }

  bool VertexSE2::read(std::istream& is)
  {
    Vector3 p;
    bool state = internal::readVector(is, p);
    setEstimate(SE2(p));
    return state;
  }

  bool VertexSE2::write(std::ostream& os) const
  {
    return internal::writeVector(os, estimate().toVector());
  }

  VertexSE2WriteGnuplotAction::VertexSE2WriteGnuplotAction(): WriteGnuplotAction(typeid(VertexSE2).name()){}

  bool VertexSE2WriteGnuplotAction::operator()(HyperGraph::HyperGraphElement* element,
                                               HyperGraphElementAction::Parameters* params_) {
    if (typeid(*element).name()!=typeName_)
      return false;
    WriteGnuplotAction::Parameters* params=static_cast<WriteGnuplotAction::Parameters*>(params_);
    if (!params || !params->os){
      std::cerr << __PRETTY_FUNCTION__ << ": warning, no valid output stream specified" << std::endl;
      return false;
    }

    VertexSE2* v =  static_cast<VertexSE2*>(element);
    *(params->os) << v->estimate().translation().x() << " " << v->estimate().translation().y()
      << " " << v->estimate().rotation().angle() << std::endl;
    return true;
  }

#ifdef G2O_HAVE_OPENGL
  VertexSE2DrawAction::VertexSE2DrawAction()
      : DrawAction(typeid(VertexSE2).name()), drawActions_(nullptr), triangleX_(nullptr), triangleY_(nullptr) {}

  bool VertexSE2DrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_){
    if (!DrawAction::refreshPropertyPtrs(params_))
      return false;
    if (previousParams_){
      triangleX_ = previousParams_->makeProperty<FloatProperty>(typeName_ + "::TRIANGLE_X", .2f);
      triangleY_ = previousParams_->makeProperty<FloatProperty>(typeName_ + "::TRIANGLE_Y", .05f);
    } else {
      triangleX_ = 0;
      triangleY_ = 0;
    }
    return true;
  }

  bool VertexSE2DrawAction::operator()(HyperGraph::HyperGraphElement* element,
                                       HyperGraphElementAction::Parameters* params_) {
    if (typeid(*element).name() != typeName_) return false;
    initializeDrawActionsCache();
    refreshPropertyPtrs(params_);

    if (! previousParams_)
      return true;

    if (show_ && !show_->value())
      return true;

    VertexSE2* that = static_cast<VertexSE2*>(element);

    glColor3f(POSE_VERTEX_COLOR);
    glPushMatrix();
    glTranslatef((float)that->estimate().translation().x(),(float)that->estimate().translation().y(),0.f);
    glRotatef((float)RAD2DEG(that->estimate().rotation().angle()),0.f,0.f,1.f);
    opengl::drawArrow2D((float)triangleX_->value(), (float)triangleY_->value(), (float)triangleX_->value()*.3f);
    drawCache(that->cacheContainer(), params_);
    drawUserData(that->userData().get(), params_);
    glPopMatrix();
    return true;
  }
#endif


} // end namespace
