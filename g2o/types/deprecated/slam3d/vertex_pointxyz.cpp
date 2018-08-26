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

#include "vertex_pointxyz.h"

#include "g2o/stuff/opengl_wrapper.h"

#include <cstdio>
#include <typeinfo>

namespace g2o {
namespace deprecated {

  // TRACK VERTEX

  bool VertexPointXYZ::read(std::istream& is) {
    Vector3 lv;
    for (int i=0; i<3; i++)
      is >> lv[i];
    setEstimate(lv);
    return true;
  }

  bool VertexPointXYZ::write(std::ostream& os) const {
    const Vector3& lv = estimate();
    for (int i=0; i<3; i++){
      os << lv[i] << " ";
    }
    return os.good();
  }


#ifdef G2O_HAVE_OPENGL
  VertexPointXYZDrawAction::VertexPointXYZDrawAction(): DrawAction(typeid(VertexPointXYZ).name()){
  }

  bool VertexPointXYZDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_){
    if (! DrawAction::refreshPropertyPtrs(params_))
      return false;
    if (_previousParams){
      _pointSize = _previousParams->makeProperty<FloatProperty>(_typeName + "::POINT_SIZE", 1.f);
    } else {
      _pointSize = 0;
    }
    return true;
  }


  HyperGraphElementAction* VertexPointXYZDrawAction::operator()(HyperGraph::HyperGraphElement* element, 
                     HyperGraphElementAction::Parameters* params ){

    if (typeid(*element).name()!=_typeName)
      return nullptr;

    refreshPropertyPtrs(params);
    if (! _previousParams)
      return this;
    
    if (_show && !_show->value())
      return this;
    VertexPointXYZ* that = static_cast<VertexPointXYZ*>(element);
    

    glPushAttrib(GL_ENABLE_BIT | GL_POINT_BIT);
    glDisable(GL_LIGHTING);
    glColor3f(0.8f,0.5f,0.3f);
    if (_pointSize) {
      glPointSize(_pointSize->value());
    }
    glBegin(GL_POINTS);
    glVertex3f((float)that->estimate()(0),(float)that->estimate()(1),(float)that->estimate()(2));
    glEnd();
    glPopAttrib();
    return this;
  }
#endif

  VertexPointXYZWriteGnuplotAction::VertexPointXYZWriteGnuplotAction() :
    WriteGnuplotAction(typeid(VertexPointXYZ).name())
  {
  }

  HyperGraphElementAction* VertexPointXYZWriteGnuplotAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_ )
  {
    if (typeid(*element).name()!=_typeName)
      return nullptr;
    WriteGnuplotAction::Parameters* params=static_cast<WriteGnuplotAction::Parameters*>(params_);
    if (!params->os){
      std::cerr << __PRETTY_FUNCTION__ << ": warning, no valid os specified" << std::endl;
      return nullptr;
    }

    VertexPointXYZ* v = static_cast<VertexPointXYZ*>(element);
    *(params->os) << v->estimate().x() << " " << v->estimate().y() << " " << v->estimate().z() << " " << std::endl;
    return this;
  }

} // end namespace
} // end namespace
