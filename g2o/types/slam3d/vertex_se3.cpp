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

#include "vertex_se3.h"
#include "g2o/core/factory.h"
#include "g2o/stuff/opengl_wrapper.h"

#include <iostream>
#include "g2o/core/cache.h"

namespace g2o {

  VertexSE3::VertexSE3() :
    BaseVertex<6, Eigen::Isometry3d>(),
    _numOplusCalls(0)
  {
    setToOriginImpl();
    updateCache();
  }

  bool VertexSE3::read(std::istream& is)
  {
    Vector7d est;
    for (int i=0; i<7; i++)
      is  >> est[i];
    setEstimate(internal::fromVectorQT(est));
    return true;
  }

  bool VertexSE3::write(std::ostream& os) const
  {
    Vector7d est=internal::toVectorQT(_estimate);
    for (int i=0; i<7; i++)
      os << est[i] << " ";
    return os.good();
  }

  VertexSE3WriteGnuplotAction::VertexSE3WriteGnuplotAction(): WriteGnuplotAction(typeid(VertexSE3).name()){}

  HyperGraphElementAction* VertexSE3WriteGnuplotAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
      return 0;
    WriteGnuplotAction::Parameters* params=static_cast<WriteGnuplotAction::Parameters*>(params_);
    if (!params->os){
      std::cerr << __PRETTY_FUNCTION__ << ": warning, no valid os specified" << std::endl;
      return 0;
    }
    
    VertexSE3* v =  static_cast<VertexSE3*>(element);
    Vector6d est=internal::toVectorMQT(v->estimate());
    for (int i=0; i<6; i++)
      *(params->os) << est[i] << " ";
    *(params->os) << std::endl;
    return this;
  }

#ifdef G2O_HAVE_OPENGL
  void drawTriangle(float xSize, float ySize){
    Vector3f p[3];
    glBegin(GL_TRIANGLES);
    p[0] << 0., 0., 0.;
    p[1] << -xSize, ySize, 0.;
    p[2] << -xSize, -ySize, 0.;
    for (int i = 1; i < 2; ++i) {
      Vector3f normal = (p[i] - p[0]).cross(p[i+1] - p[0]);
      glNormal3f(normal.x(), normal.y(), normal.z());
      glVertex3f(p[0].x(), p[0].y(), p[0].z());
      glVertex3f(p[i].x(), p[i].y(), p[i].z());
      glVertex3f(p[i+1].x(), p[i+1].y(), p[i+1].z());
    }    
    glEnd();
  }

  VertexSE3DrawAction::VertexSE3DrawAction(): DrawAction(typeid(VertexSE3).name()){
    _cacheDrawActions = 0;
  }

  bool VertexSE3DrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_){
    if (!DrawAction::refreshPropertyPtrs(params_))
      return false;
    if (_previousParams){
      _triangleX = _previousParams->makeProperty<FloatProperty>(_typeName + "::TRIANGLE_X", .2f);
      _triangleY = _previousParams->makeProperty<FloatProperty>(_typeName + "::TRIANGLE_Y", .05f);
    } else {
      _triangleX = 0;
      _triangleY = 0;
    }
    return true;
  }

  HyperGraphElementAction* VertexSE3DrawAction::operator()(HyperGraph::HyperGraphElement* element, 
                 HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
      return 0;
    if (! _cacheDrawActions){
      _cacheDrawActions = HyperGraphActionLibrary::instance()->actionByName("draw");
    }

    refreshPropertyPtrs(params_);
    if (! _previousParams)
      return this;
    
    if (_show && !_show->value())
      return this;

    VertexSE3* that = static_cast<VertexSE3*>(element);

    glColor3f(0.5f,0.5f,0.8f);
    glPushMatrix();
    glMultMatrixd(that->estimate().matrix().data());
    if (_triangleX && _triangleY){
      drawTriangle(_triangleX->value(), _triangleY->value());
    }
    CacheContainer* caches=that->cacheContainer();
    if (caches){
      for (CacheContainer::iterator it=caches->begin(); it!=caches->end(); it++){
        Cache* c = it->second;
        (*_cacheDrawActions)(c, params_);
      }
    }
    OptimizableGraph::Data* d=that->userData();
    while (d && _cacheDrawActions ){
      (*_cacheDrawActions)(d, params_);
      d=d->next();
    }
    glPopMatrix();
    return this;
  }
#endif

}
