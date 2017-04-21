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

#include "edge_xy_vxvy.h"

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

using namespace Eigen;

namespace g2o {

  EdgeXY_VXVY::EdgeXY_VXVY() :
    BaseBinaryEdge<4, Vector4d, VertexXY_VXVY, VertexXY_VXVY>()
  {
  }

  bool EdgeXY_VXVY::read(std::istream& is)
  {
    is >> _measurement[0] >> _measurement[1] >> _measurement[2] >> _measurement[3];
    is >> information()(0,0) >> information()(0,1) >> information()(0,2) >> information()(0,3) >> information()(1,1) >> information()(1,2) >> information()(1,3) >> information()(2,2) >> information()(2,3) >> information()(3,3);
    information()(1,0) = information()(0,1);
    information()(2,0) = information()(0,2);
    information()(2,1) = information()(1,2);
    information()(3,0) = information()(0,3);
    information()(3,1) = information()(1,3);
    information()(3,2) = information()(2,3);
    return true;
  }

  bool EdgeXY_VXVY::write(std::ostream& os) const
  {
    os << measurement()[0] << " " << measurement()[1] << " " << measurement()[2] << " " << measurement()[3] << " ";
    os << information()(0,0) << " " << information()(0,1) << " " << information()(0,2) << " " << information()(0,3) << " " << information()(1,1) << " " << information()(1,2) << " "  << information()(1,3) << " " << information()(2,2) << " " << information()(2,3)<< " " << information()(3,3);
    return os.good();
  }

  void EdgeXY_VXVY::initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to)
  {
    assert(from.size() == 1 && from.count(_vertices[0]) == 1 && "Can not initialize VertexXY_VXVY position by VertexXY_VXVY");

    VertexXY_VXVY* vi = static_cast<VertexXY_VXVY*>(_vertices[0]);
    VertexXY_VXVY* vj = static_cast<VertexXY_VXVY*>(_vertices[1]);
    if (from.count(vi) > 0 && to == vj) {
      //vj->setEstimate(vi->estimate() + _measurement*0.033); 
      
      double a = vi->estimate().x();
      double b = vi->estimate().y();
      double c = _measurement[0]/0.033;
      double d = _measurement[1]/0.033;
      
      vi->setEstimate(Vector4d(a,b,c,d));
      
      double p = vi->estimate().x() + _measurement[0] + _measurement[2]*0.033;
      double q = vi->estimate().y() + _measurement[1] + _measurement[3]*0.033;
      double r = vi->estimate().z() + _measurement[2];
      double s = vi->estimate().w() + _measurement[3];
      
      vj->setEstimate(Vector4d(p,q,r,s));
      
      //double *dataToSet;
      //dataToSet[0] = vi->estimate().x() + _measurement[0]*0.033 + 0.5*_measurement[2]*0.033*0.033; 
      //dataToSet[1] = vi->estimate().y() + _measurement[1]*0.033 + 0.5*_measurement[3]*0.033*0.033;
      //dataToSet[2] = vi->estimate().z() + _measurement[2]*0.033;
      //dataToSet[3] = vi->estimate().w() + _measurement[3]*0.033;
      //vj->setEstimateData(dataToSet);
      ///@TODO check this line's validity

    }
  }

#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
  void EdgeXY_VXVY::linearizeOplus()
  {
      const VertexXY_VXVY* vi = static_cast<const VertexXY_VXVY*>(_vertices[0]);
      const VertexXY_VXVY* vj = static_cast<const VertexXY_VXVY*>(_vertices[1]);
      
      unsigned long long timestamp2, timestamp1;
      timestamp1 = vi->timestamp;
      timestamp2 = vj->timestamp;
      
      double deltaT = ((double)abs(timestamp2 - timestamp1))/1000000;
      //std::cout<<deltaT<<std::endl;      
      
    _jacobianOplusXi(0, 0) =  -1;  _jacobianOplusXi(0, 1) = 0;   _jacobianOplusXi(0, 2) = -deltaT; _jacobianOplusXi(0, 3) = 0;
    _jacobianOplusXi(1, 0) =  0;  _jacobianOplusXi(1, 1) = -1;   _jacobianOplusXi(1, 2) = 0; _jacobianOplusXi(1, 3) = -deltaT;
    _jacobianOplusXi(2, 0) =  0;  _jacobianOplusXi(2, 1) = 0;   _jacobianOplusXi(2, 2) = -1; _jacobianOplusXi(2, 3) = 0;
    _jacobianOplusXi(3, 0) =  0;  _jacobianOplusXi(3, 1) = 0;   _jacobianOplusXi(3, 2) = 0; _jacobianOplusXi(3, 3) = -1;    

    _jacobianOplusXj(0, 0) = 1;  _jacobianOplusXj(0, 1)= 0;  _jacobianOplusXj(0, 2)= 0; _jacobianOplusXj(0, 3)= 0;
    _jacobianOplusXj(1, 0) = 0;  _jacobianOplusXj(1, 1)= 1;  _jacobianOplusXj(1, 2)= 0; _jacobianOplusXj(1, 3)= 0;
    _jacobianOplusXj(2, 0) = 0;  _jacobianOplusXj(2, 1)= 0;  _jacobianOplusXj(2, 2)= 1; _jacobianOplusXj(2, 3)= 0;
    _jacobianOplusXj(3, 0) = 0;  _jacobianOplusXj(3, 1)= 0;  _jacobianOplusXj(3, 2)= 0; _jacobianOplusXj(3, 3)= 1;
  
  }
#endif

  EdgeXY_VXVYWriteGnuplotAction::EdgeXY_VXVYWriteGnuplotAction(): WriteGnuplotAction(typeid(EdgeXY_VXVY).name()){}

  HyperGraphElementAction* EdgeXY_VXVYWriteGnuplotAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
      return 0;
    WriteGnuplotAction::Parameters* params=static_cast<WriteGnuplotAction::Parameters*>(params_);
    if (!params->os){
      std::cerr << __PRETTY_FUNCTION__ << ": warning, on valid os specified" << std::endl;
      return 0;
    }

    EdgeXY_VXVY* e =  static_cast<EdgeXY_VXVY*>(element);
    VertexXY_VXVY* fromEdge = static_cast<VertexXY_VXVY*>(e->vertex(0));
    VertexXY_VXVY* toEdge   = static_cast<VertexXY_VXVY*>(e->vertex(1));
    *(params->os) << fromEdge->estimate().x() << " " << fromEdge->estimate().y() << fromEdge->estimate().z() << " " << fromEdge->estimate().w() << std::endl;
    *(params->os) << toEdge->estimate().x() << " " << toEdge->estimate().y() << toEdge->estimate().z() << " " << toEdge->estimate().w() << std::endl;
    *(params->os) << std::endl;
    return this;
  }

#ifdef G2O_HAVE_OPENGL
  EdgeXY_VXVYDrawAction::EdgeXY_VXVYDrawAction(): DrawAction(typeid(EdgeXY_VXVY).name()){}

  HyperGraphElementAction* EdgeXY_VXVYDrawAction::operator()(HyperGraph::HyperGraphElement* element, 
                HyperGraphElementAction::Parameters*  params_){
    if (typeid(*element).name()!=_typeName)
      return 0;

    refreshPropertyPtrs(params_);
    if (! _previousParams)
      return this;
    
    if (_show && !_show->value())
      return this;


    EdgeXY_VXVY* e =  static_cast<EdgeXY_VXVY*>(element);
    VertexXY_VXVY* fromEdge = static_cast<VertexXY_VXVY*>(e->vertex(0));
    VertexXY_VXVY* toEdge   = static_cast<VertexXY_VXVY*>(e->vertex(1));
    glColor3f(0.4f,0.4f,0.2f);
    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glVertex3f((float)fromEdge->estimate().x(),(float)fromEdge->estimate().y(),0.f);
    glVertex3f((float)toEdge->estimate().x(),(float)toEdge->estimate().y(),0.f);
    glEnd();
    glPopAttrib();
    return this;
  }
#endif

} // end namespace
