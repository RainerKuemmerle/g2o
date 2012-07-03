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

#include "edge_se3_quat.h"
#include "g2o/core/factory.h"
#include "se3quat_gradients.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#endif

#include <iostream>

using namespace std;

namespace g2o {
namespace deprecated {


  EdgeSE3::EdgeSE3() :
    BaseBinaryEdge<6, SE3Quat, VertexSE3, VertexSE3>()
  {
  }


  bool EdgeSE3::setMeasurementFromState(){
    const VertexSE3* v1 = dynamic_cast<const VertexSE3*>(_vertices[0]);
    const VertexSE3* v2 = dynamic_cast<const VertexSE3*>(_vertices[1]);
    _measurement = (v1->estimate().inverse()*v2->estimate());
    _inverseMeasurement = _measurement.inverse();
    return true;
  }

  bool EdgeSE3::read(std::istream& is)
  {
    Vector7d meas;
    for (int i=0; i<7; i++)
      is >> meas[i];
    setMeasurement(SE3Quat(meas));
    
    for (int i=0; i<6; i++)
      for (int j=i; j<6; j++) {
        is >> information()(i,j);
        if (i!=j)
          information()(j,i) = information()(i,j);
      }
    return true;
  }

  bool EdgeSE3::write(std::ostream& os) const
  {
    for (int i=0; i<7; i++)
      os << measurement()[i] << " ";
    for (int i=0; i<6; i++)
      for (int j=i; j<6; j++){
        os << " " <<  information()(i,j);
      }
    return os.good();
  }

  void EdgeSE3::initialEstimate(const OptimizableGraph::VertexSet& from_, OptimizableGraph::Vertex* /*to_*/)
  {
    VertexSE3* from = static_cast<VertexSE3*>(_vertices[0]);
    VertexSE3* to = static_cast<VertexSE3*>(_vertices[1]);
    if (from_.count(from) > 0) {
      to->setEstimate(from->estimate() * _measurement);
    } else
      from->setEstimate(to->estimate() * _inverseMeasurement);
  }

#ifdef EDGE_SE3_QUAT_ANALYTIC_JACOBIAN
  void EdgeSE3::linearizeOplus(){
    VertexSE3* from = static_cast<VertexSE3*>(_vertices[0]);
    VertexSE3* to = static_cast<VertexSE3*>(_vertices[1]);
    
    Matrix3d izR        = _inverseMeasurement.rotation().toRotationMatrix();
    const Vector3d& izt = _inverseMeasurement.translation();
    
    SE3Quat iXiXj         = from->estimate().inverse() * to->estimate();
    Matrix3d iRiRj        = iXiXj.rotation().toRotationMatrix();
    const Vector3d& ititj = iXiXj.translation();

    jacobian_3d_qman ( _jacobianOplusXi , _jacobianOplusXj,
           izR(0,0), izR(0,1), izR(0,2), izt(0),
           izR(1,0), izR(1,1), izR(1,2), izt(1),
           izR(2,0), izR(2,1), izR(2,2), izt(2),
           iRiRj(0,0), iRiRj(0,1), iRiRj(0,2), ititj(0),
           iRiRj(1,0), iRiRj(1,1), iRiRj(1,2), ititj(1),
           iRiRj(2,0), iRiRj(2,1), iRiRj(2,2), ititj(2));
  }
#endif



  EdgeSE3WriteGnuplotAction::EdgeSE3WriteGnuplotAction(): WriteGnuplotAction(typeid(EdgeSE3).name()){}

  HyperGraphElementAction* EdgeSE3WriteGnuplotAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
      return 0;
    WriteGnuplotAction::Parameters* params=static_cast<WriteGnuplotAction::Parameters*>(params_);
    if (!params->os){
      std::cerr << __PRETTY_FUNCTION__ << ": warning, on valid os specified" << std::endl;
      return 0;
    }

    EdgeSE3* e =  static_cast<EdgeSE3*>(element);
    VertexSE3* fromEdge = static_cast<VertexSE3*>(e->vertex(0));
    VertexSE3* toEdge   = static_cast<VertexSE3*>(e->vertex(1));
    *(params->os) << fromEdge->estimate().translation().x() << " " 
      << fromEdge->estimate().translation().y() << " " 
      << fromEdge->estimate().translation().z() << " ";
    *(params->os) << fromEdge->estimate().rotation().x() << " " 
      << fromEdge->estimate().rotation().y() << " " 
      << fromEdge->estimate().rotation().z() << " " << std::endl;
    *(params->os) << toEdge->estimate().translation().x() << " " 
      << toEdge->estimate().translation().y() << " " 
      << toEdge->estimate().translation().z() << " ";
    *(params->os) << toEdge->estimate().rotation().x() << " " 
      << toEdge->estimate().rotation().y() << " " 
      << toEdge->estimate().rotation().z() << " " << std::endl;
    *(params->os) << std::endl;
    return this;
  }

#ifdef G2O_HAVE_OPENGL
  EdgeSE3DrawAction::EdgeSE3DrawAction(): DrawAction(typeid(EdgeSE3).name()){}

  HyperGraphElementAction* EdgeSE3DrawAction::operator()(HyperGraph::HyperGraphElement* element, 
               HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
      return 0;
    refreshPropertyPtrs(params_);
    if (! _previousParams)
      return this;
    
    if (_show && !_show->value())
      return this;
    
    EdgeSE3* e =  static_cast<EdgeSE3*>(element);
    VertexSE3* fromEdge = static_cast<VertexSE3*>(e->vertex(0));
    VertexSE3* toEdge   = static_cast<VertexSE3*>(e->vertex(1));
    glColor3f(0.5f,0.5f,0.8f);
    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glVertex3f((float)fromEdge->estimate().translation().x(),(float)fromEdge->estimate().translation().y(),(float)fromEdge->estimate().translation().z());
    glVertex3f((float)toEdge->estimate().translation().x(),(float)toEdge->estimate().translation().y(),(float)toEdge->estimate().translation().z());
    glEnd();
    glPopAttrib();
    return this;
  }
#endif

} // end namespace
} // end namespace
