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

#include "edge_se2_segment2d.h"

namespace g2o {

EdgeSE2Segment2D::EdgeSE2Segment2D() : BaseBinaryEdge<4, Vector4, VertexSE2, VertexSegment2D>() {}

bool EdgeSE2Segment2D::read(std::istream& is) {
  internal::readVector(is, _measurement);
  return readInformationMatrix(is);
}

bool EdgeSE2Segment2D::write(std::ostream& os) const {
  internal::writeVector(os, measurement());
  return writeInformationMatrix(os);
}

void EdgeSE2Segment2D::initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to) {
  assert(from.size() == 1 && from.count(_vertices[0]) == 1 &&
         "Can not initialize VertexSE2 position by VertexSegment2D. I could if i wanted. Not now");

  VertexSE2* vi = static_cast<VertexSE2*>(_vertices[0]);
  VertexSegment2D* vj = static_cast<VertexSegment2D*>(_vertices[1]);
  if (from.count(vi) > 0 && to == vj) {
    vj->setEstimateP1(vi->estimate() * measurementP1());
    vj->setEstimateP2(vi->estimate() * measurementP2());
  }
}

// #ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
//   void EdgeSE2Segment2D::linearizeOplus()
//   {
//     const VertexSE2* vi     = static_cast<const VertexSE2*>(_vertices[0]);
//     const VertexSegment2D* vj = static_cast<const VertexSegment2D*>(_vertices[1]);
//     const number_t& x1        = vi->estimate().translation()[0];
//     const number_t& y1        = vi->estimate().translation()[1];
//     const number_t& th1       = vi->estimate().rotation().angle();
//     const number_t& x2        = vj->estimate()[0];
//     const number_t& y2        = vj->estimate()[1];

//     number_t aux_1 = cos(th1) ;
//     number_t aux_2 = -aux_1 ;
//     number_t aux_3 = sin(th1) ;

//     _jacobianOplusXi( 0 , 0 ) = aux_2 ;
//     _jacobianOplusXi( 0 , 1 ) = -aux_3 ;
//     _jacobianOplusXi( 0 , 2 ) = aux_1*y2-aux_1*y1-aux_3*x2+aux_3*x1 ;
//     _jacobianOplusXi( 1 , 0 ) = aux_3 ;
//     _jacobianOplusXi( 1 , 1 ) = aux_2 ;
//     _jacobianOplusXi( 1 , 2 ) = -aux_3*y2+aux_3*y1-aux_1*x2+aux_1*x1 ;

//     _jacobianOplusXj( 0 , 0 ) = aux_1 ;
//     _jacobianOplusXj( 0 , 1 ) = aux_3 ;
//     _jacobianOplusXj( 1 , 0 ) = -aux_3 ;
//     _jacobianOplusXj( 1 , 1 ) = aux_1 ;
//   }
// #endif

//   EdgeSE2Segment2DWriteGnuplotAction::EdgeSE2Segment2DWriteGnuplotAction():
//   WriteGnuplotAction(typeid(EdgeSE2Segment2D).name()){}

//   HyperGraphElementAction* EdgeSE2Segment2DWriteGnuplotAction::operator()(HyperGraph::HyperGraphElement* element,
//   HyperGraphElementAction::Parameters* params_){
//     if (typeid(*element).name()!=_typeName)
//       return nullptr;
//     WriteGnuplotAction::Parameters* params=static_cast<WriteGnuplotAction::Parameters*>(params_);
//     if (!params->os){
//       std::cerr << __PRETTY_FUNCTION__ << ": warning, on valid os specified" << std::endl;
//       return nullptr;
//     }

//     EdgeSE2Segment2D* e =  static_cast<EdgeSE2Segment2D*>(element);
//     VertexSE2* fromEdge = static_cast<VertexSE2*>(e->vertex(0));
//     VertexSegment2D* toEdge   = static_cast<VertexSegment2D*>(e->vertex(1));
//     *(params->os) << fromEdge->estimate().translation().x() << " " << fromEdge->estimate().translation().y()
//       << " " << fromEdge->estimate().rotation().angle() << std::endl;
//     *(params->os) << toEdge->estimate().x() << " " << toEdge->estimate().y() << std::endl;
//     *(params->os) << std::endl;
//     return this;
//   }

// #ifdef G2O_HAVE_OPENGL
//   EdgeSE2Segment2DDrawAction::EdgeSE2Segment2DDrawAction(): DrawAction(typeid(EdgeSE2Segment2D).name()){}

//   HyperGraphElementAction* EdgeSE2Segment2DDrawAction::operator()(HyperGraph::HyperGraphElement* element,
//                 HyperGraphElementAction::Parameters*  params_){
//     if (typeid(*element).name()!=_typeName)
//       return nullptr;

//     refreshPropertyPtrs(params_);
//     if (! _previousParams)
//       return this;

//     if (_show && !_show->value())
//       return this;

//     EdgeSE2Segment2D* e =  static_cast<EdgeSE2Segment2D*>(element);
//     VertexSE2* fromEdge = static_cast<VertexSE2*>(e->vertex(0));
//     VertexSegment2D* toEdge   = static_cast<VertexSegment2D*>(e->vertex(1));
//     glColor3f(0.4f,0.4f,0.2f);
//     glPushAttrib(GL_ENABLE_BIT);
//     glDisable(GL_LIGHTING);
//     glBegin(GL_LINES);
//     glVertex3f((float)fromEdge->estimate().translation().x(),(float)fromEdge->estimate().translation().y(),0.f);
//     glVertex3f((float)toEdge->estimate().x(),(float)toEdge->estimate().y(),0.f);
//     glEnd();
//     glPopAttrib();
//     return this;
//   }
// #endif

}  // namespace g2o
