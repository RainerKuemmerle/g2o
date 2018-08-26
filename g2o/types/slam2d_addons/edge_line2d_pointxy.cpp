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

#include "edge_line2d_pointxy.h"

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

namespace g2o {

  EdgeLine2DPointXY::EdgeLine2DPointXY() :
    BaseBinaryEdge<1, number_t, VertexLine2D, VertexPointXY>()
  {
  }

  bool EdgeLine2DPointXY::read(std::istream& is)
  {
    is >> _measurement;
    is >> information()(0,0);
    return true;
  }

  bool EdgeLine2DPointXY::write(std::ostream& os) const
  {
    os << measurement() << " ";
    os << information()(0,0);
    return os.good();
  }

  // void EdgeLine2DPointXY::initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to)
  // {
  //   assert(from.size() == 1 && from.count(_vertices[0]) == 1 && "Can not initialize VertexLine2D position by VertexPointXY");

  //   VertexLine2D* vi     = static_cast<VertexLine2D*>(_vertices[0]);
  //   VertexPointXY* vj = static_cast<VertexPointXY*>(_vertices[1]);
  //   if (from.count(vi) > 0 && to == vj) {
  //     Line2D T=vi->estimate();
  //     Vector2 est=_measurement;
  //     est[0] += T.rotation().angle();
  //     est[0] = normalize_theta(est[0]);
  //     Vector2 n(cos(est[0]), sin(est[0]));
  //     est[1] += n.dot(T.translation());
  //     vj->setEstimate(est);
  //   }
  // }

// #ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
//   void EdgeLine2DPointXY::linearizeOplus()
//   {
//     const VertexLine2D* vi     = static_cast<const VertexLine2D*>(_vertices[0]);
//     const VertexPointXY* vj = static_cast<const VertexPointXY*>(_vertices[1]);
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

//   EdgeLine2DPointXYWriteGnuplotAction::EdgeLine2DPointXYWriteGnuplotAction(): WriteGnuplotAction(typeid(EdgeLine2DPointXY).name()){}

//   HyperGraphElementAction* EdgeLine2DPointXYWriteGnuplotAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_){
//     if (typeid(*element).name()!=_typeName)
//       return nullptr;
//     WriteGnuplotAction::Parameters* params=static_cast<WriteGnuplotAction::Parameters*>(params_);
//     if (!params->os){
//       std::cerr << __PRETTY_FUNCTION__ << ": warning, on valid os specified" << std::endl;
//       return nullptr;
//     }

//     EdgeLine2DPointXY* e =  static_cast<EdgeLine2DPointXY*>(element);
//     VertexLine2D* fromEdge = static_cast<VertexLine2D*>(e->vertex(0));
//     VertexPointXY* toEdge   = static_cast<VertexPointXY*>(e->vertex(1));
//     *(params->os) << fromEdge->estimate().translation().x() << " " << fromEdge->estimate().translation().y()
//       << " " << fromEdge->estimate().rotation().angle() << std::endl;
//     *(params->os) << toEdge->estimate().x() << " " << toEdge->estimate().y() << std::endl;
//     *(params->os) << std::endl;
//     return this;
//   }

// #ifdef G2O_HAVE_OPENGL
//   EdgeLine2DPointXYDrawAction::EdgeLine2DPointXYDrawAction(): DrawAction(typeid(EdgeLine2DPointXY).name()){}

//   HyperGraphElementAction* EdgeLine2DPointXYDrawAction::operator()(HyperGraph::HyperGraphElement* element,
//                 HyperGraphElementAction::Parameters*  params_){
//     if (typeid(*element).name()!=_typeName)
//       return nullptr;

//     refreshPropertyPtrs(params_);
//     if (! _previousParams)
//       return this;

//     if (_show && !_show->value())
//       return this;


//     EdgeLine2DPointXY* e =  static_cast<EdgeLine2DPointXY*>(element);
//     VertexLine2D* fromEdge = static_cast<VertexLine2D*>(e->vertex(0));
//     VertexPointXY* toEdge   = static_cast<VertexPointXY*>(e->vertex(1));
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

} // end namespace
