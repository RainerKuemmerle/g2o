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

#include "edge_se2_line2d.h"

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

  EdgeSE2Line2D::EdgeSE2Line2D() :
    BaseBinaryEdge<2, Line2D, VertexSE2, VertexLine2D>()
  {
  }

  bool EdgeSE2Line2D::read(std::istream& is)
  {
    is >> _measurement[0] >> _measurement[1];
    is >> information()(0,0) >> information()(0,1) >> information()(1,1);
    information()(1,0) = information()(0,1);
    return true;
  }

  bool EdgeSE2Line2D::write(std::ostream& os) const
  {
    os << measurement()[0] << " " << measurement()[1] << " ";
    os << information()(0,0) << " " << information()(0,1) << " " << information()(1,1);
    return os.good();
  }

  void EdgeSE2Line2D::initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to)
  {
    assert(from.size() == 1 && from.count(_vertices[0]) == 1 && "Can not initialize VertexSE2 position by VertexLine2D");

    VertexSE2* vi     = static_cast<VertexSE2*>(_vertices[0]);
    VertexLine2D* vj = static_cast<VertexLine2D*>(_vertices[1]);
    if (from.count(vi) > 0 && to == vj) {
      SE2 T=vi->estimate();
      Vector2D est=_measurement;
      est[0] += T.rotation().angle();
      est[0] = normalize_theta(est[0]);
      Vector2D n(cos(est[0]), sin(est[0]));
      est[1] += n.dot(T.translation());
      vj->setEstimate(est);
    }
  }

// #ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
//   void EdgeSE2Line2D::linearizeOplus()
//   {
//     const VertexSE2* vi     = static_cast<const VertexSE2*>(_vertices[0]);
//     const VertexLine2D* vj = static_cast<const VertexLine2D*>(_vertices[1]);
//     const double& x1        = vi->estimate().translation()[0];
//     const double& y1        = vi->estimate().translation()[1];
//     const double& th1       = vi->estimate().rotation().angle();
//     const double& x2        = vj->estimate()[0];
//     const double& y2        = vj->estimate()[1];

//     double aux_1 = cos(th1) ;
//     double aux_2 = -aux_1 ;
//     double aux_3 = sin(th1) ;

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

//   EdgeSE2Line2DWriteGnuplotAction::EdgeSE2Line2DWriteGnuplotAction(): WriteGnuplotAction(typeid(EdgeSE2Line2D).name()){}

//   HyperGraphElementAction* EdgeSE2Line2DWriteGnuplotAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_){
//     if (typeid(*element).name()!=_typeName)
//       return 0;
//     WriteGnuplotAction::Parameters* params=static_cast<WriteGnuplotAction::Parameters*>(params_);
//     if (!params->os){
//       std::cerr << __PRETTY_FUNCTION__ << ": warning, on valid os specified" << std::endl;
//       return 0;
//     }

//     EdgeSE2Line2D* e =  static_cast<EdgeSE2Line2D*>(element);
//     VertexSE2* fromEdge = static_cast<VertexSE2*>(e->vertex(0));
//     VertexLine2D* toEdge   = static_cast<VertexLine2D*>(e->vertex(1));
//     *(params->os) << fromEdge->estimate().translation().x() << " " << fromEdge->estimate().translation().y()
//       << " " << fromEdge->estimate().rotation().angle() << std::endl;
//     *(params->os) << toEdge->estimate().x() << " " << toEdge->estimate().y() << std::endl;
//     *(params->os) << std::endl;
//     return this;
//   }

// #ifdef G2O_HAVE_OPENGL
//   EdgeSE2Line2DDrawAction::EdgeSE2Line2DDrawAction(): DrawAction(typeid(EdgeSE2Line2D).name()){}

//   HyperGraphElementAction* EdgeSE2Line2DDrawAction::operator()(HyperGraph::HyperGraphElement* element,
//                 HyperGraphElementAction::Parameters*  params_){
//     if (typeid(*element).name()!=_typeName)
//       return 0;

//     refreshPropertyPtrs(params_);
//     if (! _previousParams)
//       return this;

//     if (_show && !_show->value())
//       return this;


//     EdgeSE2Line2D* e =  static_cast<EdgeSE2Line2D*>(element);
//     VertexSE2* fromEdge = static_cast<VertexSE2*>(e->vertex(0));
//     VertexLine2D* toEdge   = static_cast<VertexLine2D*>(e->vertex(1));
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
