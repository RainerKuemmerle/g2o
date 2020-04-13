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

#include "vertex_ellipse.h"

#include "g2o/stuff/macros.h"
#include "g2o/stuff/misc.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#include "g2o/stuff/opengl_primitives.h"
#include "EXTERNAL/freeglut/freeglut_minimal.h"
#endif

#include <iomanip>

#include <Eigen/Eigenvalues>

using namespace std;

namespace g2o {

VertexEllipse::VertexEllipse()
    : RobotData(), _covariance(Matrix3F::Zero()), _UMatrix(Matrix2F::Zero()), _singularValues(Vector2F::Zero()) {}

VertexEllipse::~VertexEllipse() {}

void VertexEllipse::_updateSVD() const {
  Eigen::SelfAdjointEigenSolver<Matrix2F> eigenSolver(_covariance.block<2, 2>(0, 0));
  _UMatrix = eigenSolver.eigenvectors();
  _singularValues = eigenSolver.eigenvalues();

  }

  bool VertexEllipse::read(std::istream& is)
  {
    float cxx, cxy, cxt, cyy, cyt, ctt;
    is >> cxx >> cxy >> cxt >> cyy >> cyt >> ctt;
    _covariance(0,0) = cxx;
    _covariance(0,1) = cxy;
    _covariance(0,2) = cxt;
    _covariance(1,0) = cxy;
    _covariance(1,1) = cyy;
    _covariance(1,2) = cyt;
    _covariance(2,0) = cxt;
    _covariance(2,1) = cyt;
    _covariance(2,2) = ctt;

    _updateSVD();

    int size;
    is >> size;
    for (int i =0; i<size; i++){
      float x, y;
      is >> x >> y;
      addMatchingVertex(x,y);
    }

    return true;
  }

  bool VertexEllipse::write(std::ostream& os) const
  {
    os << _covariance(0,0) << " " << _covariance(0,1) << " " << _covariance(0,2) << " "
       << _covariance(1,1) << " " << _covariance(1,2) << " " << _covariance(2,2) << " ";

    os << _matchingVertices.size() << " " ;
    for (size_t i=0 ; i< _matchingVertices.size(); i++){
      os << _matchingVertices[i].x() << " " << _matchingVertices[i].y() << " ";
    }

    return os.good();
  }



#ifdef G2O_HAVE_OPENGL
  VertexEllipseDrawAction::VertexEllipseDrawAction(): DrawAction(typeid(VertexEllipse).name()){
    _scaleFactor = 0;
  }

  bool VertexEllipseDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_){
    if (!DrawAction::refreshPropertyPtrs(params_))
      return false;
    if (_previousParams){
      _scaleFactor = _previousParams->makeProperty<DoubleProperty>(_typeName + "::", 1);
    } else {
      _scaleFactor = 0;
    }
    return true;
  }

  HyperGraphElementAction* VertexEllipseDrawAction::operator()(HyperGraph::HyperGraphElement* element,
							       HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
      return nullptr;

    refreshPropertyPtrs(params_);
    if (! _previousParams){
      return this;
    }
    if (_show && !_show->value())
      return this;

    VertexEllipse* that = dynamic_cast<VertexEllipse*>(element);

    glPushMatrix();

    float sigmaTheta = sqrt(that->covariance()(2,2));
    float x = 0.1f*cosf(sigmaTheta);
    float y = 0.1f*sinf(sigmaTheta);

    glColor3f(1.f,0.7f,1.f);
    glBegin(GL_LINE_STRIP);
    glVertex3f(x,y,0);
    glVertex3f(0,0,0);
    glVertex3f(x,-y,0);
    glEnd();

    glColor3f(0.f,1.f,0.f);
    for (size_t i=0; i< that->matchingVertices().size(); i++){
      glBegin(GL_LINES);
      glVertex3f(0,0,0);
      glVertex3f(that->matchingVertices()[i].x(),that->matchingVertices()[i].y(),0);
      glEnd();
    }

    Matrix2F rot = that->U();
    float angle = std::atan2(rot(1,0), rot(0,0));
    glRotatef(angle*180.0/const_pi(), 0., 0., 1.);
    Vector2F sv = that->singularValues();
    glScalef(sqrt(sv(0)), sqrt(sv(1)), 1);

    glColor3f(1.f,0.7f,1.f);
    glBegin(GL_LINE_LOOP);
    for(int i=0; i<36; i++){
      float rad = i*const_pi() /18.0;
      glVertex2f(std::cos(rad),
		 std::sin(rad));
    }
    glEnd();



    glPopMatrix();
    return this;
  }
#endif

}
