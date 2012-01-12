// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "vertex_se2.h"
#include <typeinfo>

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

  VertexSE2::VertexSE2() :
    BaseVertex<3, SE2>()
  {
  }

  bool VertexSE2::read(std::istream& is)
  {
    Eigen::Vector3d p;
    is >> p[0] >> p[1] >> p[2];
    setEstimate(p);
    return true;
  }

  bool VertexSE2::write(std::ostream& os) const
  {
    Eigen::Vector3d p = estimate().toVector();
    os << p[0] << " " << p[1] << " " << p[2];
    return os.good();
  }

  VertexSE2WriteGnuplotAction::VertexSE2WriteGnuplotAction(): WriteGnuplotAction(typeid(VertexSE2).name()){}

  HyperGraphElementAction* VertexSE2WriteGnuplotAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
      return 0;
    WriteGnuplotAction::Parameters* params=static_cast<WriteGnuplotAction::Parameters*>(params_);
    if (!params || !params->os){
      std::cerr << __PRETTY_FUNCTION__ << ": warning, no valid output stream specified" << std::endl;
      return 0;
    }
    
    VertexSE2* v =  static_cast<VertexSE2*>(element);
    *(params->os) << v->estimate().translation().x() << " " << v->estimate().translation().y()
      << " " << v->estimate().rotation().angle() << std::endl;
    return this;
  }

#ifdef G2O_HAVE_OPENGL
  VertexSE2DrawAction::VertexSE2DrawAction(): DrawAction(typeid(VertexSE2).name()){}

  HyperGraphElementAction* VertexSE2DrawAction::operator()(HyperGraph::HyperGraphElement* element, 
                 HyperGraphElementAction::Parameters* /*params_*/){
    if (typeid(*element).name()!=_typeName)
      return 0;
    VertexSE2* that = static_cast<VertexSE2*>(element);
    glColor3f(0.5,0.5,0.8);
    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);
    glPushMatrix();
    glTranslatef(that->estimate().translation().x(),that->estimate().translation().y(),0.);
    glRotatef(RAD2DEG(that->estimate().rotation().angle()),0.,0.,1.);
    glBegin(GL_TRIANGLE_FAN);
    glVertex3f( 0.1 ,0.  ,0.);
    glVertex3f(-0.1 ,-0.05, 0.);
    glVertex3f(-0.1 , 0.05, 0.);
    glEnd();
    glPopMatrix();
    glPopAttrib();
    return this;
  }
#endif


} // end namespace
