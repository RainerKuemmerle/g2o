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

#include "vertex_point_xy.h"

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

#include <typeinfo>

#include "g2o/stuff/macros.h"

namespace g2o {

  VertexPointXY::VertexPointXY() :
    BaseVertex<2, Vector2d>()
  {
    _estimate.setZero();
  }

  bool VertexPointXY::read(std::istream& is)
  {
    is >> _estimate[0] >> _estimate[1];
    return true;
  }

  bool VertexPointXY::write(std::ostream& os) const
  {
    os << estimate()(0) << " " << estimate()(1);
    return os.good();
  }

  VertexPointXYWriteGnuplotAction::VertexPointXYWriteGnuplotAction(): WriteGnuplotAction(typeid(VertexPointXY).name()){}

  HyperGraphElementAction* VertexPointXYWriteGnuplotAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
      return 0;

    WriteGnuplotAction::Parameters* params=static_cast<WriteGnuplotAction::Parameters*>(params_);
    if (!params->os){
      std::cerr << __PRETTY_FUNCTION__ << ": warning, on valid os specified" << std::endl;
      return false;
    }
         
    VertexPointXY* v =  static_cast<VertexPointXY*>(element);
    *(params->os) << v->estimate().x() << " " << v->estimate().y() << std::endl;
    return this;
  }

#ifdef G2O_HAVE_OPENGL
  VertexPointXYDrawAction::VertexPointXYDrawAction(): DrawAction(typeid(VertexPointXY).name()){}

  HyperGraphElementAction* VertexPointXYDrawAction::operator()(HyperGraph::HyperGraphElement* element, 
                     HyperGraphElementAction::Parameters* /*params_*/ ){

    if (typeid(*element).name()!=_typeName)
      return 0;
    VertexPointXY* that = static_cast<VertexPointXY*>(element);
    glColor3f(0.8,0.5,0.3);
    glBegin(GL_POINTS);
    glVertex3f(that->estimate().x(),that->estimate().y(),0.);
    glEnd();
    return this;
  }
#endif

} // end namespace
