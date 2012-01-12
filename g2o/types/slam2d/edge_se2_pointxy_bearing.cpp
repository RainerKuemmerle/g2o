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

#include "edge_se2_pointxy_bearing.h"

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

  EdgeSE2PointXYBearing::EdgeSE2PointXYBearing()
  {
  }


  void EdgeSE2PointXYBearing::initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* /*to*/)
  {
    assert(from.size() == 1 && from.count(_vertices[0]) == 1 && "Can not initialize VertexSE2 position by VertexPointXY");

    if (from.count(_vertices[0]) != 1)
      return;
    double r=2.;
    const VertexSE2* v1 = static_cast<const VertexSE2*>(_vertices[0]);
    VertexPointXY* l2 = static_cast<VertexPointXY*>(_vertices[1]);
    SE2 t=v1->estimate();
    t.setRotation(t.rotation().angle()+_measurement);
    Vector2d vr;
    vr[0]=r; vr[1]=0;
    l2->setEstimate(t*vr);
  }

  bool EdgeSE2PointXYBearing::read(std::istream& is)
  {
    is >> _measurement >> information()(0,0);
    return true;
  }

  bool EdgeSE2PointXYBearing::write(std::ostream& os) const
  {
    os << measurement() << " " << information()(0,0);
    return os.good();
  }


  EdgeSE2PointXYBearingWriteGnuplotAction::EdgeSE2PointXYBearingWriteGnuplotAction(): WriteGnuplotAction(typeid(EdgeSE2PointXYBearing).name()){}

  HyperGraphElementAction* EdgeSE2PointXYBearingWriteGnuplotAction::operator()(HyperGraph::HyperGraphElement* element,
                         HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
      return 0;
    WriteGnuplotAction::Parameters* params=static_cast<WriteGnuplotAction::Parameters*>(params_);
    if (!params->os){
      std::cerr << __PRETTY_FUNCTION__ << ": warning, on valid os specified" << std::endl;
      return 0;
    }

    EdgeSE2PointXYBearing* e =  static_cast<EdgeSE2PointXYBearing*>(element);
    VertexSE2* fromEdge = static_cast<VertexSE2*>(e->vertex(0));
    VertexPointXY* toEdge   = static_cast<VertexPointXY*>(e->vertex(1));
    *(params->os) << fromEdge->estimate().translation().x() << " " << fromEdge->estimate().translation().y()
      << " " << fromEdge->estimate().rotation().angle() << std::endl;
    *(params->os) << toEdge->estimate().x() << " " << toEdge->estimate().y() << std::endl;
    *(params->os) << std::endl;
    return this;
  }

#ifdef G2O_HAVE_OPENGL
  EdgeSE2PointXYBearingDrawAction::EdgeSE2PointXYBearingDrawAction(): DrawAction(typeid(EdgeSE2PointXYBearing).name()){}

  HyperGraphElementAction* EdgeSE2PointXYBearingDrawAction::operator()(HyperGraph::HyperGraphElement* element,  HyperGraphElementAction::Parameters* /*params_*/){
    if (typeid(*element).name()!=_typeName)
      return 0;
    EdgeSE2PointXYBearing* e =  static_cast<EdgeSE2PointXYBearing*>(element);
    VertexSE2* fromEdge = static_cast<VertexSE2*>(e->vertex(0));
    VertexPointXY* toEdge   = static_cast<VertexPointXY*>(e->vertex(1));
    glColor3f(0.3,0.3,0.1);
    glBegin(GL_LINES);
    glVertex3f(fromEdge->estimate().translation().x(),fromEdge->estimate().translation().y(),0.);
    glVertex3f(toEdge->estimate().x(),toEdge->estimate().y(),0.);
    glEnd();
    return this;
  }
#endif

} // end namespace
