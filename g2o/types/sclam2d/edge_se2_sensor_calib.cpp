#include "edge_se2_sensor_calib.h"

#ifdef G2O_HAVE_OPENGL
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#endif

namespace g2o {

  EdgeSE2SensorCalib::EdgeSE2SensorCalib() :
    BaseMultiEdge<3, SE2>()
  {
    resize(3);
  }

  void EdgeSE2SensorCalib::initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to)
  {
    (void) to;
    VertexSE2* vi = static_cast<VertexSE2*>(_vertices[0]);
    VertexSE2* vj = static_cast<VertexSE2*>(_vertices[1]);
    VertexSE2* l  = static_cast<VertexSE2*>(_vertices[2]);
    if (from.count(l) == 0)
      return;
    if (from.count(vi) == 1) {
      vj->setEstimate(vi->estimate() * l->estimate() * measurement() * l->estimate().inverse());
    } else {
      vi->setEstimate(vj->estimate() * l->estimate() * _inverseMeasurement * l->estimate().inverse());
    }
  }

  bool EdgeSE2SensorCalib::read(std::istream& is)
  {
    Vector3d p;
    is >> p(0) >> p(1) >> p(2);
    _measurement.fromVector(p);
    _inverseMeasurement=measurement().inverse();
    for (int i = 0; i < information().rows(); ++i)
      for (int j = i; j < information().cols(); ++j) {
        is >> information()(i, j);
        if (i != j)
          information()(j, i) = information()(i, j);
      }
    return true;
  }

  bool EdgeSE2SensorCalib::write(std::ostream& os) const
  {
    Vector3d p = measurement().toVector();
    os << p(0) << " " << p(1) << " " << p(2);
    for (int i = 0; i < information().rows(); ++i)
      for (int j = i; j < information().cols(); ++j)
        os << " " << information()(i, j);
    return os.good();
  }

#ifdef G2O_HAVE_OPENGL
  EdgeSE2SensorCalibDrawAction::EdgeSE2SensorCalibDrawAction() : 
    DrawAction(typeid(EdgeSE2SensorCalib).name())
  {
  }

  HyperGraphElementAction* EdgeSE2SensorCalibDrawAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* )
  {
    if (typeid(*element).name()!=_typeName)
      return 0;
    EdgeSE2SensorCalib* e = static_cast<EdgeSE2SensorCalib*>(element);
    VertexSE2* fromEdge = static_cast<VertexSE2*>(e->vertex(0));
    VertexSE2* toEdge   = static_cast<VertexSE2*>(e->vertex(1));
    glColor3f(0.5,0.5,1.0);
    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glVertex3f(fromEdge->estimate().translation().x(),fromEdge->estimate().translation().y(),0.);
    glVertex3f(toEdge->estimate().translation().x(),toEdge->estimate().translation().y(),0.);
    glEnd();
    glPopAttrib();
    return this;
  }
#endif

} // end namespace
