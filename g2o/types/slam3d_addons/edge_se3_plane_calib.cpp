#include "edge_se3_plane_calib.h"

#ifdef G2O_HAVE_OPENGL
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#endif

namespace Slam3dAddons {
  using namespace g2o;

  EdgeSE3PlaneSensorCalib::EdgeSE3PlaneSensorCalib() :
    BaseMultiEdge<3, Plane3D>()
  {
    resize(3);
  }

  // void EdgeSE2SensorCalib::initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to)
  // {
  //   (void) to;
  //   VertexSE2* vi = static_cast<VertexSE2*>(_vertices[0]);
  //   VertexSE2* vj = static_cast<VertexSE2*>(_vertices[1]);
  //   VertexSE2* l  = static_cast<VertexSE2*>(_vertices[2]);
  //   if (from.count(l) == 0)
  //     return;
  //   if (from.count(vi) == 1) {
  //     vj->setEstimate(vi->estimate() * l->estimate() * measurement() * l->estimate().inverse());
  //   } else {
  //     vi->setEstimate(vj->estimate() * l->estimate() * _inverseMeasurement * l->estimate().inverse());
  //   }
  // }

  bool EdgeSE3PlaneSensorCalib::read(std::istream& is)
  {
    Vector4d v;
    is >> v(0) >> v(1) >> v(2) >> v(3);
    setMeasurement(Plane3D(v));
    for (int i = 0; i < information().rows(); ++i)
      for (int j = i; j < information().cols(); ++j) {
        is >> information()(i, j);
        if (i != j)
          information()(j, i) = information()(i, j);
      }
    return true;
  }

  bool EdgeSE3PlaneSensorCalib::write(std::ostream& os) const
  {
    Vector4d v = _measurement.toVector();
    os << v(0) << " " << v(1) << " " << v(2) << " " << v(3) << " ";
    for (int i = 0; i < information().rows(); ++i)
      for (int j = i; j < information().cols(); ++j)
        os << " " << information()(i, j);
    return os.good();
  }

// #ifdef G2O_HAVE_OPENGL
//   EdgeSE2SensorCalibDrawAction::EdgeSE2SensorCalibDrawAction() : 
//     DrawAction(typeid(EdgeSE2SensorCalib).name())
//   {
//   }

//   HyperGraphElementAction* EdgeSE2SensorCalibDrawAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* )
//   {
//     if (typeid(*element).name()!=_typeName)
//       return 0;
//     EdgeSE2SensorCalib* e = static_cast<EdgeSE2SensorCalib*>(element);
//     VertexSE2* fromEdge = static_cast<VertexSE2*>(e->vertices()[0]);
//     VertexSE2* toEdge   = static_cast<VertexSE2*>(e->vertices()[1]);
//     glColor3f(0.5,0.5,1.0);
//     glPushAttrib(GL_ENABLE_BIT);
//     glDisable(GL_LIGHTING);
//     glBegin(GL_LINES);
//     glVertex3f(fromEdge->estimate().translation().x(),fromEdge->estimate().translation().y(),0.);
//     glVertex3f(toEdge->estimate().translation().x(),toEdge->estimate().translation().y(),0.);
//     glEnd();
//     glPopAttrib();
//     return this;
//   }
// #endif

} // end namespace
