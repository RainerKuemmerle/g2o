#include "edge_se2_odom_differential_calib.h"

#ifdef G2O_HAVE_OPENGL
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#endif

namespace g2o {

  EdgeSE2OdomDifferentialCalib::EdgeSE2OdomDifferentialCalib() :
    BaseMultiEdge<3, VelocityMeasurement>()
  {
    resize(3);
  }

  bool EdgeSE2OdomDifferentialCalib::read(std::istream& is)
  {
    double vl, vr, dt;
    is >> vl >> vr >> dt;
    VelocityMeasurement vm(vl, vr, dt);
    setMeasurement(vm);
    for (int i = 0; i < information().rows(); ++i)
      for (int j = i; j < information().cols(); ++j) {
        is >> information()(i, j);
        if (i != j)
          information()(j, i) = information()(i, j);
      }
    return true;
  }

  bool EdgeSE2OdomDifferentialCalib::write(std::ostream& os) const
  {
    os << measurement().vl() << " " << measurement().vr() << " " << measurement().dt();
    for (int i = 0; i < information().rows(); ++i)
      for (int j = i; j < information().cols(); ++j)
        os << " " << information()(i, j);
    return os.good();
  }

#ifdef G2O_HAVE_OPENGL
  EdgeSE2OdomDifferentialCalibDrawAction::EdgeSE2OdomDifferentialCalibDrawAction() :
    DrawAction(typeid(EdgeSE2OdomDifferentialCalib).name())
  {
  }

  HyperGraphElementAction* EdgeSE2OdomDifferentialCalibDrawAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* )
  {
    if (typeid(*element).name()!=_typeName)
      return 0;
    EdgeSE2OdomDifferentialCalib* e = static_cast<EdgeSE2OdomDifferentialCalib*>(element);
    VertexSE2* fromEdge = static_cast<VertexSE2*>(e->vertex(0));
    VertexSE2* toEdge   = static_cast<VertexSE2*>(e->vertex(1));
    glColor3f(0.5,0.5,0.5);
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
