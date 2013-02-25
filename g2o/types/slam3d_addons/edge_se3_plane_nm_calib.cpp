#include <iostream>
#include "edge_se3_plane_nm_calib.h"

#ifdef G2O_HAVE_OPENGL
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#endif

namespace Slam3dAddons {
  using namespace g2o;
  using namespace std;
  
  EdgeSE3PlaneNMSensorCalib::EdgeSE3PlaneNMSensorCalib() :
    BaseMultiEdge<4, Vector4d>()
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

  bool EdgeSE3PlaneNMSensorCalib::read(std::istream& is)
  {
    Vector4d v;
    is >> v(0) >> v(1) >> v(2) >> v(3);
    setMeasurement(v);
    for (int i = 0; i < information().rows(); ++i)
      for (int j = i; j < information().cols(); ++j) {
        is >> information()(i, j);
        if (i != j)
          information()(j, i) = information()(i, j);
      }
    return true;
  }

  bool EdgeSE3PlaneNMSensorCalib::write(std::ostream& os) const
  {
    Vector4d v = _measurement;
    os << v(0) << " " << v(1) << " " << v(2) << " " << v(3) << " ";
    for (int i = 0; i < information().rows(); ++i)
      for (int j = i; j < information().cols(); ++j)
        os << " " << information()(i, j);
    return os.good();
  }


#ifdef G2O_HAVE_OPENGL

  EdgeSE3PlaneNMSensorCalibDrawAction::EdgeSE3PlaneNMSensorCalibDrawAction(): DrawAction(typeid(EdgeSE3PlaneNMSensorCalib).name()){
  }

  bool EdgeSE3PlaneNMSensorCalibDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_){
    if (!DrawAction::refreshPropertyPtrs(params_))
      return false;
    if (_previousParams){
      _planeWidth = _previousParams->makeProperty<FloatProperty>(_typeName + "::PLANE_WIDTH", 3.0f);
      _planeHeight = _previousParams->makeProperty<FloatProperty>(_typeName + "::PLANE_HEIGHT", 3.0f);
    } else {
      _planeWidth = 0;
      _planeHeight = 0;
    }
    return true;
  }

  HyperGraphElementAction* EdgeSE3PlaneNMSensorCalibDrawAction::operator()(HyperGraph::HyperGraphElement* element, 
                 HyperGraphElementAction::Parameters* params_){

    if (typeid(*element).name()!=_typeName)
      return 0;


    refreshPropertyPtrs(params_);
    if (! _previousParams) 
      return this;
    
    if (_show && !_show->value())
      return this;


    EdgeSE3PlaneNMSensorCalib* that = dynamic_cast<EdgeSE3PlaneNMSensorCalib*>(element);
 
    if (! that)
      return this;

    const VertexSE3* robot       = dynamic_cast<const VertexSE3*>(that->vertex(0));
    const VertexSE3* sensor = dynamic_cast<const VertexSE3*>(that->vertex(2));

    cout << "that->vertex(0): " << that->vertex(0
) << " that->vertex(2): " << that->vertex(2) <<   endl;   
    

    if (! robot|| ! sensor)
      return 0;
    
    Plane3D plane(that->measurement());

    double d=plane.distance();
    double azimuth=Plane3D::azimuth(plane.normal());
    double elevation=Plane3D::elevation(plane.normal());
    // std::cerr << "D=" << d << std::endl;
    // std::cerr << "azimuth=" << azimuth << std::endl;
    // std::cerr << "elevation=" << azimuth << std::endl;

    
    glColor3f(0.4,0.25,0.25);
    glPushMatrix();
    Eigen::Isometry3d robotAndSensor = robot->estimate() * sensor->estimate();
    glMultMatrixd(robotAndSensor.matrix().data());


    glRotatef(RAD2DEG(azimuth),0.,0.,1.);
    glRotatef(RAD2DEG(elevation),0.,-1.,0.);
    glTranslatef(d,0.,0.);
    
    
    float planeWidth = 0.5;
    float planeHeight = 0.5;
    if (0) {
      planeWidth = _planeWidth->value();
      planeHeight = _planeHeight->value();
    }
    if (_planeWidth && _planeHeight){
      glBegin(GL_QUADS);
      glNormal3f(-1,0,0);
      glVertex3f(0,-planeWidth, -planeHeight);
      glVertex3f(0, planeWidth, -planeHeight);
      glVertex3f(0, planeWidth,  planeHeight);
      glVertex3f(0,-planeWidth,  planeHeight);
      glEnd();
    }

    glPopMatrix();
    
    return this;
  }
#endif

} // end namespace
