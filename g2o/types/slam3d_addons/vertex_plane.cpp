#include "vertex_plane.h"
#include <stdio.h>

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

namespace Slam3dAddons {
  using namespace g2o;

  VertexPlane::VertexPlane(){
    color << .2, .2, .2;
  }
  
  bool VertexPlane::read(std::istream& is) {
    Vector4d lv;
    for (int i=0; i<4; i++)
      is >> lv[i];
    setEstimate(Plane3D(lv));
    is >> color(0) >> color(1) >> color(2);
    return true;
  }

  bool VertexPlane::write(std::ostream& os) const {
    Vector4d lv=_estimate.toVector();
    for (int i=0; i<4; i++){
      os << lv[i] << " ";
    }
    os << color(0) << " " << color(1) << " " << color(2) << " ";
    return os.good();
  }

#ifdef G2O_HAVE_OPENGL
  void drawTriangle(float xSize, float ySize);

  VertexPlaneDrawAction::VertexPlaneDrawAction(): DrawAction(typeid(VertexPlane).name()){
  }

  bool VertexPlaneDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_){
    if (!DrawAction::refreshPropertyPtrs(params_))
      return false;
    if (_previousParams){
      _planeWidth = _previousParams->makeProperty<FloatProperty>(_typeName + "::PLANE_WIDTH", 3);
      _planeHeight = _previousParams->makeProperty<FloatProperty>(_typeName + "::PLANE_HEIGHT", 3);
    } else {
      _planeWidth = 0;
      _planeHeight = 0;
    }
    return true;
  }

  HyperGraphElementAction* VertexPlaneDrawAction::operator()(HyperGraph::HyperGraphElement* element, 
                 HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
      return 0;

    refreshPropertyPtrs(params_);
    if (! _previousParams)
      return this;
    
    if (_show && !_show->value())
      return this;

    VertexPlane* that = static_cast<VertexPlane*>(element);
    double d=that->estimate().distance();
    double azimuth=Plane3D::azimuth(that->estimate().normal());
    double elevation=Plane3D::elevation(that->estimate().normal());
    // std::cerr << "D=" << d << std::endl;
    // std::cerr << "azimuth=" << azimuth << std::endl;
    // std::cerr << "elevation=" << azimuth << std::endl;
    glColor3f(that->color(0), that->color(1), that->color(2));
    glPushMatrix();
    glRotatef(RAD2DEG(azimuth),0.,0.,1.);
    glRotatef(RAD2DEG(elevation),0.,-1.,0.);
    glTranslatef(d,0.,0.);
    
    if (_planeWidth && _planeHeight){
      glBegin(GL_QUADS);
      glNormal3f(-1,0,0);
      glVertex3f(0,-_planeWidth->value(), -_planeHeight->value());
      glVertex3f(0, _planeWidth->value(), -_planeHeight->value());
      glVertex3f(0, _planeWidth->value(),  _planeHeight->value());
      glVertex3f(0,-_planeWidth->value(),  _planeHeight->value());
      glEnd();
    }

    glPopMatrix();
    return this;
  }
#endif

}

