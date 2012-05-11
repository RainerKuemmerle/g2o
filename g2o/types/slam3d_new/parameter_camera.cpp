#include "parameter_camera.h"
#include "isometry3d_gradients.h"
#include "isometry3d_mappings.h"

#ifdef G2O_HAVE_OPENGL
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#endif

using namespace std;

namespace Slam3dNew {
  using namespace g2o;

  ParameterCamera::ParameterCamera(){
    setId(-1);
    setKcam(1,1,0.5,0.5);
    setOffset();
  }

  void ParameterCamera::setOffset(const Eigen::Isometry3d& offset_){
    ParameterSE3Offset::setOffset(offset_);
    _Kcam_inverseOffsetR = _Kcam * inverseOffset().rotation();
  }

  void ParameterCamera::setKcam(double fx, double fy, double cx, double cy){
    _Kcam.setZero();
    _Kcam(0,0) = fx;
    _Kcam(1,1) = fy;
    _Kcam(0,2) = cx;
    _Kcam(1,2) = cy;
    _Kcam(2,2) = 1.0;
    _invKcam = _Kcam.inverse();
    _Kcam_inverseOffsetR = _Kcam * inverseOffset().rotation();
  }


  bool ParameterCamera::read(std::istream& is) {
    Vector7d off;
    for (int i=0; i<7; i++)
      is >> off[i];
    setOffset(fromVectorQT(off));
    double fx,fy,cx,cy;
    is >> fx >> fy >> cx >> cy;
    setKcam(fx,fy,cx,cy);
    return is.good();
  }
  
  bool ParameterCamera::write(std::ostream& os) const {
    Vector7d off = toVectorQT(_offset);
    for (int i=0; i<7; i++)
      os << off[i] << " ";
    os << _Kcam(0,0) << " ";
    os << _Kcam(1,1) << " ";
    os << _Kcam(0,2) << " ";
    os << _Kcam(1,2) << " ";
    return os.good();
  }

  bool CacheCamera::resolveDependancies(){
    if  (!CacheSE3Offset::resolveDependancies())
      return false;
    params = dynamic_cast<ParameterCamera*>(_parameters[0]);
    return params != 0;
  }

  void CacheCamera::updateImpl(){
    CacheSE3Offset::updateImpl();
    _w2i.matrix().topLeftCorner<3,4>() = params->Kcam() * w2n().matrix().topLeftCorner<3,4>();
  }

#ifdef G2O_HAVE_OPENGL
  static void drawMyPyramid(float height, float side){
    Vector3f p[6];
    p[0] << 0, 0., 0.;
    p[1] << -side, -side, height;
    p[2] << -side,  side, height;
    p[3] << side,  side, height;
    p[4] << side, -side, height;
    p[5] << -side, -side, height;

    glBegin(GL_TRIANGLES);
    for (int i = 1; i < 5; ++i) {
      Vector3f normal = (p[i] - p[0]).cross(p[i+1] - p[0]);
      glNormal3f(normal.x(), normal.y(), normal.z());
      glVertex3f(p[0].x(), p[0].y(), p[0].z());
      glVertex3f(p[i].x(), p[i].y(), p[i].z());
      glVertex3f(p[i+1].x(), p[i+1].y(), p[i+1].z());
    }
    glEnd();
  }

  CacheCameraDrawAction::CacheCameraDrawAction(): DrawAction(typeid(CacheCamera).name()){
    _previousParams = (DrawAction::Parameters*)0x42;
    refreshPropertyPtrs(0);
  }


  bool CacheCameraDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_){
    if (! DrawAction::refreshPropertyPtrs(params_))
      return false;
    if (_previousParams){
      _cameraZ = _previousParams->makeProperty<FloatProperty>(_typeName + "::CAMERA_Z", .05);
      _cameraSide = _previousParams->makeProperty<FloatProperty>(_typeName + "::CAMERA_SIDE", .05);
      
    } else {
      _cameraZ = 0;
      _cameraSide = 0;
    }
    return true;
  }

  HyperGraphElementAction* CacheCameraDrawAction::operator()(HyperGraph::HyperGraphElement* element, 
                 HyperGraphElementAction::Parameters* params){
    if (typeid(*element).name()!=_typeName)
      return 0;
    CacheCamera* that = static_cast<CacheCamera*>(element);
    refreshPropertyPtrs(params);
    if (! _previousParams)
      return this;
    
    if (_show && !_show->value())
      return this;

    glPushMatrix();
    glMultMatrixd(that->camParams()->offset().data());
    if (_cameraZ && _cameraSide)
      drawMyPyramid(_cameraZ->value(), _cameraSide->value());
    glPopMatrix();

    return this;
  }
#endif

}
