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

#include "parameter_camera.h"
#include "g2o/stuff/opengl_wrapper.h"

using namespace std;

namespace g2o {
namespace deprecated {


  ParameterCamera::ParameterCamera(){
    setId(-1);
    setKcam(1,1,0.5,0.5);
    setOffset();
  }

  void ParameterCamera::setOffset(const SE3Quat& offset_){
    ParameterSE3Offset::setOffset(offset_);
    _Kcam_inverseOffsetR = _Kcam * inverseOffsetMatrix().rotation();
  }

  void ParameterCamera::setKcam(double fx, double fy, double cx, double cy){
    _Kcam.setZero();
    _Kcam(0,0) = fx;
    _Kcam(1,1) = fy;
    _Kcam(0,2) = cx;
    _Kcam(1,2) = cy;
    _Kcam(2,2) = 1.0;
    _invKcam = _Kcam.inverse();
    _Kcam_inverseOffsetR = _Kcam * inverseOffsetMatrix().rotation();
  }


  bool ParameterCamera::read(std::istream& is) {
    Vector7 off;
    for (int i=0; i<7; i++)
      is >> off[i];
    setOffset(SE3Quat(off));
    double fx,fy,cx,cy;
    is >> fx >> fy >> cx >> cy;
    setKcam(fx,fy,cx,cy);
    return is.good();
  }
  
  bool ParameterCamera::write(std::ostream& os) const {
    Vector7 off = offset().toVector();
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
    _w2i.matrix().topLeftCorner<3,4>() = params->Kcam() * w2nMatrix().matrix().topLeftCorner<3,4>();
  }

#ifdef G2O_HAVE_OPENGL
  static void drawMyPyramid(float height, float side){
    Eigen::Vector3f p[6];
    p[0] << 0, 0., 0.;
    p[1] << -side, -side, height;
    p[2] << -side,  side, height;
    p[3] << side,  side, height;
    p[4] << side, -side, height;
    p[5] << -side, -side, height;

    glBegin(GL_TRIANGLES);
    for (int i = 1; i < 5; ++i) {
      Eigen::Vector3f normal = (p[i] - p[0]).cross(p[i+1] - p[0]);
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
      _cameraZ = _previousParams->makeProperty<FloatProperty>(_typeName + "::CAMERA_Z", .05f);
      _cameraSide = _previousParams->makeProperty<FloatProperty>(_typeName + "::CAMERA_SIDE", .05f);
      
    } else {
      _cameraZ = 0;
      _cameraSide = 0;
    }
    return true;
  }

  HyperGraphElementAction* CacheCameraDrawAction::operator()(HyperGraph::HyperGraphElement* element, 
                 HyperGraphElementAction::Parameters* params){
    if (typeid(*element).name()!=_typeName)
      return nullptr;
    CacheCamera* that = static_cast<CacheCamera*>(element);
    refreshPropertyPtrs(params);
    if (! _previousParams)
      return this;
    
    if (_show && !_show->value())
      return this;

    glPushMatrix();
    glMultMatrixd(that->camParams()->offsetMatrix().data());
    if (_cameraZ && _cameraSide)
      drawMyPyramid(_cameraZ->value(), _cameraSide->value());
    glPopMatrix();

    return this;
  }
#endif

}
}
