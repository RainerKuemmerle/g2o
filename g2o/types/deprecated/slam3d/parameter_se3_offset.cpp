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

#include "parameter_se3_offset.h"

#include "vertex_se3_quat.h"
#include "g2o/stuff/opengl_wrapper.h"

using namespace Eigen;

namespace g2o {
namespace deprecated {

  ParameterSE3Offset::ParameterSE3Offset(){
    setOffset();
  }

  void ParameterSE3Offset::setOffset(const SE3Quat& offset_){
    _offset = offset_;
    _offsetMatrix= _offset.rotation().toRotationMatrix();
    _offsetMatrix.translation() = _offset.translation();
    _inverseOffsetMatrix = _offsetMatrix.inverse();
  }

  bool ParameterSE3Offset::read(std::istream& is) {
    Vector7 off;
    for (int i=0; i<7; i++) {
      is >> off[i];
      std::cerr << off[i] << " " ;
    }
    std::cerr <<  std::endl;
    setOffset(SE3Quat(off));
    return is.good();
  }
  
  bool ParameterSE3Offset::write(std::ostream& os) const {
    Vector7 off = _offset.toVector();
    for (int i=0; i<7; i++)
      os << off[i] << " ";
    return os.good();
  }

  CacheSE3Offset::CacheSE3Offset() :
    Cache(),
    _offsetParam(0)
  {
  }

  bool CacheSE3Offset::resolveDependancies(){
    _offsetParam = dynamic_cast <ParameterSE3Offset*> (_parameters[0]);
    return _offsetParam != 0;
  }

  void CacheSE3Offset::updateImpl(){
    const VertexSE3* v = static_cast<const VertexSE3*>(vertex());
    _se3_n2w = v->estimate() * _offsetParam->offset();

    _n2w = _se3_n2w.rotation().toRotationMatrix();
    _n2w.translation() = _se3_n2w.translation();

    _se3_w2n = _se3_n2w.inverse();
    _w2n = _se3_w2n.rotation().toRotationMatrix();
    _w2n.translation() = _se3_w2n.translation();

    SE3Quat w2l = v->estimate().inverse();
    _w2l = w2l.rotation().toRotationMatrix();
    _w2l.translation() = w2l.translation();
  }  

  void CacheSE3Offset::setOffsetParam(ParameterSE3Offset* offsetParam)
  {
    _offsetParam = offsetParam;
  }

#ifdef G2O_HAVE_OPENGL
  CacheSE3OffsetDrawAction::CacheSE3OffsetDrawAction(): DrawAction(typeid(CacheSE3Offset).name()){
    _previousParams = (DrawAction::Parameters*)0x42;
    refreshPropertyPtrs(0);
  }


  bool CacheSE3OffsetDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_){
    if (! DrawAction::refreshPropertyPtrs(params_))
      return false;
    if (_previousParams){
      _cubeSide = _previousParams->makeProperty<FloatProperty>(_typeName + "::CUBE_SIDE", .05f);
    } else {
      _cubeSide = 0;
    }
    return true;
  }

  HyperGraphElementAction* CacheSE3OffsetDrawAction::operator()(HyperGraph::HyperGraphElement* element, 
                HyperGraphElementAction::Parameters* params){
    if (typeid(*element).name()!=_typeName)
      return nullptr;
    CacheSE3Offset* that = static_cast<CacheSE3Offset*>(element);
    refreshPropertyPtrs(params);
    if (! _previousParams)
      return this;
    
    if (_show && !_show->value())
      return this;

    glPushMatrix();
    const Vector3& offsetT=that->offsetParam()->offset().translation();
    AngleAxis aa(that->offsetParam()->offset().rotation());
    glTranslatef((float)offsetT.x(), (float)offsetT.y(), (float)offsetT.z());
    glRotatef((float)RAD2DEG(aa.angle()),(float)aa.axis().x(),(float)aa.axis().y(),(float)aa.axis().z());
    // if (_cubeSide)
    //   drawMyPyramid(_cubeSide->value(), _cubeSide->value());
    glPopMatrix();

    return this;
  }
#endif

} // end namespace
} // end namespace
