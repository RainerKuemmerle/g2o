#include "parameter_se3_offset.h"

#include "vertex_se3_quat.h"

#ifdef G2O_HAVE_OPENGL
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#endif

namespace g2o {

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
    Vector7d off;
    for (int i=0; i<7; i++) {
      is >> off[i];
      std::cerr << off[i] << " " ;
    }
    std::cerr <<  std::endl;
    setOffset(SE3Quat(off));
    return is.good();
  }
  
  bool ParameterSE3Offset::write(std::ostream& os) const {
    Vector7d off = _offset.toVector();
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
      _cubeSide = _previousParams->makeProperty<FloatProperty>(_typeName + "::CUBE_SIDE", .05);
    } else {
      _cubeSide = 0;
    }
    return true;
  }

  HyperGraphElementAction* CacheSE3OffsetDrawAction::operator()(HyperGraph::HyperGraphElement* element, 
                HyperGraphElementAction::Parameters* params){
    if (typeid(*element).name()!=_typeName)
      return 0;
    CacheSE3Offset* that = static_cast<CacheSE3Offset*>(element);
    refreshPropertyPtrs(params);
    if (! _previousParams)
      return this;
    
    if (_show && !_show->value())
      return this;

    glPushMatrix();
    const Vector3d& offsetT=that->offsetParam()->offset().translation();
    AngleAxisd aa(that->offsetParam()->offset().rotation());
    glTranslatef(offsetT.x(), offsetT.y(), offsetT.z());
    glRotatef(RAD2DEG(aa.angle()),aa.axis().x(),aa.axis().y(),aa.axis().z());
    // if (_cubeSide)
    //   drawMyPyramid(_cubeSide->value(), _cubeSide->value());
    glPopMatrix();

    return this;
  }
#endif

} // end namespace
