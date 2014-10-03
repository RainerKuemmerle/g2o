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

#include "parameter_se2_offset.h"
#include "vertex_se2.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#endif

namespace g2o {

  ParameterSE2Offset::ParameterSE2Offset(){
    setOffset();
  }

  void ParameterSE2Offset::setOffset(const SE2& offset_){
    _offset = offset_;
    _offsetMatrix= _offset.rotation().toRotationMatrix();
    _offsetMatrix.translation() = _offset.translation();
    _inverseOffsetMatrix = _offsetMatrix.inverse();
  }

  bool ParameterSE2Offset::read(std::istream& is) {
    Vector3D off;
    for (int i=0; i<3; i++) {
      is >> off[i];
      std::cerr << off[i] << " " ;
    }
    std::cerr <<  std::endl;
    setOffset(SE2(off));
    return is.good() || is.eof();
  }
  
  bool ParameterSE2Offset::write(std::ostream& os) const {
    Vector3D off = _offset.toVector();
    for (int i=0; i<3; i++)
      os << off[i] << " ";
    return os.good();
  }

  CacheSE2Offset::CacheSE2Offset() :
    Cache(),
    _offsetParam(0)
  {
  }

  bool CacheSE2Offset::resolveDependancies(){
    _offsetParam = dynamic_cast <ParameterSE2Offset*> (_parameters[0]);
    return _offsetParam != 0;
  }

  void CacheSE2Offset::updateImpl(){
    const VertexSE2* v = static_cast<const VertexSE2*>(vertex());
    _se2_n2w = v->estimate() * _offsetParam->offset();

    _n2w = _se2_n2w.rotation().toRotationMatrix();
    _n2w.translation() = _se2_n2w.translation();

    _se2_w2n = _se2_n2w.inverse();
    _w2n = _se2_w2n.rotation().toRotationMatrix();
    _w2n.translation() = _se2_w2n.translation();

    SE2 w2l = v->estimate().inverse();
    _w2l = w2l.rotation().toRotationMatrix();
    _w2l.translation() = w2l.translation();

    double alpha=v->estimate().rotation().angle();
    double c=cos(alpha), s=sin(alpha);
    Matrix2D RInversePrime;
    RInversePrime << -s, c, -c, -s;
    _RpInverse_RInversePrime = _offsetParam->offset().rotation().toRotationMatrix().transpose()*RInversePrime;
    _RpInverse_RInverse=w2l.rotation();
  }  

  void CacheSE2Offset::setOffsetParam(ParameterSE2Offset* offsetParam)
  {
    _offsetParam = offsetParam;
  }


} // end namespace
