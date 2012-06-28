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

#include "parameter_stereo_camera.h"
#include "isometry3d_gradients.h"
#include "isometry3d_mappings.h"

#ifdef WINDOWS
#include <windows.h>
#endif

using namespace std;

namespace g2o {

  ParameterStereoCamera::ParameterStereoCamera(){
    setBaseline(0.075);
  }

  bool ParameterStereoCamera::read(std::istream& is) {
    Vector7d off;
    for (int i=0; i<7; i++)
      is >> off[i];
    // normalize the quaternion to recover numerical precision lost by storing as human readable text
    Vector4d::MapType(off.data()+3).normalize();
    setOffset(internal::fromVectorQT(off));
    double fx,fy,cx,cy;
    is >> fx >> fy >> cx >> cy;
    setKcam(fx,fy,cx,cy);
    double baseline_;
    is >> baseline_;
    setBaseline(baseline_);
    return is.good();
  }
  
  bool ParameterStereoCamera::write(std::ostream& os) const {
    Vector7d off = internal::toVectorQT(_offset);
    for (int i=0; i<7; i++)
      os << off[i] << " ";
    os << _Kcam(0,0) << " ";
    os << _Kcam(1,1) << " ";
    os << _Kcam(0,2) << " ";
    os << _Kcam(1,2) << " ";
    os << baseline() << " ";
    return os.good();
  }


}
