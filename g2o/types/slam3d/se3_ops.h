// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
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

#ifndef G2O_MATH_STUFF
#define G2O_MATH_STUFF

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "g2o_types_slam3d_api.h"

namespace g2o {

  inline G2O_TYPES_SLAM3D_API Matrix3D skew(const Vector3D&v);
  inline G2O_TYPES_SLAM3D_API Vector3D deltaR(const Matrix3D& R);
  inline G2O_TYPES_SLAM3D_API Vector2D project(const Vector3D&);
  inline G2O_TYPES_SLAM3D_API Vector3D project(const Vector4D&);
  inline G2O_TYPES_SLAM3D_API Vector3D unproject(const Vector2D&);
  inline G2O_TYPES_SLAM3D_API Vector4D unproject(const Vector3D&);

  #include "se3_ops.hpp"

}

#endif //MATH_STUFF
