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

#ifndef G2O_SBA_CAMERAPARAMETERS_H
#define G2O_SBA_CAMERAPARAMETERS_H

#include <Eigen/Core>

#include "g2o/core/eigen_types.h"
#include "g2o/core/parameter.h"
#include "g2o_types_sba_api.h"

namespace g2o {

class G2O_TYPES_SBA_API CameraParameters : public g2o::Parameter {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  CameraParameters();
  CameraParameters(number_t focal_length, const Vector2 &principle_point, number_t baseline);

  Vector2 cam_map(const Vector3 &trans_xyz) const;
  Vector3 stereocam_uvu_map(const Vector3 &trans_xyz) const;
  bool read(std::istream &is);
  bool write(std::ostream &os) const;

  number_t focal_length;
  Vector2 principle_point;
  number_t baseline;
};

}  // namespace g2o

#endif
