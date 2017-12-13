// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, H. Strasdat, W. Burgard
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

#ifndef _DQUAT2MAT_H_
#define _DQUAT2MAT_H_
#include <Eigen/Core>
#include "g2o_types_slam3d_api.h"

namespace g2o {
  namespace internal {

    void  G2O_TYPES_SLAM3D_API compute_dq_dR ( Eigen::Matrix<number_t, 3 , 9, Eigen::ColMajor>&  dq_dR , const number_t&  r11 , const number_t&  r21 , const number_t&  r31 , const number_t&  r12 , const number_t&  r22 , const number_t&  r32 , const number_t&  r13 , const number_t&  r23 , const number_t&  r33 );

    void  G2O_TYPES_SLAM3D_API compute_dR_dq ( Eigen::Matrix<number_t, 9 , 3, Eigen::ColMajor>&  dR_dq , const number_t&  qx , const number_t&  qy , const number_t&  qz , const number_t&  qw ) ;
  }
}
#endif
