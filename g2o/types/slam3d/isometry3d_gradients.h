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

#ifndef G2O_ISOMETRY3D_GRADIENTS_H
#define G2O_ISOMETRY3D_GRADIENTS_H_
#include <Eigen/Geometry>
namespace g2o {
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;
  void computeEdgeSE3Gradient(Eigen::Isometry3d& E,
                              Matrix6d& Ji, 
                              Matrix6d& Jj,
                              const Eigen::Isometry3d& Z, 
                              const Eigen::Isometry3d& Xi,
                              const Eigen::Isometry3d& Xj,
                              const Eigen::Isometry3d& Pi=Eigen::Isometry3d(), 
                              const Eigen::Isometry3d& Pj=Eigen::Isometry3d());

void computeEdgeSE3PriorGradient(Eigen::Isometry3d& E,
                                 Matrix6d& J, 
                                 const Eigen::Isometry3d& Z, 
                                 const Eigen::Isometry3d& X,
                                 const Eigen::Isometry3d& P=Eigen::Isometry3d());

}
#endif
