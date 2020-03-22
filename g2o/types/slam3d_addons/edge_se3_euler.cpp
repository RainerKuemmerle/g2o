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

#include "edge_se3_euler.h"
#include "g2o/core/factory.h"
#include <iostream>

using namespace Eigen;

namespace g2o
{

  /** conversion code from Euler angles */
static void jac_quat3_euler3(Eigen::Matrix<number_t, 6, 6, Eigen::ColMajor>& J, const Isometry3& t)
{
  Vector7 t0 = g2o::internal::toVectorQT(t);

  number_t delta= cst(1e-6);
  number_t idelta= 1 / (2 * delta);

  Vector7 ta = t0;
  Vector7 tb = t0;
  for (int i=0; i<6; i++){
    ta=tb=t0;
    ta[i]-=delta;
    tb[i]+=delta;
    Vector6 ea = g2o::internal::toVectorET(g2o::internal::fromVectorQT(ta));
    Vector6 eb = g2o::internal::toVectorET(g2o::internal::fromVectorQT(tb));
    J.col(i)=(eb-ea)*idelta;
  }
}


  bool EdgeSE3Euler::read(std::istream& is)
  {
    Vector6 meas;
    for (int i=0; i<6; i++)
      is  >> meas[i];
    Isometry3 transf= g2o::internal::fromVectorET(meas);
    Matrix<number_t, 6, 6, Eigen::ColMajor> infMatEuler;
    for (int i=0; i<6; i++)
      for (int j=i; j<6; j++) {
        is >> infMatEuler(i,j);
        if (i!=j)
          infMatEuler(j,i) = infMatEuler(i,j);
      }
    Matrix<number_t, 6, 6, Eigen::ColMajor> J;
    jac_quat3_euler3(J, transf);
    Matrix<number_t, 6, 6, Eigen::ColMajor> infMat = J.transpose() * infMatEuler * J;
    setMeasurement(transf);
    setInformation(infMat);
    return true;
  }

  bool EdgeSE3Euler::write(std::ostream& os) const
  {
    Vector6 meas = g2o::internal::toVectorET(_measurement);
    for (int i=0; i<6; i++)
      os << meas[i] << " ";

    Matrix<number_t, 6, 6, Eigen::ColMajor> J;
    jac_quat3_euler3(J, measurement());
    //HACK: invert the jacobian to simulate the inverse derivative
    J=J.inverse();
    Matrix<number_t, 6, 6, Eigen::ColMajor> infMatEuler = J.transpose()*information()*J;
    for (int i=0; i<6; i++)
      for (int j=i; j<6; j++){
        os << " " <<  infMatEuler(i,j);
      }
    return os.good();
  }

} // end namespace
