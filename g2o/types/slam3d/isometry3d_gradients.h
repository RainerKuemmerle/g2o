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

#ifndef G2O_ISOMETRY3D_GRADIENTS_H_
#define G2O_ISOMETRY3D_GRADIENTS_H_

#include <Eigen/Core>

#include "dquat2mat.h"
#include "isometry3d_mappings.h"

namespace g2o::internal {

namespace {
using ColMajor33 = Eigen::Matrix<double, 3, 3, Eigen::ColMajor>;
}

// forward declaration
/* void G2O_TYPES_SLAM3D_API compute_dq_dR (Eigen::Matrix<double, 3 , 9,
 * Eigen::ColMajor>&  dq_dR , const double&  r11 , const double&  r21 ,
 * const double&  r31 , const double&  r12 , const double&  r22 , const
 * double&  r32 , const double&  r13 , const double&  r23 , const
 * double&  r33 );  */

template <typename Derived, typename DerivedOther, bool Transposed = false>
inline void skew(Eigen::MatrixBase<Derived>& s,
                 const Eigen::MatrixBase<DerivedOther>& v) {
  const double x = 2 * v(0);
  const double y = 2 * v(1);
  const double z = 2 * v(2);
  if (Transposed)
    s << 0., -z, y, z, 0, -x, -y, x, 0;
  else
    s << 0., z, -y, -z, 0, x, y, -x, 0;
}

template <typename Derived, typename DerivedOther>
inline void skewT(Eigen::MatrixBase<Derived>& s,
                  const Eigen::MatrixBase<DerivedOther>& v) {
  skew<Derived, DerivedOther, true>(s, v);
}

template <typename Derived, typename DerivedOther, bool Transposed = false>
void skew(Eigen::MatrixBase<Derived>& Sx, Eigen::MatrixBase<Derived>& Sy,
          Eigen::MatrixBase<Derived>& Sz,
          const Eigen::MatrixBase<DerivedOther>& R) {
  const double r11 = 2 * R(0, 0);
  const double r12 = 2 * R(0, 1);
  const double r13 = 2 * R(0, 2);
  const double r21 = 2 * R(1, 0);
  const double r22 = 2 * R(1, 1);
  const double r23 = 2 * R(1, 2);
  const double r31 = 2 * R(2, 0);
  const double r32 = 2 * R(2, 1);
  const double r33 = 2 * R(2, 2);
  if (Transposed) {
    Sx << 0, 0, 0, r31, r32, r33, -r21, -r22, -r23;
    Sy << -r31, -r32, -r33, 0, 0, 0, r11, r12, r13;
    Sz << r21, r22, r23, -r11, -r12, -r13, 0, 0, 0;
  } else {
    Sx << 0, 0, 0, -r31, -r32, -r33, r21, r22, r23;
    Sy << r31, r32, r33, 0, 0, 0, -r11, -r12, -r13;
    Sz << -r21, -r22, -r23, r11, r12, r13, 0, 0, 0;
  }
}

template <typename Derived, typename DerivedOther>
inline void skewT(Eigen::MatrixBase<Derived>& Sx,
                  Eigen::MatrixBase<Derived>& Sy,
                  Eigen::MatrixBase<Derived>& Sz,
                  const Eigen::MatrixBase<DerivedOther>& R) {
  skew<Derived, DerivedOther, true>(Sx, Sy, Sz, R);
}

template <typename Derived>
void computeEdgeSE3Gradient(Isometry3& E,
                            Eigen::MatrixBase<Derived> const& JiConstRef,
                            Eigen::MatrixBase<Derived> const& JjConstRef,
                            const Isometry3& Z, const Isometry3& Xi,
                            const Isometry3& Xj,
                            const Isometry3& Pi /*=Isometry3()*/,
                            const Isometry3& Pj /*=Isometry3()*/) {
  auto& Ji = const_cast<Eigen::MatrixBase<Derived>&>(JiConstRef);
  auto& Jj = const_cast<Eigen::MatrixBase<Derived>&>(JjConstRef);
  Ji.derived().resize(6, 6);
  Jj.derived().resize(6, 6);
  // compute the error at the linearization point
  const Isometry3 A = Z.inverse() * Pi.inverse();
  const Isometry3 B = Xi.inverse() * Xj;
  const Isometry3& C = Pj;

  const Isometry3 AB = A * B;
  const Isometry3 BC = B * C;
  E = AB * C;

  Isometry3::ConstLinearPart Re = extractRotation(E);
  Isometry3::ConstLinearPart Ra = extractRotation(A);
  // const Matrix3 Rb = extractRotation(B);
  Isometry3::ConstLinearPart Rc = extractRotation(C);
  Isometry3::ConstTranslationPart tc = C.translation();
  // Isometry3::ConstTranslationParttab=AB.translation();
  Isometry3::ConstLinearPart Rab = extractRotation(AB);
  Isometry3::ConstTranslationPart tbc = BC.translation();
  Isometry3::ConstLinearPart Rbc = extractRotation(BC);

  Eigen::Matrix<double, 3, 9, Eigen::ColMajor> dq_dR;
  compute_dq_dR(dq_dR, Re(0, 0), Re(1, 0), Re(2, 0), Re(0, 1), Re(1, 1),
                Re(2, 1), Re(0, 2), Re(1, 2), Re(2, 2));

  Ji.setZero();
  Jj.setZero();

  // dte/dti
  Ji.template block<3, 3>(0, 0) = -Ra;

  // dte/dtj
  Jj.template block<3, 3>(0, 0) = Rab;

  // dte/dqi
  {
    Matrix3 S;
    skewT(S, tbc);
    Ji.template block<3, 3>(0, 3) = Ra * S;
  }

  // dte/dqj
  {
    Matrix3 S;
    skew(S, tc);
    Jj.template block<3, 3>(0, 3) = Rab * S;
  }

  // dre/dqi
  {
    Eigen::Matrix<double, 9, 3, Eigen::ColMajor> M;
    Matrix3 Sxt;
    Matrix3 Syt;
    Matrix3 Szt;
    internal::skewT(Sxt, Syt, Szt, Rbc);
    ColMajor33::MapType Mx(M.data());
    Mx = Ra * Sxt;
    ColMajor33::MapType My(M.data() + 9);
    My = Ra * Syt;
    ColMajor33::MapType Mz(M.data() + 18);
    Mz = Ra * Szt;
    Ji.template block<3, 3>(3, 3) = dq_dR * M;
  }

  // dre/dqj
  {
    Eigen::Matrix<double, 9, 3, Eigen::ColMajor> M;
    Matrix3 Sx;
    Matrix3 Sy;
    Matrix3 Sz;
    internal::skew(Sx, Sy, Sz, Rc);
    ColMajor33::MapType Mx(M.data());
    Mx = Rab * Sx;
    ColMajor33::MapType My(M.data() + 9);
    My = Rab * Sy;
    ColMajor33::MapType Mz(M.data() + 18);
    Mz = Rab * Sz;
    Jj.template block<3, 3>(3, 3) = dq_dR * M;
  }
}

template <typename Derived>
void computeEdgeSE3Gradient(Isometry3& E,
                            Eigen::MatrixBase<Derived> const& JiConstRef,
                            Eigen::MatrixBase<Derived> const& JjConstRef,
                            const Isometry3& Z, const Isometry3& Xi,
                            const Isometry3& Xj) {
  auto& Ji = const_cast<Eigen::MatrixBase<Derived>&>(JiConstRef);
  auto& Jj = const_cast<Eigen::MatrixBase<Derived>&>(JjConstRef);
  Ji.derived().resize(6, 6);
  Jj.derived().resize(6, 6);
  // compute the error at the linearization point
  const Isometry3 A = Z.inverse();
  const Isometry3 B = Xi.inverse() * Xj;

  E = A * B;

  Isometry3::ConstLinearPart Re = extractRotation(E);
  Isometry3::ConstLinearPart Ra = extractRotation(A);
  Isometry3::ConstLinearPart Rb = extractRotation(B);
  Isometry3::ConstTranslationPart tb = B.translation();

  Eigen::Matrix<double, 3, 9, Eigen::ColMajor> dq_dR;
  compute_dq_dR(dq_dR, Re(0, 0), Re(1, 0), Re(2, 0), Re(0, 1), Re(1, 1),
                Re(2, 1), Re(0, 2), Re(1, 2), Re(2, 2));

  Ji.setZero();
  Jj.setZero();

  // dte/dti
  Ji.template block<3, 3>(0, 0) = -Ra;

  // dte/dtj
  Jj.template block<3, 3>(0, 0) = Re;

  // dte/dqi
  {
    Matrix3 S;
    skewT(S, tb);
    Ji.template block<3, 3>(0, 3) = Ra * S;
  }

  // dte/dqj: this is zero

  Eigen::Matrix<double, 9, 3, Eigen::ColMajor> M;
  Matrix3 Sxt;
  Matrix3 Syt;
  Matrix3 Szt;
  // dre/dqi
  {
    skewT(Sxt, Syt, Szt, Rb);
    ColMajor33::MapType Mx(M.data());
    Mx.noalias() = Ra * Sxt;
    ColMajor33::MapType My(M.data() + 9);
    My.noalias() = Ra * Syt;
    ColMajor33::MapType Mz(M.data() + 18);
    Mz.noalias() = Ra * Szt;
    Ji.template block<3, 3>(3, 3) = dq_dR * M;
  }

  // dre/dqj
  {
    Matrix3& Sx = Sxt;
    Matrix3& Sy = Syt;
    Matrix3& Sz = Szt;
    skew(Sx, Sy, Sz, Matrix3::Identity());
    ColMajor33::MapType Mx(M.data());
    Mx.noalias() = Re * Sx;
    ColMajor33::MapType My(M.data() + 9);
    My.noalias() = Re * Sy;
    ColMajor33::MapType Mz(M.data() + 18);
    Mz.noalias() = Re * Sz;
    Jj.template block<3, 3>(3, 3) = dq_dR * M;
  }
}

template <typename Derived>
void computeEdgeSE3PriorGradient(Isometry3& E,
                                 const Eigen::MatrixBase<Derived>& JConstRef,
                                 const Isometry3& Z, const Isometry3& X,
                                 const Isometry3& P = Isometry3::Identity()) {
  auto& J = const_cast<Eigen::MatrixBase<Derived>&>(JConstRef);
  J.derived().resize(6, 6);
  // compute the error at the linearization point
  const Isometry3 A = Z.inverse() * X;
  const Isometry3& B = P;
  Isometry3::ConstLinearPart Ra = extractRotation(A);
  Isometry3::ConstLinearPart Rb = extractRotation(B);
  Isometry3::ConstTranslationPart tb = B.translation();
  E = A * B;
  Isometry3::ConstLinearPart Re = extractRotation(E);

  Eigen::Matrix<double, 3, 9, Eigen::ColMajor> dq_dR;
  compute_dq_dR(dq_dR, Re(0, 0), Re(1, 0), Re(2, 0), Re(0, 1), Re(1, 1),
                Re(2, 1), Re(0, 2), Re(1, 2), Re(2, 2));

  J.setZero();

  // dte/dt
  J.template block<3, 3>(0, 0) = Ra;

  // dte/dq =0
  // dte/dqj
  {
    Matrix3 S;
    skew(S, tb);
    J.template block<3, 3>(0, 3) = Ra * S;
  }

  // dre/dt =0

  // dre/dq
  {
    Eigen::Matrix<double, 9, 3, Eigen::ColMajor> M;
    Matrix3 Sx;
    Matrix3 Sy;
    Matrix3 Sz;
    internal::skew(Sx, Sy, Sz, Rb);
    ColMajor33::MapType Mx(M.data() + M.rows() * 0);
    Mx = (Ra * Sx);
    ColMajor33::MapType My(M.data() + M.rows() * 1);
    My = (Ra * Sy);
    ColMajor33::MapType Mz(M.data() + M.rows() * 2);
    Mz = (Ra * Sz);
    J.template block<3, 3>(3, 3) = dq_dR * M;
  }
}

}  // namespace g2o::internal
#endif
