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

#include "isometry3d_mappings.h"
#include "dquat2mat.h"

#include <Eigen/Core>

namespace g2o {
  namespace internal {
    // forward declaration
    /* void G2O_TYPES_SLAM3D_API compute_dq_dR (Eigen::Matrix<number_t, 3 , 9, Eigen::ColMajor>&  dq_dR , const number_t&  r11 , const number_t&  r21 , const number_t&  r31 , const number_t&  r12 , const number_t&  r22 , const number_t&  r32 , const number_t&  r13 , const number_t&  r23 , const number_t&  r33 );  */

    template <typename Derived, typename DerivedOther, bool transposed = false>
    inline void skew(Eigen::MatrixBase<Derived>& s, const Eigen::MatrixBase<DerivedOther>& v){
      const number_t x=2*v(0);
      const number_t y=2*v(1);
      const number_t z=2*v(2);
      if (transposed)
        s << 0., -z, y, z, 0, -x, -y, x, 0;
      else
        s << 0., z, -y, -z, 0, x, y, -x, 0;
    }

    template <typename Derived, typename DerivedOther>
    inline void skewT(Eigen::MatrixBase<Derived>& s, const Eigen::MatrixBase<DerivedOther>& v){
      skew<Derived, DerivedOther, true>(s, v);
    }

    template <typename Derived, typename DerivedOther, bool transposed = false>
    void skew(Eigen::MatrixBase<Derived>& Sx,
        Eigen::MatrixBase<Derived>& Sy,
        Eigen::MatrixBase<Derived>& Sz,
        const Eigen::MatrixBase<DerivedOther>& R){
      const number_t
        r11=2*R(0,0), r12=2*R(0,1), r13=2*R(0,2),
        r21=2*R(1,0), r22=2*R(1,1), r23=2*R(1,2),
        r31=2*R(2,0), r32=2*R(2,1), r33=2*R(2,2);
      if (transposed) {
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
        const Eigen::MatrixBase<DerivedOther>& R){
      skew<Derived, DerivedOther, true>(Sx, Sy, Sz, R);
    }

  template <typename Derived>
  void computeEdgeSE3Gradient(Isometry3& E,
                              Eigen::MatrixBase<Derived> const & JiConstRef,
                              Eigen::MatrixBase<Derived> const & JjConstRef,
                              const Isometry3& Z,
                              const Isometry3& Xi,
                              const Isometry3& Xj,
                              const Isometry3& Pi/*=Isometry3()*/,
                              const Isometry3& Pj/*=Isometry3()*/)
  {
    Eigen::MatrixBase<Derived>& Ji = const_cast<Eigen::MatrixBase<Derived>&>(JiConstRef);
    Eigen::MatrixBase<Derived>& Jj = const_cast<Eigen::MatrixBase<Derived>&>(JjConstRef);
    Ji.derived().resize(6,6);
    Jj.derived().resize(6,6);
    // compute the error at the linearization point
    const Isometry3 A=Z.inverse()*Pi.inverse();
    const Isometry3 B=Xi.inverse()*Xj;
    const Isometry3& C=Pj;

    const Isometry3 AB=A*B;
    const Isometry3 BC=B*C;
    E=AB*C;

    Isometry3::ConstLinearPart Re = extractRotation(E);
    Isometry3::ConstLinearPart Ra = extractRotation(A);
    //const Matrix3 Rb = extractRotation(B);
    Isometry3::ConstLinearPart Rc = extractRotation(C);
    Isometry3::ConstTranslationPart tc = C.translation();
    //Isometry3::ConstTranslationParttab=AB.translation();
    Isometry3::ConstLinearPart Rab = extractRotation(AB);
    Isometry3::ConstTranslationPart tbc = BC.translation();
    Isometry3::ConstLinearPart Rbc = extractRotation(BC);

    Eigen::Matrix<number_t, 3 , 9, Eigen::ColMajor>  dq_dR;
    compute_dq_dR (dq_dR,
        Re(0,0),Re(1,0),Re(2,0),
        Re(0,1),Re(1,1),Re(2,1),
        Re(0,2),Re(1,2),Re(2,2));

    Ji.setZero();
    Jj.setZero();

    // dte/dti
    Ji.template block<3,3>(0,0)=-Ra;

    // dte/dtj
    Jj.template block<3,3>(0,0)=Rab;

    // dte/dqi
    {
      Matrix3 S;
      skewT(S,tbc);
      Ji.template block<3,3>(0,3)=Ra*S;
    }

    // dte/dqj
    {
      Matrix3 S;
      skew(S,tc);
      Jj.template block<3,3>(0,3)=Rab*S;
    }

    // dre/dqi
    {
      number_t buf[27];
      Eigen::Map<Eigen::Matrix<number_t, 9, 3, Eigen::ColMajor> > M(buf);
      Matrix3 Sxt,Syt,Szt;
      internal::skewT(Sxt,Syt,Szt,Rbc);
#ifdef __clang__
      Matrix3 temp = Rab * Sxt;
      Eigen::Map<Matrix3> M2(temp.data());
      Eigen::Map<Matrix3> Mx(buf);    Mx = M2;
      temp = Ra*Syt;
      Eigen::Map<Matrix3> My(buf+9);  My = M2;
      temp = Ra*Szt;
      Eigen::Map<Matrix3> Mz(buf+18); Mz = M2;
#else
      Eigen::Map<Matrix3> Mx(buf);    Mx = Ra*Sxt;
      Eigen::Map<Matrix3> My(buf+9);  My = Ra*Syt;
      Eigen::Map<Matrix3> Mz(buf+18); Mz = Ra*Szt;
#endif
      Ji.template block<3,3>(3,3) = dq_dR * M;
    }

    // dre/dqj
    {
      number_t buf[27];
      Eigen::Map <Eigen::Matrix<number_t, 9, 3, Eigen::ColMajor> > M(buf);
      Matrix3 Sx,Sy,Sz;
      internal::skew(Sx,Sy,Sz,Rc);
#ifdef __clang__
      Matrix3 temp = Rab * Sx;
      Eigen::Map<Matrix3> M2(temp.data());
      Eigen::Map<Matrix3> Mx(buf);    Mx = M2;
      temp = Rab*Sy;
      Eigen::Map<Matrix3> My(buf+9);  My = M2;
      temp = Rab*Sz;
      Eigen::Map<Matrix3> Mz(buf+18); Mz = M2;
#else
      Eigen::Map<Matrix3> Mx(buf);    Mx = Rab*Sx;
      Eigen::Map<Matrix3> My(buf+9);  My = Rab*Sy;
      Eigen::Map<Matrix3> Mz(buf+18); Mz = Rab*Sz;
#endif
      Jj.template block<3,3>(3,3) = dq_dR * M;
    }
  }

  template <typename Derived>
  void computeEdgeSE3Gradient(Isometry3& E,
                              Eigen::MatrixBase<Derived> const & JiConstRef,
                              Eigen::MatrixBase<Derived> const & JjConstRef,
                              const Isometry3& Z,
                              const Isometry3& Xi,
                              const Isometry3& Xj)
  {
    Eigen::MatrixBase<Derived>& Ji = const_cast<Eigen::MatrixBase<Derived>&>(JiConstRef);
    Eigen::MatrixBase<Derived>& Jj = const_cast<Eigen::MatrixBase<Derived>&>(JjConstRef);
    Ji.derived().resize(6,6);
    Jj.derived().resize(6,6);
    // compute the error at the linearization point
    const Isometry3 A=Z.inverse();
    const Isometry3 B=Xi.inverse()*Xj;

    E=A*B;

    Isometry3::ConstLinearPart Re = extractRotation(E);
    Isometry3::ConstLinearPart Ra = extractRotation(A);
    Isometry3::ConstLinearPart Rb = extractRotation(B);
    Isometry3::ConstTranslationPart tb = B.translation();

    Eigen::Matrix<number_t, 3, 9, Eigen::ColMajor>  dq_dR;
    compute_dq_dR (dq_dR,
        Re(0,0),Re(1,0),Re(2,0),
        Re(0,1),Re(1,1),Re(2,1),
        Re(0,2),Re(1,2),Re(2,2));

    Ji.setZero();
    Jj.setZero();

    // dte/dti
    Ji.template block<3,3>(0,0)=-Ra;

    // dte/dtj
    Jj.template block<3,3>(0,0)=Re;

    // dte/dqi
    {
      Matrix3 S;
      skewT(S,tb);
      Ji.template block<3,3>(0,3)=Ra*S;
    }

    // dte/dqj: this is zero

    number_t buf[27];
    Eigen::Map<Eigen::Matrix<number_t, 9, 3, Eigen::ColMajor> > M(buf);
    Matrix3 Sxt,Syt,Szt;
    // dre/dqi
    {
      skewT(Sxt,Syt,Szt,Rb);
      Eigen::Map<Matrix3> Mx(buf);    Mx.noalias() = Ra*Sxt;
      Eigen::Map<Matrix3> My(buf+9);  My.noalias() = Ra*Syt;
      Eigen::Map<Matrix3> Mz(buf+18); Mz.noalias() = Ra*Szt;
      Ji.template block<3,3>(3,3) = dq_dR * M;
    }

    // dre/dqj
    {
      Matrix3& Sx = Sxt;
      Matrix3& Sy = Syt;
      Matrix3& Sz = Szt;
      skew(Sx,Sy,Sz,Matrix3::Identity());
      Eigen::Map<Matrix3> Mx(buf);    Mx.noalias() = Re*Sx;
      Eigen::Map<Matrix3> My(buf+9);  My.noalias() = Re*Sy;
      Eigen::Map<Matrix3> Mz(buf+18); Mz.noalias() = Re*Sz;
      Jj.template block<3,3>(3,3) = dq_dR * M;
    }
  }


  template <typename Derived>
  void computeEdgeSE3PriorGradient(Isometry3& E,
                                   const Eigen::MatrixBase<Derived>& JConstRef,
                                   const Isometry3& Z,
                                   const Isometry3& X,
                                   const Isometry3& P=Isometry3::Identity())
  {
    Eigen::MatrixBase<Derived>& J = const_cast<Eigen::MatrixBase<Derived>&>(JConstRef);
    J.derived().resize(6,6);
    // compute the error at the linearization point
    const Isometry3 A = Z.inverse()*X;
    const Isometry3& B = P;
    Isometry3::ConstLinearPart Ra = extractRotation(A);
    Isometry3::ConstLinearPart Rb = extractRotation(B);
    Isometry3::ConstTranslationPart tb = B.translation();
    E = A*B;
    Isometry3::ConstLinearPart Re = extractRotation(E);

    Eigen::Matrix<number_t, 3, 9, Eigen::ColMajor> dq_dR;
    compute_dq_dR (dq_dR,
        Re(0,0),Re(1,0),Re(2,0),
        Re(0,1),Re(1,1),Re(2,1),
        Re(0,2),Re(1,2),Re(2,2));

    J.setZero();

    // dte/dt
    J.template block<3,3>(0,0)=Ra;

    // dte/dq =0
    // dte/dqj
    {
      Matrix3 S;
      skew(S,tb);
      J.template block<3,3>(0,3)=Ra*S;
    }

    // dre/dt =0

    // dre/dq
    {
      number_t buf[27];
      Eigen::Map<Eigen::Matrix<number_t, 9, 3, Eigen::ColMajor> > M(buf);
      Matrix3 Sx,Sy,Sz;
      internal::skew(Sx,Sy,Sz,Rb);
#ifdef __clang__
      Matrix3 temp = Ra * Sx;
      Eigen::Map<Matrix3> M2(temp.data());
      Eigen::Map<Matrix3> Mx(buf);    Mx = M2;
      temp = Ra*Sy;
      Eigen::Map<Matrix3> My(buf+9);  My = M2;
      temp = Ra*Sz;
      Eigen::Map<Matrix3> Mz(buf+18); Mz = M2;
#else
      Eigen::Map<Matrix3> Mx(buf);    Mx = Ra*Sx;
      Eigen::Map<Matrix3> My(buf+9);  My = Ra*Sy;
      Eigen::Map<Matrix3> Mz(buf+18); Mz = Ra*Sz;
#endif
      J.template block<3,3>(3,3) = dq_dR * M;
    }

  }

} // end namespace internal
} // end namespace g2o
#endif
