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

#include "g2o_types_slam3d_api.h"
#include "isometry3d_mappings.h"
#include "dquat2mat.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace g2o {
  namespace internal {
    // forward declaration
    /* void G2O_TYPES_SLAM3D_API compute_dq_dR (Eigen::Matrix<double, 3 , 9 >&  dq_dR , const double&  r11 , const double&  r21 , const double&  r31 , const double&  r12 , const double&  r22 , const double&  r32 , const double&  r13 , const double&  r23 , const double&  r33 );  */

    template <typename Derived, typename DerivedOther>
    inline void skew(Eigen::MatrixBase<Derived>& s, const Eigen::MatrixBase<DerivedOther>& v){
      const double x=2*v(0);
      const double y=2*v(1);
      const double z=2*v(2);
      s <<  0.,  z, -y, -z,  0,  x,  y, -x,  0;
    }

    template <typename Derived, typename DerivedOther>
    inline void skewT(Eigen::MatrixBase<Derived>& s, const Eigen::MatrixBase<DerivedOther>& v){
      const double x=2*v(0);
      const double y=2*v(1);
      const double z=2*v(2);
      s <<  0., -z,  y,  z,  0, -x,  -y,  x,  0;
    }

    template <typename Derived, typename DerivedOther>
    void skew(Eigen::MatrixBase<Derived>& Sx, 
        Eigen::MatrixBase<Derived>& Sy, 
        Eigen::MatrixBase<Derived>& Sz, 
        const Eigen::MatrixBase<DerivedOther>& R){
      const double 
        r11=2*R(0,0), r12=2*R(0,1), r13=2*R(0,2),
        r21=2*R(1,0), r22=2*R(1,1), r23=2*R(1,2),
        r31=2*R(2,0), r32=2*R(2,1), r33=2*R(2,2);
      Sx <<    0,    0,    0,  -r31, -r32, -r33,   r21,   r22,  r23;
      Sy <<  r31,  r32,  r33,     0,    0,    0,  -r11,  -r12, -r13;
      Sz << -r21, -r22, -r23,   r11,   r12, r13,     0,    0,    0;
    }

    template <typename Derived, typename DerivedOther>
    inline void skewT(Eigen::MatrixBase<Derived>& Sx, 
        Eigen::MatrixBase<Derived>& Sy, 
        Eigen::MatrixBase<Derived>& Sz, 
        const Eigen::MatrixBase<DerivedOther>& R){
      const double
        r11=2*R(0,0), r12=2*R(0,1), r13=2*R(0,2),
	r21=2*R(1,0), r22=2*R(1,1), r23=2*R(1,2),
	r31=2*R(2,0), r32=2*R(2,1), r33=2*R(2,2);
      Sx <<    0,    0,    0,   r31,  r32,   r33,  -r21,  -r22, -r23;
      Sy << -r31, -r32, -r33,     0,    0,     0,   r11,   r12,  r13;
      Sz <<  r21,  r22,  r23,  -r11,  -r12, -r13,     0,    0,    0;
    }

  template <typename Derived>
  void computeEdgeSE3Gradient(Eigen::Isometry3d& E,
                              Eigen::MatrixBase<Derived> const & JiConstRef, 
                              Eigen::MatrixBase<Derived> const & JjConstRef,
                              const Eigen::Isometry3d& Z, 
                              const Eigen::Isometry3d& Xi,
                              const Eigen::Isometry3d& Xj,
                              const Eigen::Isometry3d& Pi/*=Eigen::Isometry3d()*/, 
                              const Eigen::Isometry3d& Pj/*=Eigen::Isometry3d()*/)
  {
    using namespace Eigen;
    Eigen::MatrixBase<Derived>& Ji = const_cast<Eigen::MatrixBase<Derived>&>(JiConstRef);
    Eigen::MatrixBase<Derived>& Jj = const_cast<Eigen::MatrixBase<Derived>&>(JjConstRef);
    Ji.derived().resize(6,6);
    Jj.derived().resize(6,6);
    // compute the error at the linearization point
    const Isometry3d A=Z.inverse()*Pi.inverse();
    const Isometry3d B=Xi.inverse()*Xj;
    const Isometry3d& C=Pj;

    const Isometry3d AB=A*B;  
    const Isometry3d BC=B*C;
    E=AB*C;

    Eigen::Isometry3d::ConstLinearPart Re = extractRotation(E);
    Eigen::Isometry3d::ConstLinearPart Ra = extractRotation(A);
    //const Matrix3d Rb = extractRotation(B);
    Eigen::Isometry3d::ConstLinearPart Rc = extractRotation(C);
    Eigen::Isometry3d::ConstTranslationPart tc = C.translation();
    //Eigen::Isometry3d::ConstTranslationParttab=AB.translation();
    Eigen::Isometry3d::ConstLinearPart Rab = extractRotation(AB);
    Eigen::Isometry3d::ConstTranslationPart tbc = BC.translation();  
    Eigen::Isometry3d::ConstLinearPart Rbc = extractRotation(BC);

    Matrix<double, 3 , 9 >  dq_dR;
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
      Matrix3d S;
      skewT(S,tbc);
      Ji.template block<3,3>(0,3)=Ra*S;
    }

    // dte/dqj
    {
      Matrix3d S;
      skew(S,tc);
      Jj.template block<3,3>(0,3)=Rab*S;
    }

    // dre/dqi
    {
      double buf[27];
      Map<Matrix<double, 9,3> > M(buf);
      Matrix3d Sxt,Syt,Szt;
      internal::skewT(Sxt,Syt,Szt,Rbc);
#ifdef __clang__
      Matrix3d temp = Rab * Sxt;
      Map<Matrix3d> M2(temp.data());
      Map<Matrix3d> Mx(buf);    Mx = M2;
      temp = Ra*Syt;
      Map<Matrix3d> My(buf+9);  My = M2;
      temp = Ra*Szt;
      Map<Matrix3d> Mz(buf+18); Mz = M2;
#else
      Map<Matrix3d> Mx(buf);    Mx = Ra*Sxt;
      Map<Matrix3d> My(buf+9);  My = Ra*Syt;
      Map<Matrix3d> Mz(buf+18); Mz = Ra*Szt;
#endif
      Ji.template block<3,3>(3,3) = dq_dR * M;
    }

    // dre/dqj
    {
      double buf[27];
      Map <Matrix<double, 9,3> > M(buf);
      Matrix3d Sx,Sy,Sz;
      internal::skew(Sx,Sy,Sz,Rc);
#ifdef __clang__
      Matrix3d temp = Rab * Sx;
      Map<Matrix3d> M2(temp.data());
      Map<Matrix3d> Mx(buf);    Mx = M2;
      temp = Rab*Sy;
      Map<Matrix3d> My(buf+9);  My = M2;
      temp = Rab*Sz;
      Map<Matrix3d> Mz(buf+18); Mz = M2;
#else
      Map<Matrix3d> Mx(buf);    Mx = Rab*Sx;
      Map<Matrix3d> My(buf+9);  My = Rab*Sy;
      Map<Matrix3d> Mz(buf+18); Mz = Rab*Sz;
#endif
      Jj.template block<3,3>(3,3) = dq_dR * M;
    }
  }

  template <typename Derived>
  void computeEdgeSE3Gradient(Eigen::Isometry3d& E,
                              Eigen::MatrixBase<Derived> const & JiConstRef, 
                              Eigen::MatrixBase<Derived> const & JjConstRef,
                              const Eigen::Isometry3d& Z, 
                              const Eigen::Isometry3d& Xi,
                              const Eigen::Isometry3d& Xj)
  {
    using namespace Eigen;
    Eigen::MatrixBase<Derived>& Ji = const_cast<Eigen::MatrixBase<Derived>&>(JiConstRef);
    Eigen::MatrixBase<Derived>& Jj = const_cast<Eigen::MatrixBase<Derived>&>(JjConstRef);
    Ji.derived().resize(6,6);
    Jj.derived().resize(6,6);
    // compute the error at the linearization point
    const Isometry3d A=Z.inverse();
    const Isometry3d B=Xi.inverse()*Xj;

    E=A*B;

    Eigen::Isometry3d::ConstLinearPart Re = extractRotation(E);
    Eigen::Isometry3d::ConstLinearPart Ra = extractRotation(A);
    Eigen::Isometry3d::ConstLinearPart Rb = extractRotation(B);
    Eigen::Isometry3d::ConstTranslationPart tb = B.translation();  

    Matrix<double, 3 , 9 >  dq_dR;
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
      Matrix3d S;
      skewT(S,tb);
      Ji.template block<3,3>(0,3)=Ra*S;
    }

    // dte/dqj: this is zero

    double buf[27];
    Map<Matrix<double, 9,3> > M(buf);
    Matrix3d Sxt,Syt,Szt;
    // dre/dqi
    {
      skewT(Sxt,Syt,Szt,Rb);
      Map<Matrix3d> Mx(buf);    Mx.noalias() = Ra*Sxt;
      Map<Matrix3d> My(buf+9);  My.noalias() = Ra*Syt;
      Map<Matrix3d> Mz(buf+18); Mz.noalias() = Ra*Szt;
      Ji.template block<3,3>(3,3) = dq_dR * M;
    }

    // dre/dqj
    {
      Matrix3d& Sx = Sxt;
      Matrix3d& Sy = Syt;
      Matrix3d& Sz = Szt;
      skew(Sx,Sy,Sz,Matrix3d::Identity());
      Map<Matrix3d> Mx(buf);    Mx.noalias() = Re*Sx;
      Map<Matrix3d> My(buf+9);  My.noalias() = Re*Sy;
      Map<Matrix3d> Mz(buf+18); Mz.noalias() = Re*Sz;
      Jj.template block<3,3>(3,3) = dq_dR * M;
    }
  }


  template <typename Derived>
  void computeEdgeSE3PriorGradient(Eigen::Isometry3d& E,
                                   const Eigen::MatrixBase<Derived>& JConstRef, 
                                   const Eigen::Isometry3d& Z, 
                                   const Eigen::Isometry3d& X,
                                   const Eigen::Isometry3d& P=Eigen::Isometry3d())
  {
    using namespace Eigen;
    Eigen::MatrixBase<Derived>& J = const_cast<Eigen::MatrixBase<Derived>&>(JConstRef);
    J.derived().resize(6,6);
    // compute the error at the linearization point
    const Isometry3d A = Z.inverse()*X;
    const Isometry3d& B = P;
    Eigen::Isometry3d::ConstLinearPart Ra = extractRotation(A);
    Eigen::Isometry3d::ConstLinearPart Rb = extractRotation(B);
    Eigen::Isometry3d::ConstTranslationPart tb = B.translation();
    E = A*B;
    Eigen::Isometry3d::ConstLinearPart Re = extractRotation(E);

    Matrix<double, 3 , 9 >  dq_dR;
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
      Matrix3d S;
      skew(S,tb);
      J.template block<3,3>(0,3)=Ra*S;
    }

    // dre/dt =0

    // dre/dq
    {
      double buf[27];
      Map<Matrix<double, 9,3> > M(buf);
      Matrix3d Sx,Sy,Sz;
      internal::skew(Sx,Sy,Sz,Rb);
#ifdef __clang__
      Matrix3d temp = Ra * Sx;
      Map<Matrix3d> M2(temp.data());
      Map<Matrix3d> Mx(buf);    Mx = M2;
      temp = Ra*Sy;
      Map<Matrix3d> My(buf+9);  My = M2;
      temp = Ra*Sz;
      Map<Matrix3d> Mz(buf+18); Mz = M2;
#else
      Map<Matrix3d> Mx(buf);    Mx = Ra*Sx;
      Map<Matrix3d> My(buf+9);  My = Ra*Sy;
      Map<Matrix3d> Mz(buf+18); Mz = Ra*Sz;
#endif
      J.template block<3,3>(3,3) = dq_dR * M;
    }

  }

} // end namespace internal
} // end namespace g2o
#endif
