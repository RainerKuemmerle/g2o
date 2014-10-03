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

#ifndef G2O_DEPRECATED_SE3QUAT_GRADIENTS_H
#define G2O_DEPRECATED_SE3QUAT_GRADIENTS_H

#include <Eigen/Core>

namespace g2o {
namespace deprecated {

template <typename Derived>
void jacobian_3d_qman ( Eigen::MatrixBase<Derived> &  Ji , Eigen::MatrixBase<Derived> &  Jj,
    const double&  z11 , const double&  z12 , const double&  z13 , const double&  z14 ,
  const double&  z21 , const double&  z22 , const double&  z23 , const double&  z24 ,
  const double&  z31 , const double&  z32 , const double&  z33 , const double&  z34 ,
  const double&  xab11 , const double&  xab12 , const double&  xab13 , const double&  xab14 ,
  const double&  xab21 , const double&  xab22 , const double&  xab23 , const double&  xab24 ,
  const double&  xab31 , const double&  xab32 , const double&  xab33 , const double&  xab34 )
{
  Ji.derived().resize(6,6);
  Jj.derived().resize(6,6);
  (void) z14;
  (void) z24;
  (void) z34;
  double  _aux1 = xab11*z11 ; 
  double  _aux2 = xab21*z12 ; 
  double  _aux3 = xab31*z13 ; 
  double  _aux4 = xab13*z11 ; 
  double  _aux5 = xab23*z12 ; 
  double  _aux6 = xab33*z13 ; 
  double  _aux7 = xab11*z21 ; 
  double  _aux8 = xab21*z22 ; 
  double  _aux9 = xab31*z23 ; 
  double  _aux10 = xab12*z21 ; 
  double  _aux11 = xab22*z22 ; 
  double  _aux12 = xab32*z23 ; 
  double  _aux13 = xab12*z31 ; 
  double  _aux14 = xab22*z32 ; 
  double  _aux15 = xab32*z33 ; 
  double  _aux16 = xab13*z31 ; 
  double  _aux17 = xab23*z32 ; 
  double  _aux18 = xab33*z33 ; 
  double  _aux19 = _aux15+_aux14+_aux13-xab33*z23-xab23*z22-xab13*z21 ; 
  double  _aux20 = -2*xab21*z23+2*xab31*z22+2*xab22*z13-2*xab32*z12 ; 
  double  _aux21 = _aux9+_aux8+_aux7-xab32*z13-xab22*z12-xab12*z11 ; 
  double  _aux22 = 2*xab21*z33-2*xab31*z32-2*xab23*z13+2*xab33*z12 ; 
  double  _aux23 = _aux6+_aux5+_aux4-xab31*z33-xab21*z32-xab11*z31 ; 
  double  _aux24 = -2*xab22*z33+2*xab32*z32+2*xab23*z23-2*xab33*z22 ; 
  double  _aux25 = _aux3+_aux2+_aux18+_aux17+_aux16+_aux12+_aux11+_aux10+_aux1+1 ; 
  double  _aux26 = 2*(-2*xab23*z33+2*xab33*z32-2*xab22*z23+2*xab32*z22-2*xab21*z13+2*xab31*z12)*_aux25+2*_aux19*_aux24+2*_aux22*_aux23+2*_aux20*_aux21 ; 
  double  _aux27 = sqrt(pow(_aux25,2)+pow(_aux23,2)+pow(_aux21,2)+pow(_aux19,2)) ; 
  double  _aux28 = 1/pow(_aux27,3) ; 
  double  _aux29 = 1/_aux27 ; 
  double  _aux30 = 2*xab11*z23-2*xab31*z21-2*xab12*z13+2*xab32*z11 ; 
  double  _aux31 = -2*xab11*z33+2*xab31*z31+2*xab13*z13-2*xab33*z11 ; 
  double  _aux32 = 2*xab12*z33-2*xab32*z31-2*xab13*z23+2*xab33*z21 ; 
  double  _aux33 = 2*_aux19*_aux32+2*_aux23*_aux31+2*_aux21*_aux30+2*(2*xab13*z33-2*xab33*z31+2*xab12*z23-2*xab32*z21+2*xab11*z13-2*xab31*z11)*_aux25 ; 
  double  _aux34 = -2*xab11*z22+2*xab21*z21+2*xab12*z12-2*xab22*z11 ; 
  double  _aux35 = 2*xab11*z32-2*xab21*z31-2*xab13*z12+2*xab23*z11 ; 
  double  _aux36 = -2*xab12*z32+2*xab22*z31+2*xab13*z22-2*xab23*z21 ; 
  double  _aux37 = 2*_aux19*_aux36+2*_aux23*_aux35+2*_aux21*_aux34+2*(-2*xab13*z32+2*xab23*z31-2*xab12*z22+2*xab22*z21-2*xab11*z12+2*xab21*z11)*_aux25 ; 
  double  _aux38 = 2*xab12*z21 ; 
  double  _aux39 = 2*xab22*z22 ; 
  double  _aux40 = 2*xab32*z23 ; 
  double  _aux41 = 2*xab13*z31 ; 
  double  _aux42 = 2*xab23*z32 ; 
  double  _aux43 = 2*xab33*z33 ; 
  double  _aux44 = _aux43+_aux42+_aux41+_aux40+_aux39+_aux38 ; 
  double  _aux45 = -2*xab13*z11 ; 
  double  _aux46 = -2*xab23*z12 ; 
  double  _aux47 = -2*xab33*z13 ; 
  double  _aux48 = _aux47+_aux46+_aux45 ; 
  double  _aux49 = -2*xab32*z13-2*xab22*z12-2*xab12*z11 ; 
  double  _aux50 = -2*xab12*z31 ; 
  double  _aux51 = -2*xab22*z32 ; 
  double  _aux52 = -2*xab32*z33 ; 
  double  _aux53 = 2*_aux25*(_aux52+_aux51+_aux50+2*xab33*z23+2*xab23*z22+2*xab13*z21)+2*_aux23*_aux49+2*_aux21*_aux48+2*_aux19*_aux44 ; 
  double  _aux54 = -2*xab11*z21 ; 
  double  _aux55 = -2*xab21*z22 ; 
  double  _aux56 = -2*xab31*z23 ; 
  double  _aux57 = _aux56+_aux55+_aux54 ; 
  double  _aux58 = -2*xab33*z23-2*xab23*z22-2*xab13*z21 ; 
  double  _aux59 = 2*xab11*z11 ; 
  double  _aux60 = 2*xab21*z12 ; 
  double  _aux61 = 2*xab31*z13 ; 
  double  _aux62 = _aux61+_aux60+_aux59+_aux43+_aux42+_aux41 ; 
  double  _aux63 = 2*_aux23*_aux62+2*_aux21*_aux58+2*_aux19*_aux57+2*_aux25*(_aux47+_aux46+_aux45+2*xab31*z33+2*xab21*z32+2*xab11*z31) ; 
  double  _aux64 = _aux61+_aux60+_aux59+_aux40+_aux39+_aux38 ; 
  double  _aux65 = _aux52+_aux51+_aux50 ; 
  double  _aux66 = -2*xab31*z33-2*xab21*z32-2*xab11*z31 ; 
  double  _aux67 = 2*_aux19*_aux66+2*_aux23*_aux65+2*_aux21*_aux64+2*_aux25*(_aux56+_aux55+_aux54+2*xab32*z13+2*xab22*z12+2*xab12*z11) ; 

  Ji ( 0 , 0 ) = -z11 ; 
  Ji ( 0 , 1 ) = -z12 ; 
  Ji ( 0 , 2 ) = -z13 ; 
  Ji ( 0 , 3 ) = 2*xab34*z12-2*xab24*z13 ; 
  Ji ( 0 , 4 ) = 2*xab14*z13-2*xab34*z11 ; 
  Ji ( 0 , 5 ) = 2*xab24*z11-2*xab14*z12 ; 
  Ji ( 1 , 0 ) = -z21 ; 
  Ji ( 1 , 1 ) = -z22 ; 
  Ji ( 1 , 2 ) = -z23 ; 
  Ji ( 1 , 3 ) = 2*xab34*z22-2*xab24*z23 ; 
  Ji ( 1 , 4 ) = 2*xab14*z23-2*xab34*z21 ; 
  Ji ( 1 , 5 ) = 2*xab24*z21-2*xab14*z22 ; 
  Ji ( 2 , 0 ) = -z31 ; 
  Ji ( 2 , 1 ) = -z32 ; 
  Ji ( 2 , 2 ) = -z33 ; 
  Ji ( 2 , 3 ) = 2*xab34*z32-2*xab24*z33 ; 
  Ji ( 2 , 4 ) = 2*xab14*z33-2*xab34*z31 ; 
  Ji ( 2 , 5 ) = 2*xab24*z31-2*xab14*z32 ; 
  Ji ( 3 , 0 ) = 0 ; 
  Ji ( 3 , 1 ) = 0 ; 
  Ji ( 3 , 2 ) = 0 ; 
  Ji ( 3 , 3 ) = _aux24*_aux29-0.5*_aux19*_aux26*_aux28 ; 
  Ji ( 3 , 4 ) = _aux29*_aux32-0.5*_aux19*_aux28*_aux33 ; 
  Ji ( 3 , 5 ) = _aux29*_aux36-0.5*_aux19*_aux28*_aux37 ; 
  Ji ( 4 , 0 ) = 0 ; 
  Ji ( 4 , 1 ) = 0 ; 
  Ji ( 4 , 2 ) = 0 ; 
  Ji ( 4 , 3 ) = _aux22*_aux29-0.5*_aux23*_aux26*_aux28 ; 
  Ji ( 4 , 4 ) = _aux29*_aux31-0.5*_aux23*_aux28*_aux33 ; 
  Ji ( 4 , 5 ) = _aux29*_aux35-0.5*_aux23*_aux28*_aux37 ; 
  Ji ( 5 , 0 ) = 0 ; 
  Ji ( 5 , 1 ) = 0 ; 
  Ji ( 5 , 2 ) = 0 ; 
  Ji ( 5 , 3 ) = _aux20*_aux29-0.5*_aux21*_aux26*_aux28 ; 
  Ji ( 5 , 4 ) = _aux29*_aux30-0.5*_aux21*_aux28*_aux33 ; 
  Ji ( 5 , 5 ) = _aux29*_aux34-0.5*_aux21*_aux28*_aux37 ; 

  Jj( 0 , 0 ) = _aux3+_aux2+_aux1 ; 
  Jj( 0 , 1 ) = xab32*z13+xab22*z12+xab12*z11 ; 
  Jj( 0 , 2 ) = _aux6+_aux5+_aux4 ; 
  Jj( 0 , 3 ) = 0 ; 
  Jj( 0 , 4 ) = 0 ; 
  Jj( 0 , 5 ) = 0 ; 
  Jj( 1 , 0 ) = _aux9+_aux8+_aux7 ; 
  Jj( 1 , 1 ) = _aux12+_aux11+_aux10 ; 
  Jj( 1 , 2 ) = xab33*z23+xab23*z22+xab13*z21 ; 
  Jj( 1 , 3 ) = 0 ; 
  Jj( 1 , 4 ) = 0 ; 
  Jj( 1 , 5 ) = 0 ; 
  Jj( 2 , 0 ) = xab31*z33+xab21*z32+xab11*z31 ; 
  Jj( 2 , 1 ) = _aux15+_aux14+_aux13 ; 
  Jj( 2 , 2 ) = _aux18+_aux17+_aux16 ; 
  Jj( 2 , 3 ) = 0 ; 
  Jj( 2 , 4 ) = 0 ; 
  Jj( 2 , 5 ) = 0 ; 
  Jj( 3 , 0 ) = 0 ; 
  Jj( 3 , 1 ) = 0 ; 
  Jj( 3 , 2 ) = 0 ; 
  Jj( 3 , 3 ) = _aux29*_aux44-0.5*_aux19*_aux28*_aux53 ; 
  Jj( 3 , 4 ) = _aux29*_aux57-0.5*_aux19*_aux28*_aux63 ; 
  Jj( 3 , 5 ) = _aux29*_aux66-0.5*_aux19*_aux28*_aux67 ; 
  Jj( 4 , 0 ) = 0 ; 
  Jj( 4 , 1 ) = 0 ; 
  Jj( 4 , 2 ) = 0 ; 
  Jj( 4 , 3 ) = _aux29*_aux49-0.5*_aux23*_aux28*_aux53 ; 
  Jj( 4 , 4 ) = _aux29*_aux62-0.5*_aux23*_aux28*_aux63 ; 
  Jj( 4 , 5 ) = _aux29*_aux65-0.5*_aux23*_aux28*_aux67 ; 
  Jj( 5 , 0 ) = 0 ; 
  Jj( 5 , 1 ) = 0 ; 
  Jj( 5 , 2 ) = 0 ; 
  Jj( 5 , 3 ) = _aux29*_aux48-0.5*_aux21*_aux28*_aux53 ; 
  Jj( 5 , 4 ) = _aux29*_aux58-0.5*_aux21*_aux28*_aux63 ; 
  Jj( 5 , 5 ) = _aux29*_aux64-0.5*_aux21*_aux28*_aux67 ; 
}

} // end namespace
} // end namespace

#endif
