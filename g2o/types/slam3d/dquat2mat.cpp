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

void  compute_dq_dR ( Eigen::Matrix<double, 3 , 9 >&  dq_dR , const double&  r11 , const double&  r21 , const double&  r31 , const double&  r12 , const double&  r22 , const double&  r32 , const double&  r13 , const double&  r23 , const double&  r33 ) { 
  double  _aux1 = pow(r33+r22+r11+1,0.5) ; 
  double  _aux2 = 1/pow(_aux1,3) ; 
  double  _aux3 = -0.25*(r32-r23)*_aux2 ; 
  double  _aux4 = 1/_aux1 ; 
  double  _aux5 = 0.5*_aux4 ; 
  double  _aux6 = -0.5*_aux4 ; 
  double  _aux7 = -0.25*(r13-r31)*_aux2 ; 
  double  _aux8 = -0.25*(r21-r12)*_aux2 ; 
   dq_dR ( 0 , 0 ) = _aux3 ; 
   dq_dR ( 0 , 1 ) = 0 ; 
   dq_dR ( 0 , 2 ) = 0 ; 
   dq_dR ( 0 , 3 ) = 0 ; 
   dq_dR ( 0 , 4 ) = _aux3 ; 
   dq_dR ( 0 , 5 ) = _aux5 ; 
   dq_dR ( 0 , 6 ) = 0 ; 
   dq_dR ( 0 , 7 ) = _aux6 ; 
   dq_dR ( 0 , 8 ) = _aux3 ; 
   dq_dR ( 1 , 0 ) = _aux7 ; 
   dq_dR ( 1 , 1 ) = 0 ; 
   dq_dR ( 1 , 2 ) = _aux6 ; 
   dq_dR ( 1 , 3 ) = 0 ; 
   dq_dR ( 1 , 4 ) = _aux7 ; 
   dq_dR ( 1 , 5 ) = 0 ; 
   dq_dR ( 1 , 6 ) = _aux5 ; 
   dq_dR ( 1 , 7 ) = 0 ; 
   dq_dR ( 1 , 8 ) = _aux7 ; 
   dq_dR ( 2 , 0 ) = _aux8 ; 
   dq_dR ( 2 , 1 ) = _aux5 ; 
   dq_dR ( 2 , 2 ) = 0 ; 
   dq_dR ( 2 , 3 ) = _aux6 ; 
   dq_dR ( 2 , 4 ) = _aux8 ; 
   dq_dR ( 2 , 5 ) = 0 ; 
   dq_dR ( 2 , 6 ) = 0 ; 
   dq_dR ( 2 , 7 ) = 0 ; 
   dq_dR ( 2 , 8 ) = _aux8 ; 
} 
void  compute_dR_dq ( Eigen::Matrix<double, 9 , 3 >&  dR_dq , const double&  qx , const double&  qy , const double&  qz , const double&  qw ) { 
  double  _aux1 = -4*qy ; 
  double  _aux2 = -4*qz ; 
  double  _aux3 = 1/qw ; 
  double  _aux4 = 2*qx*qz ; 
  double  _aux5 = -_aux3*(_aux4-2*qw*qy) ; 
  double  _aux6 = 2*qy*qz ; 
  double  _aux7 = -_aux3*(_aux6-2*qw*qx) ; 
  double  _aux8 = -2*pow(qw,2) ; 
  double  _aux9 = _aux8+2*pow(qz,2) ; 
  double  _aux10 = 2*qw*qz ; 
  double  _aux11 = (_aux10+2*qx*qy)*_aux3 ; 
  double  _aux12 = _aux8+2*pow(qy,2) ; 
  double  _aux13 = _aux3*(_aux6+2*qw*qx) ; 
  double  _aux14 = _aux3*(_aux4+2*qw*qy) ; 
  double  _aux15 = -4*qx ; 
  double  _aux16 = _aux8+2*pow(qx,2) ; 
  double  _aux17 = (_aux10-2*qx*qy)*_aux3 ; 
   dR_dq ( 0 , 0 ) = 0 ; 
   dR_dq ( 0 , 1 ) = _aux1 ; 
   dR_dq ( 0 , 2 ) = _aux2 ; 
   dR_dq ( 1 , 0 ) = _aux5 ; 
   dR_dq ( 1 , 1 ) = _aux7 ; 
   dR_dq ( 1 , 2 ) = -_aux3*_aux9 ; 
   dR_dq ( 2 , 0 ) = _aux11 ; 
   dR_dq ( 2 , 1 ) = _aux12*_aux3 ; 
   dR_dq ( 2 , 2 ) = _aux13 ; 
   dR_dq ( 3 , 0 ) = _aux14 ; 
   dR_dq ( 3 , 1 ) = _aux13 ; 
   dR_dq ( 3 , 2 ) = _aux3*_aux9 ; 
   dR_dq ( 4 , 0 ) = _aux15 ; 
   dR_dq ( 4 , 1 ) = 0 ; 
   dR_dq ( 4 , 2 ) = _aux2 ; 
   dR_dq ( 5 , 0 ) = -_aux16*_aux3 ; 
   dR_dq ( 5 , 1 ) = _aux17 ; 
   dR_dq ( 5 , 2 ) = _aux5 ; 
   dR_dq ( 6 , 0 ) = _aux17 ; 
   dR_dq ( 6 , 1 ) = -_aux12*_aux3 ; 
   dR_dq ( 6 , 2 ) = _aux7 ; 
   dR_dq ( 7 , 0 ) = _aux16*_aux3 ; 
   dR_dq ( 7 , 1 ) = _aux11 ; 
   dR_dq ( 7 , 2 ) = _aux14 ; 
   dR_dq ( 8 , 0 ) = _aux15 ; 
   dR_dq ( 8 , 1 ) = _aux1 ; 
   dR_dq ( 8 , 2 ) = 0 ; 
} 
