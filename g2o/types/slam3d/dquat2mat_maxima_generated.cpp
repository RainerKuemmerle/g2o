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

void  compute_dq_dR_w ( Eigen::Matrix<number_t, 3 , 9 >&  dq_dR_w , const number_t&  qw , const number_t&  r00 , const number_t&  r10 , const number_t&  r20 , const number_t&  r01 , const number_t&  r11 , const number_t&  r21 , const number_t&  r02 , const number_t&  r12 , const number_t&  r22 ) { 
  (void) r00;
  (void) r11;
  (void) r22;
  number_t  _aux1 = 1/pow(qw,3) ; 
  number_t  _aux2 = -0.03125*(r21-r12)*_aux1 ; 
  number_t  _aux3 = 1/qw ; 
  number_t  _aux4 = 0.25*_aux3 ; 
  number_t  _aux5 = -0.25*_aux3 ; 
  number_t  _aux6 = 0.03125*(r20-r02)*_aux1 ; 
  number_t  _aux7 = -0.03125*(r10-r01)*_aux1 ; 
   dq_dR_w ( 0 , 0 ) = _aux2 ; 
   dq_dR_w ( 0 , 1 ) = 0 ; 
   dq_dR_w ( 0 , 2 ) = 0 ; 
   dq_dR_w ( 0 , 3 ) = 0 ; 
   dq_dR_w ( 0 , 4 ) = _aux2 ; 
   dq_dR_w ( 0 , 5 ) = _aux4 ; 
   dq_dR_w ( 0 , 6 ) = 0 ; 
   dq_dR_w ( 0 , 7 ) = _aux5 ; 
   dq_dR_w ( 0 , 8 ) = _aux2 ; 
   dq_dR_w ( 1 , 0 ) = _aux6 ; 
   dq_dR_w ( 1 , 1 ) = 0 ; 
   dq_dR_w ( 1 , 2 ) = _aux5 ; 
   dq_dR_w ( 1 , 3 ) = 0 ; 
   dq_dR_w ( 1 , 4 ) = _aux6 ; 
   dq_dR_w ( 1 , 5 ) = 0 ; 
   dq_dR_w ( 1 , 6 ) = _aux4 ; 
   dq_dR_w ( 1 , 7 ) = 0 ; 
   dq_dR_w ( 1 , 8 ) = _aux6 ; 
   dq_dR_w ( 2 , 0 ) = _aux7 ; 
   dq_dR_w ( 2 , 1 ) = _aux4 ; 
   dq_dR_w ( 2 , 2 ) = 0 ; 
   dq_dR_w ( 2 , 3 ) = _aux5 ; 
   dq_dR_w ( 2 , 4 ) = _aux7 ; 
   dq_dR_w ( 2 , 5 ) = 0 ; 
   dq_dR_w ( 2 , 6 ) = 0 ; 
   dq_dR_w ( 2 , 7 ) = 0 ; 
   dq_dR_w ( 2 , 8 ) = _aux7 ; 
} 
void  compute_dq_dR_x ( Eigen::Matrix<number_t, 3 , 9 >&  dq_dR_x , const number_t&  qx , const number_t&  r00 , const number_t&  r10 , const number_t&  r20 , const number_t&  r01 , const number_t&  r11 , const number_t&  r21 , const number_t&  r02 , const number_t&  r12 , const number_t&  r22 ) { 
  (void) r00;
  (void) r11;
  (void) r21;
  (void) r12;
  (void) r22;
  number_t  _aux1 = 1/qx ; 
  number_t  _aux2 = -0.125*_aux1 ; 
  number_t  _aux3 = 1/pow(qx,3) ; 
  number_t  _aux4 = r10+r01 ; 
  number_t  _aux5 = 0.25*_aux1 ; 
  number_t  _aux6 = 0.03125*_aux3*_aux4 ; 
  number_t  _aux7 = r20+r02 ; 
  number_t  _aux8 = 0.03125*_aux3*_aux7 ; 
   dq_dR_x ( 0 , 0 ) = 0.125*_aux1 ; 
   dq_dR_x ( 0 , 1 ) = 0 ; 
   dq_dR_x ( 0 , 2 ) = 0 ; 
   dq_dR_x ( 0 , 3 ) = 0 ; 
   dq_dR_x ( 0 , 4 ) = _aux2 ; 
   dq_dR_x ( 0 , 5 ) = 0 ; 
   dq_dR_x ( 0 , 6 ) = 0 ; 
   dq_dR_x ( 0 , 7 ) = 0 ; 
   dq_dR_x ( 0 , 8 ) = _aux2 ; 
   dq_dR_x ( 1 , 0 ) = -0.03125*_aux3*_aux4 ; 
   dq_dR_x ( 1 , 1 ) = _aux5 ; 
   dq_dR_x ( 1 , 2 ) = 0 ; 
   dq_dR_x ( 1 , 3 ) = _aux5 ; 
   dq_dR_x ( 1 , 4 ) = _aux6 ; 
   dq_dR_x ( 1 , 5 ) = 0 ; 
   dq_dR_x ( 1 , 6 ) = 0 ; 
   dq_dR_x ( 1 , 7 ) = 0 ; 
   dq_dR_x ( 1 , 8 ) = _aux6 ; 
   dq_dR_x ( 2 , 0 ) = -0.03125*_aux3*_aux7 ; 
   dq_dR_x ( 2 , 1 ) = 0 ; 
   dq_dR_x ( 2 , 2 ) = _aux5 ; 
   dq_dR_x ( 2 , 3 ) = 0 ; 
   dq_dR_x ( 2 , 4 ) = _aux8 ; 
   dq_dR_x ( 2 , 5 ) = 0 ; 
   dq_dR_x ( 2 , 6 ) = _aux5 ; 
   dq_dR_x ( 2 , 7 ) = 0 ; 
   dq_dR_x ( 2 , 8 ) = _aux8 ; 
} 
void  compute_dq_dR_y ( Eigen::Matrix<number_t, 3 , 9 >&  dq_dR_y , const number_t&  qy , const number_t&  r00 , const number_t&  r10 , const number_t&  r20 , const number_t&  r01 , const number_t&  r11 , const number_t&  r21 , const number_t&  r02 , const number_t&  r12 , const number_t&  r22 ) { 
  (void) r00;
  (void) r20;
  (void) r11;
  (void) r02;
  (void) r22;
  number_t  _aux1 = 1/pow(qy,3) ; 
  number_t  _aux2 = r10+r01 ; 
  number_t  _aux3 = 0.03125*_aux1*_aux2 ; 
  number_t  _aux4 = 1/qy ; 
  number_t  _aux5 = 0.25*_aux4 ; 
  number_t  _aux6 = -0.125*_aux4 ; 
  number_t  _aux7 = r21+r12 ; 
  number_t  _aux8 = 0.03125*_aux1*_aux7 ; 
   dq_dR_y ( 0 , 0 ) = _aux3 ; 
   dq_dR_y ( 0 , 1 ) = _aux5 ; 
   dq_dR_y ( 0 , 2 ) = 0 ; 
   dq_dR_y ( 0 , 3 ) = _aux5 ; 
   dq_dR_y ( 0 , 4 ) = -0.03125*_aux1*_aux2 ; 
   dq_dR_y ( 0 , 5 ) = 0 ; 
   dq_dR_y ( 0 , 6 ) = 0 ; 
   dq_dR_y ( 0 , 7 ) = 0 ; 
   dq_dR_y ( 0 , 8 ) = _aux3 ; 
   dq_dR_y ( 1 , 0 ) = _aux6 ; 
   dq_dR_y ( 1 , 1 ) = 0 ; 
   dq_dR_y ( 1 , 2 ) = 0 ; 
   dq_dR_y ( 1 , 3 ) = 0 ; 
   dq_dR_y ( 1 , 4 ) = 0.125*_aux4 ; 
   dq_dR_y ( 1 , 5 ) = 0 ; 
   dq_dR_y ( 1 , 6 ) = 0 ; 
   dq_dR_y ( 1 , 7 ) = 0 ; 
   dq_dR_y ( 1 , 8 ) = _aux6 ; 
   dq_dR_y ( 2 , 0 ) = _aux8 ; 
   dq_dR_y ( 2 , 1 ) = 0 ; 
   dq_dR_y ( 2 , 2 ) = 0 ; 
   dq_dR_y ( 2 , 3 ) = 0 ; 
   dq_dR_y ( 2 , 4 ) = -0.03125*_aux1*_aux7 ; 
   dq_dR_y ( 2 , 5 ) = _aux5 ; 
   dq_dR_y ( 2 , 6 ) = 0 ; 
   dq_dR_y ( 2 , 7 ) = _aux5 ; 
   dq_dR_y ( 2 , 8 ) = _aux8 ; 
} 
void  compute_dq_dR_z ( Eigen::Matrix<number_t, 3 , 9 >&  dq_dR_z , const number_t&  qz , const number_t&  r00 , const number_t&  r10 , const number_t&  r20 , const number_t&  r01 , const number_t&  r11 , const number_t&  r21 , const number_t&  r02 , const number_t&  r12 , const number_t&  r22 ) { 
  (void) r00;
  (void) r10;
  (void) r01;
  (void) r11;
  (void) r22;
  number_t  _aux1 = 1/pow(qz,3) ; 
  number_t  _aux2 = r20+r02 ; 
  number_t  _aux3 = 0.03125*_aux1*_aux2 ; 
  number_t  _aux4 = 1/qz ; 
  number_t  _aux5 = 0.25*_aux4 ; 
  number_t  _aux6 = r21+r12 ; 
  number_t  _aux7 = 0.03125*_aux1*_aux6 ; 
  number_t  _aux8 = -0.125*_aux4 ; 
   dq_dR_z ( 0 , 0 ) = _aux3 ; 
   dq_dR_z ( 0 , 1 ) = 0 ; 
   dq_dR_z ( 0 , 2 ) = _aux5 ; 
   dq_dR_z ( 0 , 3 ) = 0 ; 
   dq_dR_z ( 0 , 4 ) = _aux3 ; 
   dq_dR_z ( 0 , 5 ) = 0 ; 
   dq_dR_z ( 0 , 6 ) = _aux5 ; 
   dq_dR_z ( 0 , 7 ) = 0 ; 
   dq_dR_z ( 0 , 8 ) = -0.03125*_aux1*_aux2 ; 
   dq_dR_z ( 1 , 0 ) = _aux7 ; 
   dq_dR_z ( 1 , 1 ) = 0 ; 
   dq_dR_z ( 1 , 2 ) = 0 ; 
   dq_dR_z ( 1 , 3 ) = 0 ; 
   dq_dR_z ( 1 , 4 ) = _aux7 ; 
   dq_dR_z ( 1 , 5 ) = _aux5 ; 
   dq_dR_z ( 1 , 6 ) = 0 ; 
   dq_dR_z ( 1 , 7 ) = _aux5 ; 
   dq_dR_z ( 1 , 8 ) = -0.03125*_aux1*_aux6 ; 
   dq_dR_z ( 2 , 0 ) = _aux8 ; 
   dq_dR_z ( 2 , 1 ) = 0 ; 
   dq_dR_z ( 2 , 2 ) = 0 ; 
   dq_dR_z ( 2 , 3 ) = 0 ; 
   dq_dR_z ( 2 , 4 ) = _aux8 ; 
   dq_dR_z ( 2 , 5 ) = 0 ; 
   dq_dR_z ( 2 , 6 ) = 0 ; 
   dq_dR_z ( 2 , 7 ) = 0 ; 
   dq_dR_z ( 2 , 8 ) = 0.125*_aux4 ; 
} 
void  compute_dR_dq ( Eigen::Matrix<number_t, 9 , 3 >&  dR_dq , const number_t&  qx , const number_t&  qy , const number_t&  qz , const number_t&  qw ) { 
  number_t  _aux1 = -4*qy ; 
  number_t  _aux2 = -4*qz ; 
  number_t  _aux3 = 1/qw ; 
  number_t  _aux4 = 2*qx*qz ; 
  number_t  _aux5 = -_aux3*(_aux4-2*qw*qy) ; 
  number_t  _aux6 = 2*qy*qz ; 
  number_t  _aux7 = -_aux3*(_aux6-2*qw*qx) ; 
  number_t  _aux8 = -2*pow(qw,2) ; 
  number_t  _aux9 = _aux8+2*pow(qz,2) ; 
  number_t  _aux10 = 2*qw*qz ; 
  number_t  _aux11 = (_aux10+2*qx*qy)*_aux3 ; 
  number_t  _aux12 = _aux8+2*pow(qy,2) ; 
  number_t  _aux13 = _aux3*(_aux6+2*qw*qx) ; 
  number_t  _aux14 = _aux3*(_aux4+2*qw*qy) ; 
  number_t  _aux15 = -4*qx ; 
  number_t  _aux16 = _aux8+2*pow(qx,2) ; 
  number_t  _aux17 = (_aux10-2*qx*qy)*_aux3 ; 
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
