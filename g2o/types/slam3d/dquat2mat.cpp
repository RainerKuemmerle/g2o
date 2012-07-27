#include "dquat2mat.h"
#include <iostream>
namespace g2o {
  namespace internal {
    using namespace std;

#include "dquat2mat_maxima_generated.cpp"

    int _q2m(double& S, double& qw, const double&  r00 , const double&  r10 , const double&  r20 , const double&  r01 , const double&  r11 , const double&  r21 , const double&  r02 , const double&  r12 , const double&  r22 ){
      double tr=r00 + r11 + r22;
      if (tr > 0) { 
	S = sqrt(tr + 1.0) * 2; // S=4*qw 
	qw = 0.25 * S;
	// qx = (r21 - r12) / S;
	// qy = (r02 - r20) / S; 
	// qz = (r10 - r01) / S; 
	return 0;
      } else if ((r00 > r11)&(r00 > r22)) { 
	S = sqrt(1.0 + r00 - r11 - r22) * 2; // S=4*qx 
	qw = (r21 - r12) / S;
	// qx = 0.25 * S;
	// qy = (r01 + r10) / S; 
	// qz = (r02 + r20) / S; 
	return 1;
      } else if (r11 > r22) { 
	S = sqrt(1.0 + r11 - r00 - r22) * 2; // S=4*qy
	qw = (r02 - r20) / S;
	// qx = (r01 + r10) / S; 
	// qy = 0.25 * S;
	// qz = (r12 + r21) / S; 
	return 2;
      } else { 
	S = sqrt(1.0 + r22 - r00 - r11) * 2; // S=4*qz
	qw = (r10 - r01) / S;
	// qx = (r02 + r20) / S;
	// qy = (r12 + r21) / S;
	// qz = 0.25 * S;
	return 3;
      }
    }
    
    void  compute_dq_dR ( Eigen::Matrix<double, 3 , 9 >&  dq_dR , const double&  r11 , const double&  r21 , const double&  r31 , const double&  r12 , const double&  r22 , const double&  r32 , const double&  r13 , const double&  r23 , const double&  r33 ){
      double qw;
      double S;
      int whichCase=_q2m( S, qw, r11 ,  r21 ,  r31 ,  r12 ,  r22 ,  r32 ,  r13 ,  r23 ,  r33 );
      S*=.25;
      switch(whichCase){
      case 0: compute_dq_dR_w(dq_dR, S, r11 ,  r21 ,  r31 ,  r12 ,  r22 ,  r32 ,  r13 ,  r23 ,  r33 ); 
	break;
      case 1: compute_dq_dR_x(dq_dR, S, r11 ,  r21 ,  r31 ,  r12 ,  r22 ,  r32 ,  r13 ,  r23 ,  r33 ); 
	break;
      case 2: compute_dq_dR_y(dq_dR, S, r11 ,  r21 ,  r31 ,  r12 ,  r22 ,  r32 ,  r13 ,  r23 ,  r33 ); 
	break;
      case 3: compute_dq_dR_z(dq_dR, S, r11 ,  r21 ,  r31 ,  r12 ,  r22 ,  r32 ,  r13 ,  r23 ,  r33 ); 
	break;
      }
      if (qw<=0)
	dq_dR *= -1;
    }
  }
}
