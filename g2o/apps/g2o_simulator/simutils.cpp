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

#include "simutils.h"

namespace g2o {
  // -1: outside
  // 0: p1Clipped
  // 1: p2clipped
  // 2: inside
  // 3: all clipped
  using namespace Eigen;

  int clipSegmentCircle(Eigen::Vector2d& p1, Eigen::Vector2d& p2, double r) {
    double r2=r*r;
    Eigen::Vector2d pBase=p1;
    Eigen::Vector2d dp=p2-p1;
    double length=dp.norm();
    dp.normalize();
    double p=2*dp.dot(p1);
    double q=p1.squaredNorm()-r2;
    double disc = p*p - 4*q;
    
    if (disc<=0) { // no intersection or single point intersection
      return -1; 
    }
    disc = sqrt(disc);

    double t1=.5*(-p-disc);
    double t2=.5*(-p+disc);

    if ( t1 > length || t2 <0 ) 
      return -1; // no intersection
    bool clip1=false;
    bool clip2=false;
    if ( t1 > 0 ) {
      p1 = pBase + dp*t1;
      clip1 = true;
    }
    if ( t2 < length ) {
      p2 = pBase + dp*t1;
      clip2 = true;
    }
    if (clip1)
      if (clip2) return 3;
      else return 0;
    else
      if (clip2) return 1;
    return 2;
  }

  // -1: outside
  // 0: p1Clipped
  // 1: p2clipped
  // 2: inside

  int clipSegmentLine(Eigen::Vector2d& p1, Eigen::Vector2d& p2, double a, double b, double c ){
    bool p1inside = true;
    bool p2inside = true;
    if (a*p1.x()+b*p1.y()+c < 0){
      p1inside=false;
    }
    if (a*p2.x()+b*p2.y()+c < 0){
      p2inside=false;
    }
    if (p1inside && p2inside)
      return 2;
    if (!p1inside && !p2inside)
      return -1;

    Eigen::Vector2d dp=p2-p1;
    double den=a*dp.x()+b*dp.y();
    if (den==0)
      return -1;
    double num = c + a*p1.x()+b*p1.y();
    double  t = - num/den;
    if (p1inside){
      p2=p1+dp*t;
      return 1;
    }
    p1=p1+dp*t;
    return 0;
  }

  int clipSegmentFov(Eigen::Vector2d& p1, Eigen::Vector2d& p2, double min, double max){
    bool clip1 = false, clip2 = false;
    // normal to the first line
    double amin =  sin(min), bmin = -cos(min);
    int minClip=clipSegmentLine(p1,p2,amin,bmin,0);
    switch(minClip){
    case -1: 
      return -1; 
    case  0: 
      clip1 = true; 
      break;
    case  1: 
      clip2 = true; 
      break;
    default:;
    }
    // normal to the second line
    double amax = -sin(max), bmax =  cos(max);
    int maxClip=clipSegmentLine(p1,p2,amax,bmax,0);
    switch(maxClip){
    case -1: 
      return -1; 
    case  0: 
      clip1 = true; 
      break;
    case  1: 
      clip2 = true; 
      break;
    default:;
    }
    if (clip1)
      if (clip2) return 3;
      else return 0;
    else
      if (clip2) return 1;
    return 2;
  }

  Eigen::Vector2d computeLineParameters(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2){
    Eigen::Vector2d lp;
    Eigen::Vector2d dp=p2-p1;
    lp[0]=atan2(-dp.x(), dp.y());
    Eigen::Vector2d n(cos(lp[0]), sin(lp[0]));
    lp[1]=n.dot(p1+p2)*.5;
    return lp;
  } 
} // end namespace
