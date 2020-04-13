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

#include <list>
#include <iostream>
#include "plane3d.h"

using namespace g2o;
using namespace std;
using namespace Eigen;

ostream& operator << (ostream& os, const Plane3D& p){
  Eigen::Vector4d v=p.toVector();
  os << "coeffs: " << v[0] << " " << v[1] << " " << v[2] << " " << v[3];
  return os;
}

typedef std::list<Plane3D*> PlaneList;

void transform(PlaneList& l, const SE3Quat& t){
  for (PlaneList::iterator it=l.begin(); it!=l.end(); ++it){
    Plane3D *p = *it;
    *p = t*(*p);
  }
}

ostream& operator << (ostream& os, const PlaneList& l){
  for (PlaneList::const_iterator it=l.begin(); it!=l.end(); ++it){
    const Plane3D *p = *it;
    os << *p << endl;
  }
  return os;
}


int main (/*int argc, char** argv*/){
  Plane3D p;
  cerr << "p0  " << p << endl;
  Eigen::Vector4d v;
  v << 0.5 , 0.2 , 0.1 , 10;
  Plane3D p1;
  p1.fromVector(v);
  cerr << "p1  " << p1 << endl;
  Plane3D p2 = p1;
  cerr << "p2  " << p2 << endl;

  cerr << "azimuth " << Plane3D::azimuth(p1.normal()) << endl;
  cerr << "elevation " << Plane3D::elevation(p1.normal()) << endl;
  Vector3d mv = p2.ominus(p1);
  cerr << "p ominus p " << mv[0] << " " << mv[1] << " " << mv[2] << endl;

  p1.fromVector(Eigen::Vector4d(2,2,100,10));
  cerr << "p1  " << p1 << endl;
  cerr << "azimuth " << Plane3D::azimuth(p1.normal()) << endl;
  cerr << "elevation " << Plane3D::elevation(p1.normal()) << endl;
  p2.fromVector(Eigen::Vector4d(-2,-2,100,100));
  cerr << "p2  " << p2 << endl;
  cerr << "azimuth " << Plane3D::azimuth(p2.normal()) << endl;
  cerr << "elevation " << Plane3D::elevation(p2.normal()) << endl;

  mv = p2.ominus(p1);
  cerr << "p ominus p " << mv[0] << " " << mv[1] << " " << mv[2] << endl;

  Plane3D p3=p1;
  cerr << "p3  " << p3 << endl;
  p3.oplus(mv);
  cerr << "p3.oplus(mv) " << p3 << endl;

  PlaneList l;
  Plane3D* pp = new Plane3D();
  Eigen::Vector4d coeffs;
  coeffs << 1., 0., 0., 1.;
  pp->fromVector(coeffs);
  l.push_back(pp);

  pp = new Plane3D();
  coeffs << 0., 1., 0., 1.;
  pp->fromVector(coeffs);
  l.push_back(pp);

  pp = new Plane3D();
  coeffs << 0., 0., 1., 1.;
  pp->fromVector(coeffs);
  l.push_back(pp);

  cerr << l << endl;

  AngleAxisd r(AngleAxisd(0.0, Vector3d::UnitZ())
      * AngleAxisd(0., Vector3d::UnitY())
      * AngleAxisd(0., Vector3d::UnitX()));

  SE3Quat t(r.toRotationMatrix(), Vector3d(0.9,0,0));
  cerr << t << endl;
  transform(l,t);
  cerr << l << endl;

  return 0;
}
