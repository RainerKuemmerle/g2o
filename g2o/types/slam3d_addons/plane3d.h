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

#ifndef G2O_PLANE3D_H_
#define G2O_PLANE3D_H_

#include "g2o_types_slam3d_addons_api.h"
#include "g2o/stuff/misc.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace g2o {
  
  class G2O_TYPES_SLAM3D_ADDONS_API Plane3D {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      friend Plane3D operator*(const Eigen::Isometry3d& t, const Plane3D& plane);

      Plane3D(){
        Eigen::Vector4d v;
        v << 1., 0., 0., -1.;
        fromVector(v);
      }

      Plane3D(const Eigen::Vector4d& v){
        fromVector(v);
      }


      inline Eigen::Vector4d toVector() const {
        return _coeffs;
      }

      inline const Eigen::Vector4d& coeffs() const {return _coeffs;}

      inline void fromVector(const Eigen::Vector4d& coeffs_) {
        _coeffs=coeffs_;
        normalize(_coeffs);
      }

      static double azimuth(const Eigen::Vector3d& v) {
        return atan2(v(1),v(0));
      }

      static  double elevation(const Eigen::Vector3d& v) {
        return atan2(v(2), v.head<2>().norm());
      }

    double distance() const {
      return -_coeffs(3);
    }

    Eigen::Vector3d normal() const {
      return _coeffs.head<3>();
    }

    
    static Eigen::Matrix3d rotation(const Eigen::Vector3d& v)  {
      double _azimuth = azimuth(v);
      double _elevation = elevation(v); 
      return (Eigen::AngleAxisd(_azimuth,  Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(- _elevation, Eigen::Vector3d::UnitY())).toRotationMatrix();
    }

    inline void oplus(const Eigen::Vector3d& v){
      //construct a normal from azimuth and evelation;
      double _azimuth=v[0];
      double _elevation=v[1];
      double s=sin(_elevation), c=cos(_elevation);
      Eigen::Vector3d n (c*cos(_azimuth), c*sin(_azimuth), s) ;
      
      // rotate the normal
      Eigen::Matrix3d R=rotation(normal());
      double d=distance()+v[2];
      _coeffs.head<3>() = R*n;
      _coeffs(3) = -d;
      normalize(_coeffs);
    }
    
    inline Eigen::Vector3d ominus(const Plane3D& plane){
      //construct the rotation that would bring the plane normal in (1 0 0)
      Eigen::Matrix3d R=rotation(normal()).transpose();
      Eigen::Vector3d n=R*plane.normal();
      double d=distance()-plane.distance();
      return Eigen::Vector3d(azimuth(n), elevation(n), d);
    }

    //protected:

    static inline void normalize(Eigen::Vector4d& coeffs) {
      double n=coeffs.head<3>().norm();
      coeffs = coeffs * (1./n);
    }

    Eigen::Vector4d _coeffs;
  };

  inline Plane3D operator*(const Eigen::Isometry3d& t, const Plane3D& plane){
    Eigen::Vector4d v=plane._coeffs;
    Eigen::Vector4d v2;
    Eigen::Matrix3d R=t.rotation();
    v2.head<3>() = R*v.head<3>();
    v2(3)=v(3) - t.translation().dot(v2.head<3>());
    return Plane3D(v2);
  };


}

#endif
