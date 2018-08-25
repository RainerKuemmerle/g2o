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

      friend Plane3D operator*(const Isometry3& t, const Plane3D& plane);

      Plane3D(){
        Vector4 v;
        v << 1., 0., 0., -1.;
        fromVector(v);
      }

      Plane3D(const Vector4& v){
        fromVector(v);
      }


      inline Vector4 toVector() const {
        return _coeffs;
      }

      inline const Vector4& coeffs() const {return _coeffs;}

      inline void fromVector(const Vector4& coeffs_) {
        _coeffs=coeffs_;
        normalize(_coeffs);
      }

      static number_t azimuth(const Vector3& v) {
        return std::atan2(v(1),v(0));
      }

      static  number_t elevation(const Vector3& v) {
        return std::atan2(v(2), v.head<2>().norm());
      }

    number_t distance() const {
      return -_coeffs(3);
    }

    Vector3 normal() const {
      return _coeffs.head<3>();
    }

    
    static Matrix3 rotation(const Vector3& v)  {
      number_t _azimuth = azimuth(v);
      number_t _elevation = elevation(v); 
      return (AngleAxis(_azimuth,  Vector3::UnitZ())* AngleAxis(- _elevation, Vector3::UnitY())).toRotationMatrix();
    }

    inline void oplus(const Vector3& v){
      //construct a normal from azimuth and evelation;
      number_t _azimuth=v[0];
      number_t _elevation=v[1];
      number_t s=std::sin(_elevation), c=std::cos(_elevation);
      Vector3 n (c*std::cos(_azimuth), c*std::sin(_azimuth), s) ;
      
      // rotate the normal
      Matrix3 R=rotation(normal());
      number_t d=distance()+v[2];
      _coeffs.head<3>() = R*n;
      _coeffs(3) = -d;
      normalize(_coeffs);
    }
    
    inline Vector3 ominus(const Plane3D& plane){
      //construct the rotation that would bring the plane normal in (1 0 0)
      Matrix3 R=rotation(normal()).transpose();
      Vector3 n=R*plane.normal();
      number_t d=distance()-plane.distance();
      return Vector3(azimuth(n), elevation(n), d);
    }

    //protected:

    static inline void normalize(Vector4& coeffs) {
      number_t n=coeffs.head<3>().norm();
      coeffs = coeffs * (1./n);
    }

    Vector4 _coeffs;
  };

  inline Plane3D operator*(const Isometry3& t, const Plane3D& plane){
    Vector4 v=plane._coeffs;
    Vector4 v2;
    Matrix3 R=t.rotation();
    v2.head<3>() = R*v.head<3>();
    v2(3)=v(3) - t.translation().dot(v2.head<3>());
    return Plane3D(v2);
  };


}

#endif
