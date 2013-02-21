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

#ifndef G2O_SE2_H_
#define G2O_SE2_H_

#include "g2o/stuff/misc.h"
#include "g2o_types_slam2d_api.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace g2o {
  using namespace Eigen;

  /**
   * \brief represent SE2
   */
  class G2O_TYPES_SLAM2D_API SE2 {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      SE2():_R(0),_t(0,0){}

      SE2(const Vector3d& v):_R(v[2]),_t(v[0],v[1]){}

      SE2(double x, double y, double theta):_R(theta),_t(x,y){}

      //! translational component
      inline const Vector2d& translation() const {return _t;}
      void setTranslation(const Vector2d& t_) {_t=t_;}

      //! rotational component
      inline const Rotation2Dd& rotation() const {return _R;}
      void setRotation(const Rotation2Dd& R_) {_R=R_;}

      //! concatenate two SE2 elements (motion composition)
      inline SE2 operator * (const SE2& tr2) const{
        SE2 result(*this);
        result *= tr2;
        return result;
      }

      //! motion composition operator
      inline SE2& operator *= (const SE2& tr2){
        _t+=_R*tr2._t;
        _R.angle()+=tr2._R.angle();
        _R.angle()=normalize_theta(_R.angle());
        return *this;
      }

      //! project a 2D vector
      inline Vector2d operator * (const Vector2d& v) const {
        return _t+_R*v;
      }

      //! invert :-)
      inline SE2 inverse() const{
        SE2 ret;
        ret._R=_R.inverse();
        ret._R.angle()=normalize_theta(ret._R.angle());
#ifdef _MSC_VER
        ret._t=ret._R*(Vector2d(_t*-1.));
#else
        ret._t=ret._R*(_t*-1.);
#endif
        return ret;
      }

      inline double operator [](int i) const {
        assert (i>=0 && i<3);
        if (i<2)
          return _t(i);
        return _R.angle();
      }


      //! assign from a 3D vector (x, y, theta)
      inline void fromVector (const Vector3d& v){
        *this=SE2(v[0], v[1], v[2]);
      }

      //! convert to a 3D vector (x, y, theta)
      inline Vector3d toVector() const {
        return Vector3d(_t.x(), _t.y(), _R.angle());
      }

      operator Eigen::Isometry2d() const
      {
        Eigen::Isometry2d result = (Eigen::Isometry2d) rotation();
        result.translation() = translation();
        return result;
      }

    protected:
      Rotation2Dd _R;
      Vector2d _t;
  };

} // end namespace

#endif
