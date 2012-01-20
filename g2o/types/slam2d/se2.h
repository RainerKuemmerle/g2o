// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

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

    protected:
      Rotation2Dd _R;
      Vector2d _t;
  };

} // end namespace

#endif
