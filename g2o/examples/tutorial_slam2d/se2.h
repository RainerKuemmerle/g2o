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

#ifndef G2O_TUTORIAL_SE2_H
#define G2O_TUTORIAL_SE2_H

#include "g2o/stuff/misc.h"
#include "g2o/stuff/macros.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace g2o {
  using namespace Eigen;

  namespace tutorial {

    class SE2 {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        SE2():_R(0),_t(0,0){}

        SE2(double x, double y, double theta):_R(theta),_t(x,y){}

        const Vector2d& translation() const {return _t;}

        Vector2d& translation() {return _t;}

        const Rotation2Dd& rotation() const {return _R;}

        Rotation2Dd& rotation() {return _R;}

        SE2 operator * (const SE2& tr2) const{
          SE2 result(*this);
          result._t += _R*tr2._t;
          result._R.angle()+= tr2._R.angle();
          result._R.angle()=normalize_theta(result._R.angle());
          return result;
        }

        SE2& operator *= (const SE2& tr2){
          _t+=_R*tr2._t;
          _R.angle()+=tr2._R.angle();
          _R.angle()=normalize_theta(_R.angle());
          return *this;
        }

        Vector2d operator * (const Vector2d& v) const {
          return _t+_R*v;
        }

        SE2 inverse() const{
          SE2 ret;
          ret._R=_R.inverse();
          ret._R.angle()=normalize_theta(ret._R.angle());
          ret._t=ret._R*(Vector2d(-1 * _t));
          return ret;
        }

        double operator [](int i) const {
          assert (i>=0 && i<3);
          if (i<2)
            return _t(i);
          return _R.angle();
        }

        double& operator [](int i) {
          assert (i>=0 && i<3);
          if (i<2)
            return _t(i);
          return _R.angle();
        }

        void fromVector (const Vector3d& v){
          *this=SE2(v[0], v[1], v[2]);
        }

        Vector3d toVector() const {
          Vector3d ret;
          for (int i=0; i<3; i++){
            ret(i)=(*this)[i];
          }
          return ret;
        }

      protected:
        Rotation2Dd _R;
        Vector2d _t;
    };

  } // end namespace
} // end namespace

#endif
