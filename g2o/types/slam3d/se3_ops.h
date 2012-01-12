// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
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

#ifndef G2O_MATH_STUFF
#define G2O_MATH_STUFF

#include <Eigen/Core>
#include <Eigen/Geometry>


namespace g2o {
  using namespace Eigen;

  typedef Matrix<double,4,1> Vector4d;

  inline Matrix3d skew(const Vector3d&v);
  inline Vector3d deltaR(const Matrix3d& R);
  inline Vector2d project(const Vector3d&);
  inline Vector3d project(const Vector4d&);
  inline Vector3d unproject(const Vector2d&);
  inline Vector4d unproject(const Vector3d&);

#include "se3_ops.hpp"

}

#endif //MATH_STUFF
