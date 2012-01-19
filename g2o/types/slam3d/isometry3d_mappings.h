// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
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

#ifndef G2O_ISOMATRY3D_MAPPINGS_H_
#define G2O_ISOMATRY3D_MAPPINGS_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace g2o {
  using namespace Eigen;
  
  typedef Matrix<double, 6, 1> Vector6d;
  typedef Matrix<double, 7, 1> Vector7d;
  
  Quaterniond G2O_TYPES_SLAM3D_API normalized(const Quaterniond& q);
  Quaterniond& G2O_TYPES_SLAM3D_API normalize(Quaterniond& q);

  // functions to handle the rotation part
  Vector3d G2O_TYPES_SLAM3D_API toEuler(const Eigen::Matrix3d& R);
  Matrix3d G2O_TYPES_SLAM3D_API fromEuler(const Vector3d& v);
  Vector3d G2O_TYPES_SLAM3D_API toCompactQuaternion(const Eigen::Matrix3d& R);
  Matrix3d G2O_TYPES_SLAM3D_API fromCompactQuaternion(const Vector3d& v);

  
  // functions to handle the toVector of the whole transformations
  Vector6d G2O_TYPES_SLAM3D_API toVectorMQT(const Isometry3d& t);
  Vector6d G2O_TYPES_SLAM3D_API toVectorET(const Isometry3d& t);
  Vector7d G2O_TYPES_SLAM3D_API toVectorQT(const Isometry3d& t);
  
  Isometry3d G2O_TYPES_SLAM3D_API fromVectorMQT(const Vector6d& v);
  Isometry3d G2O_TYPES_SLAM3D_API fromVectorET(const Vector6d& v);
  Isometry3d G2O_TYPES_SLAM3D_API fromVectorQT(const Vector7d& v);
  
}

#endif
