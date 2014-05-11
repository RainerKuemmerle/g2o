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

#ifndef G2O_EDGE_SE3_EULER_
#define G2O_EDGE_SE3_EULER_

#include "g2o/types/slam3d/edge_se3.h"

#define EDGE_SE3_QUAT_ANALYTIC_JACOBIAN

#include "g2o/types/slam3d/vertex_se3.h"

namespace Slam3dAddons {
  using namespace g2o;

  using namespace Eigen;


/**
 * \brief 3D edge between two VertexSE3, uses the euler angle parameterization
 * for the read/write functions *only*.
 */
class EdgeSE3Euler : public EdgeSE3
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;
};


} // end namespace

#endif
