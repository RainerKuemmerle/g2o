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

#include "vertex_se3_euler.h"
#include "g2o/core/factory.h"
#include "g2o/types/slam3d/isometry3d_mappings.h"

#include <iostream>

namespace Slam3dAddons {
  using namespace g2o;

  bool VertexSE3Euler::read(std::istream& is)
  {

    Vector6d est;
    for (int i=0; i<6; i++)
      is  >> est[i];
    Isometry3d transf= g2o::internal::fromVectorET(est);
    setEstimate(transf);
    updateCache();
    return true;
  }

  bool VertexSE3Euler::write(std::ostream& os) const
  {
    Vector6d est =  g2o::internal::toVectorET(estimate());
    for (int i=0; i<6; i++)
      os << est[i] << " ";
    return os.good();
  }

}
