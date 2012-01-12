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

#include "vertex_point_xy.h"

namespace g2o {
  namespace tutorial {

    VertexPointXY::VertexPointXY() :
      BaseVertex<2, Vector2d>()
    {
      _estimate.setZero();
    }

    bool VertexPointXY::read(std::istream& is)
    {
      is >> _estimate[0] >> _estimate[1];
      return true;
    }

    bool VertexPointXY::write(std::ostream& os) const
    {
      os << estimate()(0) << " " << estimate()(1);
      return os.good();
    }

  } // end namespace
} // end namespace
