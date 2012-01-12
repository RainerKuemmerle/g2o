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

#include "edge_se2.h"

namespace g2o {
  namespace tutorial {

    EdgeSE2::EdgeSE2() :
      BaseBinaryEdge<3, SE2, VertexSE2, VertexSE2>()
    {
    }

    bool EdgeSE2::read(std::istream& is)
    {
      Vector3d p;
      is >> p[0] >> p[1] >> p[2];
      _measurement.fromVector(p);
      _inverseMeasurement = measurement().inverse();
      for (int i = 0; i < 3; ++i)
        for (int j = i; j < 3; ++j) {
          is >> information()(i, j);
          if (i != j)
            information()(j, i) = information()(i, j);
        }
      return true;
    }

    bool EdgeSE2::write(std::ostream& os) const
    {
      Vector3d p = measurement().toVector();
      os << p.x() << " " << p.y() << " " << p.z();
      for (int i = 0; i < 3; ++i)
        for (int j = i; j < 3; ++j)
          os << " " << information()(i, j);
      return os.good();
    }

  } // end namespace
} // end namespace
