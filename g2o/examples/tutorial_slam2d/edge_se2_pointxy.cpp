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

#include "edge_se2_pointxy.h"

namespace g2o {
  namespace tutorial {

    EdgeSE2PointXY::EdgeSE2PointXY() :
      BaseBinaryEdge<2, Vector2d, VertexSE2, VertexPointXY>(),
      _sensorOffset(0), _sensorCache(0)
    {
      resizeParameters(1);
      installParameter(_sensorOffset, 0);
    }

    bool EdgeSE2PointXY::read(std::istream& is)
    {
      int paramId;
      is >> paramId;
      if (! setParameterId(0,paramId))
        return false;
      is >> _measurement[0] >> _measurement[1];
      is >> information()(0,0) >> information()(0,1) >> information()(1,1);
      information()(1,0) = information()(0,1);
      return true;
    }

    bool EdgeSE2PointXY::write(std::ostream& os) const
    {
      os << _sensorOffset->id() << " ";
      os << measurement()[0] << " " << measurement()[1] << " ";
      os << information()(0,0) << " " << information()(0,1) << " " << information()(1,1);
      return os.good();
    }

    void EdgeSE2PointXY::computeError()
    {
      const VertexPointXY* l2 = static_cast<const VertexPointXY*>(_vertices[1]);
      _error = (_sensorCache->w2n() * l2->estimate()) - _measurement;
    }

    bool EdgeSE2PointXY::resolveCaches()
    {
      ParameterVector pv(1);
      pv[0] = _sensorOffset;
      resolveCache(_sensorCache, static_cast<OptimizableGraph::Vertex*>(_vertices[0]), "TUTORIAL_CACHE_SE2_OFFSET", pv);
      return _sensorCache != 0;
    }

  } // end namespace
} // end namespace
