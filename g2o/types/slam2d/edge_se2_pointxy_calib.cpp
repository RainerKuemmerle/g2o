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

#include "edge_se2_pointxy_calib.h"

namespace g2o {

  EdgeSE2PointXYCalib::EdgeSE2PointXYCalib() :
    BaseMultiEdge<2, Vector2d>()
  {
    resize(3);
  }

  void EdgeSE2PointXYCalib::initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* /*to*/)
  {
    assert(from.size() == 1 && from.count(_vertices[0]) == 1 && "Can not initialize VertexSE2 position by VertexPointXY");

    if (from.count(_vertices[0]) != 1)
      return;
    VertexSE2* vi     = static_cast<VertexSE2*>(_vertices[0]);
    VertexPointXY* vj = static_cast<VertexPointXY*>(_vertices[1]);
    vj->setEstimate(vi->estimate() * _measurement);
  }

  bool EdgeSE2PointXYCalib::read(std::istream& is)
  {
    is >> _measurement[0] >> _measurement[1];
    is >> information()(0,0) >> information()(0,1) >> information()(1,1);
    information()(1,0) = information()(0,1);
    return true;
  }

  bool EdgeSE2PointXYCalib::write(std::ostream& os) const
  {
    os << measurement()[0] << " " << measurement()[1] << " ";
    os << information()(0,0) << " " << information()(0,1) << " " << information()(1,1);
    return os.good();
  }

} // end namespace
