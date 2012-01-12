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

#include "edge_se2_prior.h"

namespace g2o {

  EdgeSE2Prior::EdgeSE2Prior() : BaseUnaryEdge<3, SE2, VertexSE2>()
  {
  }

  void EdgeSE2Prior::initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to)
  {
    assert(from.size() == 0); (void) from; (void) to;
    VertexSE2* v1 = static_cast<VertexSE2*>(_vertices[0]);
    v1->setEstimate(_measurement);
  }

  bool EdgeSE2Prior::read(std::istream& is)
  {
    Vector3d p;
    is >> p[0] >> p[1] >> p[2];
    setMeasurement(p);
    _inverseMeasurement= _measurement.inverse();
    for (int i = 0; i < 3; ++i)
      for (int j = i; j < 3; ++j) {
        is >> information()(i, j);
        if (i != j)
          information()(j, i) = information()(i, j);
      }
    return true;
  }

  bool EdgeSE2Prior::write(std::ostream& os) const
  {
    Vector3d p = measurement().toVector();
    os << p.x() << " " << p.y() << " " << p.z();
    for (int i = 0; i < 3; ++i)
      for (int j = i; j < 3; ++j)
        os << " " << information()(i, j);
    return os.good();
  }

  void EdgeSE2Prior::setMeasurement(const SE2& m)
  {
    _measurement = m;
    _inverseMeasurement = m.inverse();
  }

  bool EdgeSE2Prior::setMeasurementData(const double* d) {
    _measurement=SE2(d[0], d[1], d[2]);
    _inverseMeasurement = _measurement.inverse();
    return true;
  }

} // end namespace
