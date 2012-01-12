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

#ifndef G2O_TYPES_SLAM3D_ONLINE_H
#define G2O_TYPES_SLAM3D_ONLINE_H

#include "g2o/types/slam3d/edge_se3_quat.h"

#include <iostream>

namespace g2o {
  
  using namespace Eigen;

  class OnlineVertexSE3 : public VertexSE3
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      OnlineVertexSE3() : VertexSE3() {}

      virtual void oplusImpl(const double* update)
      {
        VertexSE3::oplusImpl(update);
        updatedEstimate = _estimate;
      }

      void oplusUpdatedEstimate(double* update)
      {
        Map<Vector6d> v(update);
        SE3Quat increment(v);
        updatedEstimate = estimate() * increment;
      }

      VertexSE3::EstimateType updatedEstimate;
  };

  class OnlineEdgeSE3 : public EdgeSE3
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      OnlineEdgeSE3() : EdgeSE3() {}

      void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* /* to */)
      {
        OnlineVertexSE3* fromEdge = static_cast<OnlineVertexSE3*>(_vertices[0]);
        OnlineVertexSE3* toEdge   = static_cast<OnlineVertexSE3*>(_vertices[1]);
        if (from.count(fromEdge) > 0) {
          toEdge->updatedEstimate = fromEdge->updatedEstimate * _measurement;
          toEdge->setEstimate(toEdge->updatedEstimate);
        } else {
          fromEdge->updatedEstimate = toEdge->updatedEstimate * _inverseMeasurement;
          fromEdge->setEstimate(fromEdge->updatedEstimate);
        }
      }
  };

} // end namespace

#endif
