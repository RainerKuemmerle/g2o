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

#ifndef G2O_TYPES_SLAM2D_ONLINE_H
#define G2O_TYPES_SLAM2D_ONLINE_H

#include "g2o/types/slam2d/edge_se2.h"

#include <iostream>

namespace g2o {

  class OnlineVertexSE2 : public VertexSE2
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      OnlineVertexSE2() : VertexSE2() {}

      virtual void oplusImpl(const double* update)
      {
        VertexSE2::oplusImpl(update);
        updatedEstimate = _estimate;
      }

      void oplusUpdatedEstimate(double* update)
      {
        Vector3d p=_estimate.toVector();
        p+=Map<Vector3d>(update);
        p[2]=normalize_theta(p[2]);
        updatedEstimate.fromVector(p);
        //std::cerr << PVAR(updatedEstimate.toVector()) << " " << PVAR(_estimate.toVector()) << std::endl;
      }

      VertexSE2::EstimateType updatedEstimate;
  };

  class OnlineEdgeSE2 : public EdgeSE2
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      OnlineEdgeSE2() : EdgeSE2() {}

      void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* /* to */)
      {
        OnlineVertexSE2* fromEdge = static_cast<OnlineVertexSE2*>(_vertices[0]);
        OnlineVertexSE2* toEdge   = static_cast<OnlineVertexSE2*>(_vertices[1]);
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
