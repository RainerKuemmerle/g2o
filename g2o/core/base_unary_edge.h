// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, H. Strasdat, W. Burgard
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

#ifndef G2O_BASE_UNARY_EDGE_H
#define G2O_BASE_UNARY_EDGE_H

#include <iostream>
#include <cassert>
#include <limits>

#include "base_edge.h"
#include "g2o/config.h"

namespace g2o {

  using namespace Eigen;

  template <int D, typename E, typename VertexXi>
  class BaseUnaryEdge : public BaseEdge<D,E>
  {
    public:
      static const int Dimension = BaseEdge<D, E>::Dimension;
      typedef typename BaseEdge<D,E>::Measurement Measurement;
      typedef VertexXi VertexXiType;
      typedef Matrix<double, D, VertexXiType::Dimension> JacobianXiOplusType;
      typedef typename BaseEdge<D,E>::ErrorVector ErrorVector;
      typedef typename BaseEdge<D,E>::InformationType InformationType;

      BaseUnaryEdge() : BaseEdge<D,E>()
      {
        _vertices.resize(1);
      }

      virtual void resize(size_t size);

      /**
       * Linearizes the oplus operator in the vertex, and stores
       * the result in temporary variables _jacobianOplusXi and _jacobianOplusXj
       */
      virtual void linearizeOplus();

      //! returns the result of the linearization in the manifold space for the node xi
      const JacobianXiOplusType& jacobianOplusXi() const { return _jacobianOplusXi;}

      virtual void constructQuadraticForm();

      virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);

      virtual void mapHessianMemory(double*, int, int, bool) {assert(0 && "BaseUnaryEdge does not map memory of the Hessian");}

      using BaseEdge<D,E>::resize;
      using BaseEdge<D,E>::computeError;

    protected:
      using BaseEdge<D,E>::_measurement;
      using BaseEdge<D,E>::_information;
      using BaseEdge<D,E>::_error;
      using BaseEdge<D,E>::_vertices;
      using BaseEdge<D,E>::_dimension;

      JacobianXiOplusType _jacobianOplusXi;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

#include "base_unary_edge.hpp"

} // end namespace g2o

#endif
