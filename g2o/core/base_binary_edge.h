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

#ifndef G2O_BASE_BINARY_EDGE_H
#define G2O_BASE_BINARY_EDGE_H

#include <iostream>
#include <limits>

#include "base_edge.h"
#include "g2o/config.h"

namespace g2o {

  using namespace Eigen;

  template <int D, typename E, typename VertexXi, typename VertexXj>
  class BaseBinaryEdge : public BaseEdge<D, E>
  {
    public:

      typedef VertexXi VertexXiType;
      typedef VertexXj VertexXjType;

      static const int Di = VertexXiType::Dimension;
      static const int Dj = VertexXjType::Dimension;

      static const int Dimension = BaseEdge<D, E>::Dimension;
      typedef typename BaseEdge<D,E>::Measurement Measurement;
      typedef Matrix<double, D, Di> JacobianXiOplusType;
      typedef Matrix<double, D, Dj> JacobianXjOplusType;
      typedef typename BaseEdge<D,E>::ErrorVector ErrorVector;
      typedef typename BaseEdge<D,E>::InformationType InformationType;

      typedef Map<Matrix<double, Di, Dj>, Matrix<double, Di, Dj>::Flags & AlignedBit ? Aligned : Unaligned > HessianBlockType;
      typedef Map<Matrix<double, Dj, Di>, Matrix<double, Dj, Di>::Flags & AlignedBit ? Aligned : Unaligned > HessianBlockTransposedType;

      BaseBinaryEdge() : BaseEdge<D,E>(),
      _hessianRowMajor(false),
      _hessian(0, VertexXiType::Dimension, VertexXjType::Dimension), // HACK we map to the null pointer for initializing the Maps
      _hessianTransposed(0, VertexXjType::Dimension, VertexXiType::Dimension)
      {
        _vertices.resize(2);
      }

      virtual OptimizableGraph::Vertex* createFrom();
      virtual OptimizableGraph::Vertex* createTo();

      virtual void resize(size_t size);

      /**
       * Linearizes the oplus operator in the vertex, and stores
       * the result in temporary variables _jacobianOplusXi and _jacobianOplusXj
       */
      virtual void linearizeOplus();

      //! returns the result of the linearization in the manifold space for the node xi
      const JacobianXiOplusType& jacobianOplusXi() const { return _jacobianOplusXi;}
      //! returns the result of the linearization in the manifold space for the node xj
      const JacobianXjOplusType& jacobianOplusXj() const { return _jacobianOplusXj;}

      virtual void constructQuadraticForm() ;

      virtual void mapHessianMemory(double* d, int i, int j, bool rowMajor);

      using BaseEdge<D,E>::resize;
      using BaseEdge<D,E>::computeError;

    protected:
      using BaseEdge<D,E>::_measurement;
      using BaseEdge<D,E>::_information;
      using BaseEdge<D,E>::_error;
      using BaseEdge<D,E>::_vertices;
      using BaseEdge<D,E>::_dimension;

      bool _hessianRowMajor;
      HessianBlockType _hessian;
      HessianBlockTransposedType _hessianTransposed;
      JacobianXiOplusType _jacobianOplusXi;
      JacobianXjOplusType _jacobianOplusXj;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

#include "base_binary_edge.hpp"

} // end namespace g2o

#endif
