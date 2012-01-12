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

#ifndef G2O_BASE_MULTI_EDGE_H
#define G2O_BASE_MULTI_EDGE_H

#include <iostream>
#include <iomanip>
#include <limits>

#include "base_edge.h"
#include "g2o/config.h"

namespace g2o {

  using namespace Eigen;

  /**
   * \brief base class to represent an edge connecting an arbitrary number of nodes
   *
   * D - Dimension of the measurement
   * E - type to represent the measurement
   */
  template <int D, typename E>
  class BaseMultiEdge : public BaseEdge<D,E>
  {
    public:
      /**
       * \brief helper for mapping the Hessian memory of the upper triangular block
       */
      struct HessianHelper {
        Map<MatrixXd> matrix;     ///< the mapped memory
        bool transposed;          ///< the block has to be transposed
        HessianHelper() : matrix(0, 0, 0), transposed(false) {}
      };

    public:
      static const int Dimension = BaseEdge<D,E>::Dimension;
      typedef typename BaseEdge<D,E>::Measurement Measurement;
      typedef MatrixXd JacobianType;
      typedef typename BaseEdge<D,E>::ErrorVector ErrorVector;
      typedef typename BaseEdge<D,E>::InformationType InformationType;
      typedef Map<MatrixXd, MatrixXd::Flags & AlignedBit ? Aligned : Unaligned > HessianBlockType;

      BaseMultiEdge() : BaseEdge<D,E>()
      {
      }

      /**
       * Linearizes the oplus operator in the vertex, and stores
       * the result in temporary variables _jacobianOplusXi and _jacobianOplusXj
       */
      virtual void linearizeOplus();
      
      virtual void resize(size_t size);

      virtual void constructQuadraticForm() ;

      virtual void mapHessianMemory(double* d, int i, int j, bool rowMajor);

      using BaseEdge<D,E>::computeError;

    protected:
      using BaseEdge<D,E>::_measurement;
      using BaseEdge<D,E>::_information;
      using BaseEdge<D,E>::_error;
      using BaseEdge<D,E>::_vertices;
      using BaseEdge<D,E>::_dimension;

      std::vector<HessianHelper> _hessian;
      std::vector<JacobianType> _jacobianOplus; ///< jacobians of the edge (w.r.t. oplus)

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

#include "base_multi_edge.hpp"

} // end namespace g2o

#endif
