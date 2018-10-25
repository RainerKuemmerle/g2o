// This code is based on g2o/core/base_binary_edge.h and
// g2o/core/base_multi_edge.h
//
// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, H. Strasdat, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef G2O_BASE_CONSTANT_EDGE_H
#define G2O_BASE_CONSTANT_EDGE_H


#include <iostream>
#include <limits>

#include "base_edge.h"
#include "robust_kernel.h"
#include "g2o/config.h"
#include "g2o/stuff/misc.h"
#include "g2o/stuff/tuple_tools.h"

namespace g2o {

  namespace internal {
    // assumes i < j
    constexpr int pair_to_index(const int i, const int j)
    {
      return j * (j - 1) / 2 + i;
    }

    constexpr std::pair<int, int> index_to_pair(const int k)
    {
      return {k - (ct_sqrt(2*(k+1))+.5) * (ct_sqrt(2*(k+1))+.5 - 1) / 2, ct_sqrt(2*(k+1))+.5};
    }
  }

  template <int D, typename E, typename... VertexTypes>
  class BaseConstantEdge : public BaseEdge<D, E>
  {
    public:
      template<int N, typename... Types>
      using NthType = typename std::tuple_element<N, std::tuple<Types...>>::type;
      template<int VertexN>
      using VertexXnType = NthType<VertexN, VertexTypes...>;
      template<int VertexN>
      static constexpr int VertexDimension() { return VertexXnType<VertexN>::Dimension; };
      template<int VertexN>
      const VertexXnType<VertexN>* vertexXn() const
      {
        return static_cast<const VertexXnType<VertexN>*>(_vertices[VertexN]);
      }
      template<int VertexN>
      VertexXnType<VertexN>* vertexXn()
      {
        return static_cast<VertexXnType<VertexN>*>(_vertices[VertexN]);
      }

      static const int Dimension = BaseEdge<D, E>::Dimension;
      typedef typename BaseEdge<D,E>::Measurement Measurement;

      typedef typename BaseEdge<D,E>::ErrorVector ErrorVector;
      typedef typename BaseEdge<D,E>::InformationType InformationType;

      template<int EdgeDimension, int VertexDimension>
      using JacobianType = typename Eigen::Matrix<number_t, EdgeDimension, VertexDimension, EdgeDimension==1?Eigen::RowMajor:Eigen::ColMajor>::AlignedMapType;
      //! it requires quite some ugly code to get the type of hessians...
      template<int DN, int DM>
      using HessianBlockType = Eigen::Map<Eigen::Matrix<number_t, DN, DM, DN==1?Eigen::RowMajor:Eigen::ColMajor>,
                                          Eigen::Matrix<number_t, DN, DM, DN==1?Eigen::RowMajor:Eigen::ColMajor>::Flags & Eigen::PacketAccessBit ? Eigen::Aligned : Eigen::Unaligned >;
      template<int K>
      using HessianBlockTypeK = HessianBlockType<VertexDimension<internal::index_to_pair(K).first>(), VertexDimension<internal::index_to_pair(K).second>()>;
      template<int K>
      using HessianBlockTypeKTransposed = HessianBlockType<VertexDimension<internal::index_to_pair(K).second>(), VertexDimension<internal::index_to_pair(K).first>()>;
      template<typename> struct HessianTupleType;
      template<std::size_t... Ints >
      struct HessianTupleType<index_sequence<Ints...>>
      {
        using type = std::tuple<HessianBlockTypeK<Ints>...>;
        using typeTransposed = std::tuple<HessianBlockTypeKTransposed<Ints>...>;
      };
      static const std::size_t _nr_of_vertices = sizeof...(VertexTypes);
      static const std::size_t _nr_of_vertex_pairs = internal::pair_to_index(0, _nr_of_vertices);
      using HessianTuple = typename HessianTupleType<make_index_sequence<_nr_of_vertex_pairs>>::type;
      using HessianTupleTransposed = typename HessianTupleType<make_index_sequence<_nr_of_vertex_pairs>>::typeTransposed;

      BaseConstantEdge() : BaseEdge<D,E>(),
      _hessianRowMajor(false),
      _hessianTuple(tuple_init(nullptr, _hessianTuple)),
      _hessianTupleTransposed(tuple_init(nullptr, _hessianTupleTransposed)),
      _jacobianOplus({nullptr, D, VertexTypes::Dimension}...)
      {
        _vertices.resize(_nr_of_vertices, nullptr);
      }

      inline virtual OptimizableGraph::Vertex* createVertex(int)
      {
        assert(false && "createVertex is not implemented for BaseConstantEdge");
        return nullptr;
      };

      inline virtual void resize(size_t size);

      template<std::size_t... Ints >
      bool allVerticesFixedNs(index_sequence<Ints...>) const;
      inline virtual bool allVerticesFixed() const;

      inline virtual void linearizeOplus(JacobianWorkspace& jacobianWorkspace);
      template<std::size_t... Ints>
      void linearizeOplus_allocate(JacobianWorkspace& jacobianWorkspace, index_sequence<Ints...>);

      /**
       * Linearizes the oplus operator in the vertex, and stores
       * the result in temporary variables _jacobianOplus
       */
      inline virtual void linearizeOplus();
      template<std::size_t... Ints>
      void linearizeOplusNs(index_sequence<Ints...>);
      template<int N>
      void linearizeOplusN();

      //! returns the result of the linearization in the manifold space for the nodes xn
      template<int N>
      const typename std::tuple_element<N, std::tuple<JacobianType<D, VertexTypes::Dimension>...>>::type&
        jacobianOplusXn() const { return std::get<N>(_jacobianOplus);}

      inline virtual void constructQuadraticForm() ;
      template<std::size_t... Ints>
      void constructQuadraticFormKs(index_sequence<Ints...>);
      template<int K>
      void constructQuadraticFormK();

      inline virtual void mapHessianMemory(number_t* d, int i, int j, bool rowMajor);

      using BaseEdge<D,E>::resize;
      using BaseEdge<D,E>::computeError;

    protected:
      using BaseEdge<D,E>::_measurement;
      using BaseEdge<D,E>::_information;
      using BaseEdge<D,E>::_error;
      using BaseEdge<D,E>::_vertices;
      using BaseEdge<D,E>::_dimension;

      bool _hessianRowMajor;
      HessianTuple _hessianTuple;
      HessianTupleTransposed _hessianTupleTransposed;
      std::tuple<JacobianType<D, VertexTypes::Dimension>...> _jacobianOplus;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

#include "base_constant_edge.hpp"

} // end namespace g2o

#endif
