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

namespace g2o {

  namespace {
    // assumes i < j
    constexpr int pair_to_index(const int i, const int j)
    {
      return j * (j - 1) / 2 + i;
    }
    constexpr std::pair<int, int> index_to_pair(const int k)
    {
      const int j = std::sqrt(2*(k+1))+.5;
      const int i = k - j * (j - 1) / 2;
      return {i, j};
    }

    template<int N, typename... Types>
    using NthType = typename std::tuple_element<N, std::tuple<Types...>>::type;

    template<typename Value, typename... Ts2>
    auto tuple_init(const Value& value, const std::tuple<Ts2...>)
    {
      return std::tuple<Ts2...>{Ts2{value}...};
    }

    template<int I>
    struct Tuple_apply_i
    {
      template<typename F, typename T>
      void operator()(F&& f, T& t, int i)
      {
        if(i == I)
          return f(std::get<I>(t));
        else
          return Tuple_apply_i<I - 1>()(f, t, i);
      }
    };

    template<>
    struct Tuple_apply_i<-1>
    {
      template<typename F, typename T>
      void operator()(F&&, T&, int) { }
    };

    template<typename F, typename T>
    void tuple_apply_i(F&& f, T& t, int i)
    {
      Tuple_apply_i<std::tuple_size<T>::value -1>()(f, t, i);
    }


    template<int EdgeDimension, int VertexDimension>
    using JacobianType = typename Eigen::Matrix<number_t, EdgeDimension, VertexDimension, EdgeDimension==1?Eigen::RowMajor:Eigen::ColMajor>::AlignedMapType;
  }

  template <int D, typename E, typename... VertexTypes>
  class BaseConstantEdge : public BaseEdge<D, E>
  {
    public:
      template<int VertexN>
      using VertexXnType = NthType<VertexN, VertexTypes...>;
      template<int VertexN>
      static constexpr int VertexDimension() { return VertexXnType<VertexN>::Dimension; };
      template<int VertexN>
      const auto vertexXn() const
      {
        return static_cast<const VertexXnType<VertexN>*>(_vertices[VertexN]);
      }
      template<int VertexN>
      auto vertexXn()
      {
        return static_cast<VertexXnType<VertexN>*>(_vertices[VertexN]);
      }

      static const int Dimension = BaseEdge<D, E>::Dimension;
      typedef typename BaseEdge<D,E>::Measurement Measurement;

      typedef typename BaseEdge<D,E>::ErrorVector ErrorVector;
      typedef typename BaseEdge<D,E>::InformationType InformationType;

      //! it requires quite some ugly code to get the type of hessians...
      template<int DN, int DM>
      using HessianBlockType = Eigen::Map<Eigen::Matrix<number_t, DN, DM, DN==1?Eigen::RowMajor:Eigen::ColMajor>,
                                          Eigen::Matrix<number_t, DN, DM, DN==1?Eigen::RowMajor:Eigen::ColMajor>::Flags & Eigen::PacketAccessBit ? Eigen::Aligned : Eigen::Unaligned >;
      template<int K>
      using HessianBlockTypeK = HessianBlockType<VertexDimension<index_to_pair(K).first>(), VertexDimension<index_to_pair(K).second>()>;
      template<int K>
      using HessianBlockTypeKTransposed = HessianBlockType<VertexDimension<index_to_pair(K).second>(), VertexDimension<index_to_pair(K).first>()>;
      template<typename> struct HessianTupleType;
      template<std::size_t... Ints >
      struct HessianTupleType<std::integer_sequence<std::size_t, Ints...>>
      {
        using type = std::tuple<HessianBlockTypeK<Ints>...>;
        using typeTransposed = std::tuple<HessianBlockTypeKTransposed<Ints>...>;
      };
      static const auto _nr_of_vertices = sizeof...(VertexTypes);
      static const auto _nr_of_vertex_pairs = pair_to_index(0, _nr_of_vertices);
      using HessianTuple = typename HessianTupleType<std::make_index_sequence<_nr_of_vertex_pairs>>::type;
      using HessianTupleTransposed = typename HessianTupleType<std::make_index_sequence<_nr_of_vertex_pairs>>::typeTransposed;

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
      bool allVerticesFixedNs(std::integer_sequence<std::size_t, Ints...>) const;
      inline virtual bool allVerticesFixed() const;

      inline virtual void linearizeOplus(JacobianWorkspace& jacobianWorkspace);
      template<int... Ints>
      void linearizeOplus_allocate(JacobianWorkspace& jacobianWorkspace, std::integer_sequence<int, Ints...>);

      /**
       * Linearizes the oplus operator in the vertex, and stores
       * the result in temporary variables _jacobianOplus
       */
      inline virtual void linearizeOplus();
      template<int... Ints>
      void linearizeOplusNs(std::integer_sequence<int, Ints...>);
      template<int N>
      void linearizeOplusN();

      //! returns the result of the linearization in the manifold space for the nodes xn
      template<int N>
      const auto& jacobianOplusXn() const { return std::get<N>(_jacobianOplus);}

      inline virtual void constructQuadraticForm() ;
      template<int... Ints>
      void constructQuadraticFormKs(std::integer_sequence<int, Ints...>);
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
