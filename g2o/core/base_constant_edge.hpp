// This code is based on g2o/core/base_binary_edge.hpp and
// g2o/core/base_multi_edge.hpp
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

#pragma once

namespace internal {

struct QuadraticFormLock {
  QuadraticFormLock(OptimizableGraph::Vertex& vertex) : _vertex(vertex) {
#ifdef G2O_OPENMP
    _vertex.lockQuadraticForm();
#endif
  }

  ~QuadraticFormLock() {
#ifdef G2O_OPENMP
    _vertex.unlockQuadraticForm();
#endif
  }

private:
  OptimizableGraph::Vertex& _vertex;
};

} // anonymous namespace

template <int D, typename E, typename... VertexTypes>
void BaseConstantEdge<D, E, VertexTypes...>::resize(size_t size)
{
  assert(size == _nr_of_vertices && "attempting to resize a constant size edge");
  BaseEdge<D, E>::resize(size);
}

template <int D, typename E, typename... VertexTypes>
template<std::size_t... Ints >
bool BaseConstantEdge<D, E, VertexTypes...>::allVerticesFixedNs(std::integer_sequence<std::size_t, Ints...>) const
{
  return ( ... && vertexXn<Ints>()->fixed());
}

template <int D, typename E, typename... VertexTypes>
bool BaseConstantEdge<D, E, VertexTypes...>::allVerticesFixed() const
{
  return allVerticesFixedNs(std::make_index_sequence<_nr_of_vertices>());
}

template <int D, typename E, typename... VertexTypes>
void BaseConstantEdge<D, E, VertexTypes...>::constructQuadraticForm()
{
  constructQuadraticFormKs(std::make_integer_sequence<int, _nr_of_vertex_pairs>());
}

template <int D, typename E, typename... VertexTypes>
template<int... Ints>
void BaseConstantEdge<D, E, VertexTypes...>::constructQuadraticFormKs(std::integer_sequence<int, Ints...>)
{
  ( void(constructQuadraticFormK<Ints>()), ...);
}

template <int D, typename E, typename... VertexTypes>
template<int K>
void BaseConstantEdge<D, E, VertexTypes...>::constructQuadraticFormK()
{
  constexpr auto NM = internal::index_to_pair(K);

  auto from = vertexXn<NM.first>();
  auto to = vertexXn<NM.second>();
  auto& A = std::get<NM.first>(_jacobianOplus);
  auto& B = std::get<NM.second>(_jacobianOplus);
  auto& hessian = std::get<K>(_hessianTuple);
  auto& hessianTransposed = std::get<K>(_hessianTupleTransposed);

  bool fromNotFixed = !(from->fixed());
  bool toNotFixed = !(to->fixed());

  if (fromNotFixed || toNotFixed) {
    const auto& omega = this->information();
    auto omega_r = (- omega * this->error()).eval();
    if (this->robustKernel() == 0) {
      if (fromNotFixed) {
        auto AtO = A.transpose() * omega;

        {
          internal::QuadraticFormLock lck(*from);

          from->b().noalias() += A.transpose() * omega_r;
          from->A().noalias() += AtO*A;
        }

        if (toNotFixed ) {
          if (_hessianRowMajor) // we have to write to the block as transposed
            hessianTransposed.noalias() += B.transpose() * AtO.transpose();
          else
            hessian.noalias() += AtO * B;
        }
      }
      if (toNotFixed) {
        internal::QuadraticFormLock lck(*to);

        to->b().noalias() += B.transpose() * omega_r;
        to->A().noalias() += B.transpose() * omega * B;
      }
    } else { // robust (weighted) error according to some kernel
      number_t error = this->chi2();
      Vector3 rho;
      this->robustKernel()->robustify(error, rho);
      auto weightedOmega = rho[1] * omega; // this->robustInformation(rho);
      //std::cout << PVAR(rho.transpose()) << std::endl;
      //std::cout << PVAR(weightedOmega) << std::endl;

      omega_r *= rho[1];
      if (fromNotFixed) {
        {
          internal::QuadraticFormLock lck(*from);

          from->b().noalias() += A.transpose() * omega_r;
          from->A().noalias() += A.transpose() * weightedOmega * A;
        }

        if (toNotFixed ) {
          if (_hessianRowMajor) // we have to write to the block as transposed
            hessianTransposed.noalias() += B.transpose() * weightedOmega * A;
          else
            hessian.noalias() += A.transpose() * weightedOmega * B;
        }
      }
      if (toNotFixed) {
        internal::QuadraticFormLock lck(*to);

        to->b().noalias() += B.transpose() * omega_r;
        to->A().noalias() += B.transpose() * weightedOmega * B;
      }
    }
  }
};

template <int D, typename E, typename... VertexTypes>
void BaseConstantEdge<D, E, VertexTypes...>::linearizeOplus(JacobianWorkspace& jacobianWorkspace)
{
  linearizeOplus_allocate(jacobianWorkspace, std::make_integer_sequence<int, _nr_of_vertices>());
  linearizeOplus();
}

template <int D, typename E, typename... VertexTypes>
template<int... Ints>
void BaseConstantEdge<D, E, VertexTypes...>::linearizeOplus_allocate(JacobianWorkspace& jacobianWorkspace, std::integer_sequence<int, Ints...>)
{
  ( new (&std::get<Ints>(_jacobianOplus)) JacobianType<D, VertexDimension<Ints>()>(jacobianWorkspace.workspaceForVertex(Ints), D < 0 ? _dimension : D, VertexDimension<Ints>()) , ...);
}

template <int D, typename E, typename... VertexTypes>
template<int N>
void BaseConstantEdge<D, E, VertexTypes...>::linearizeOplusN()
{
  auto& jacobianOplus = std::get<N>(_jacobianOplus);
  auto vertex = vertexXn<N>();

  const number_t delta = cst(1e-9);
  const number_t scalar = 1 / (2*delta);

  bool iNotFixed = !(vertex->fixed());
  if (iNotFixed) {
    internal::QuadraticFormLock lck(*vertex);
    // estimate the jacobian numerically
    number_t add_vertex[VertexDimension<N>()] = {};

    // add small step along the unit vector in each dimension
    for (int d = 0; d < VertexDimension<N>(); ++d) {
      vertex->push();
      add_vertex[d] = delta;
      vertex->oplus(add_vertex);
      computeError();
      auto errorBak = this->error();
      vertex->pop();
      vertex->push();
      add_vertex[d] = -delta;
      vertex->oplus(add_vertex);
      computeError();
      errorBak -= this->error();
      vertex->pop();
      add_vertex[d] = 0.0;

      jacobianOplus.col(d) = scalar * errorBak;
    } // end dimension
  }
};

template <int D, typename E, typename... VertexTypes>
template <int... Ints>
void BaseConstantEdge<D, E, VertexTypes...>::linearizeOplusNs(std::integer_sequence<int, Ints...>)
{
  ( void(linearizeOplusN<Ints>()), ...);
}

template <int D, typename E, typename... VertexTypes>
void BaseConstantEdge<D, E, VertexTypes...>::linearizeOplus()
{
  // todo:
  //if (!iNotFixed && !jNotFixed)
  //  return;

  ErrorVector errorBeforeNumeric = _error;

  linearizeOplusNs(std::make_integer_sequence<int, _nr_of_vertices>());

  _error = errorBeforeNumeric;
}

template <int D, typename E, typename... VertexTypes>
void BaseConstantEdge<D, E, VertexTypes...>::mapHessianMemory(number_t* d, int i, int j, bool rowMajor)
{
  auto f = [d](auto& hessian){ new (&hessian) std::remove_reference_t<decltype(hessian)>(d); };
  if(rowMajor)
    tuple_apply_i(f, _hessianTupleTransposed, internal::pair_to_index(i, j));
  else
    tuple_apply_i(f, _hessianTuple, internal::pair_to_index(i, j));

  _hessianRowMajor = rowMajor;
}
