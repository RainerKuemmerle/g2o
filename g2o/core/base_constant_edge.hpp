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
bool BaseConstantEdge<D, E, VertexTypes...>::allVerticesFixedNs(index_sequence<Ints...>) const
{
  bool fixed[] = { vertexXn<Ints>()->fixed()... };
  return std::all_of(std::begin(fixed), std::end(fixed), [](bool value){ return value; });
}

template <int D, typename E, typename... VertexTypes>
bool BaseConstantEdge<D, E, VertexTypes...>::allVerticesFixed() const
{
  return allVerticesFixedNs(make_index_sequence<_nr_of_vertices>());
}

template <int D, typename E, typename... VertexTypes>
void BaseConstantEdge<D, E, VertexTypes...>::constructQuadraticForm()
{
  constructQuadraticFormNs(make_index_sequence<_nr_of_vertices>());
}

template <int D, typename E, typename... VertexTypes>
template<std::size_t... Ints>
void BaseConstantEdge<D, E, VertexTypes...>::constructQuadraticFormNs(index_sequence<Ints...>)
{
  int unused[] = { (constructQuadraticFormN<Ints>(), 0) ... };
  (void)unused;
}

template <int D, typename E, typename... VertexTypes>
template<int N, std::size_t... Ints>
void BaseConstantEdge<D, E, VertexTypes...>::constructOffDiagonalQuadraticFormMs(index_sequence<Ints...>)
{
  int unused[] = { (constructOffDiagonalQuadraticFormM<N, Ints>(), 0) ... };
  (void)unused;
}

template <int D, typename E, typename... VertexTypes>
template<int N, int M>
void BaseConstantEdge<D, E, VertexTypes...>::constructOffDiagonalQuadraticFormM()
{
  auto& A = std::get<N>(_jacobianOplus);
  const auto& omega = this->information();
  auto AtO = A.transpose() * omega;

  // shouldn't vertices be locked to prevent writing in jacobians while reading them?
  constexpr auto K = internal::pair_to_index(M, N);
  auto& hessian = std::get<K>(_hessianTuple);
  auto& hessianTransposed = std::get<K>(_hessianTupleTransposed);
  auto to = vertexXn<M>();
  if (!(to->fixed())) {
    auto& B = std::get<M>(_jacobianOplus);
    if (_hessianRowMajor) // we have to write to the block as transposed
      hessianTransposed.noalias()+= AtO * B;
    else
      hessian.noalias()  += B.transpose() * AtO.transpose();
  }
}

template <int D, typename E, typename... VertexTypes>
template<int N>
void BaseConstantEdge<D, E, VertexTypes...>::constructQuadraticFormN()
{
  auto from = vertexXn<N>();
  auto& A = std::get<N>(_jacobianOplus);

  if (!(from->fixed())) {
    if (this->robustKernel() == 0) {

      const auto& omega = this->information();
      auto AtO = A.transpose() * omega;
      auto omega_r = (- omega * this->error()).eval();
      {
        internal::QuadraticFormLock lck(*from);
        from->b().noalias() += A.transpose() * omega_r;
        from->A().noalias() += AtO*A;
      }
      constructOffDiagonalQuadraticFormMs<N>(make_index_sequence<N>());
    } 
    else { std::cout << "Not implemented ... " << std::endl; }
  }
};

template <int D, typename E, typename... VertexTypes>
void BaseConstantEdge<D, E, VertexTypes...>::linearizeOplus(JacobianWorkspace& jacobianWorkspace)
{
  linearizeOplus_allocate(jacobianWorkspace, make_index_sequence<_nr_of_vertices>());
  linearizeOplus();
}

template <int D, typename E, typename... VertexTypes>
template<std::size_t... Ints>
void BaseConstantEdge<D, E, VertexTypes...>::linearizeOplus_allocate(JacobianWorkspace& jacobianWorkspace, index_sequence<Ints...>)
{
  int unused[] = {
  ( new (&std::get<Ints>(_jacobianOplus)) JacobianType<D, VertexDimension<Ints>()>(jacobianWorkspace.workspaceForVertex(Ints), D < 0 ? _dimension : D, VertexDimension<Ints>()) , 0 )
  ... };
  (void)unused;
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
template <std::size_t... Ints>
void BaseConstantEdge<D, E, VertexTypes...>::linearizeOplusNs(index_sequence<Ints...>)
{
  int unused[] = { (linearizeOplusN<Ints>(), 0) ... };
  (void)unused;
}

template <int D, typename E, typename... VertexTypes>
void BaseConstantEdge<D, E, VertexTypes...>::linearizeOplus()
{
  // todo:
  //if (!iNotFixed && !jNotFixed)
  //  return;

  ErrorVector errorBeforeNumeric = _error;

  linearizeOplusNs(make_index_sequence<_nr_of_vertices>());

  _error = errorBeforeNumeric;
}

struct MapHessianMemoryK
{
  number_t* d;
  template<typename HessianT>
  void operator()(HessianT& hessian)
  {
    new (&hessian) typename std::remove_reference<decltype(hessian)>::type(d);
  }
};

template <int D, typename E, typename... VertexTypes>
void BaseConstantEdge<D, E, VertexTypes...>::mapHessianMemory(number_t* d, int i, int j, bool rowMajor)
{
  if(rowMajor)
    tuple_apply_i(MapHessianMemoryK{d}, _hessianTupleTransposed, internal::pair_to_index(i, j));
  else
    tuple_apply_i(MapHessianMemoryK{d}, _hessianTuple, internal::pair_to_index(i, j));

  _hessianRowMajor = rowMajor;
}
