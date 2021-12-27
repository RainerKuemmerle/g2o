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

template <int D, typename E, typename... VertexTypes>
void BaseFixedSizedEdge<D, E, VertexTypes...>::resize(size_t size) {
  assert(size == _nr_of_vertices &&
         "attempting to resize a constant size edge");
  BaseEdge<D, E>::resize(size);
}

template <int D, typename E, typename... VertexTypes>
template <std::size_t... Ints>
bool BaseFixedSizedEdge<D, E, VertexTypes...>::allVerticesFixedNs(
    std::index_sequence<Ints...>) const {
  bool fixed[] = {vertexXn<Ints>()->fixed()...};
  return std::all_of(std::begin(fixed), std::end(fixed),
                     [](bool value) { return value; });
}

template <int D, typename E, typename... VertexTypes>
bool BaseFixedSizedEdge<D, E, VertexTypes...>::allVerticesFixed() const {
  return allVerticesFixedNs(std::make_index_sequence<_nr_of_vertices>());
}

template <int D, typename E, typename... VertexTypes>
void BaseFixedSizedEdge<D, E, VertexTypes...>::constructQuadraticForm() {
  if (this->robustKernel()) {
    number_t error = this->chi2();
    Vector3 rho;
    this->robustKernel()->robustify(error, rho);
    Eigen::Matrix<number_t, D, 1, Eigen::ColMajor> omega_r =
        -_information * _error;
    omega_r *= rho[1];
    constructQuadraticFormNs(this->robustInformation(rho), omega_r,
                             std::make_index_sequence<_nr_of_vertices>());
  } else {
    constructQuadraticFormNs(_information, -_information * _error,
                             std::make_index_sequence<_nr_of_vertices>());
  }
}

template <int D, typename E, typename... VertexTypes>
template <std::size_t... Ints>
void BaseFixedSizedEdge<D, E, VertexTypes...>::constructQuadraticFormNs(
    const InformationType& omega, const ErrorVector& weightedError,
    std::index_sequence<Ints...>) {
  int unused[] = {(constructQuadraticFormN<Ints>(omega, weightedError), 0)...};
  (void)unused;
}

// overloading constructOffDiagonalQuadraticFormMs to
// prevent MSVC error when index_sequence is empty
template <int D, typename E, typename... VertexTypes>
template <int N, typename AtOType>
void BaseFixedSizedEdge<D, E, VertexTypes...>::
    constructOffDiagonalQuadraticFormMs(const AtOType&, std::index_sequence<>) {
}

template <int D, typename E, typename... VertexTypes>
template <int N, std::size_t... Ints, typename AtOType>
void BaseFixedSizedEdge<D, E, VertexTypes...>::
    constructOffDiagonalQuadraticFormMs(const AtOType& AtO,
                                        std::index_sequence<Ints...>) {
  int unused[] = {
      (constructOffDiagonalQuadraticFormM<N, Ints, AtOType>(AtO), 0)...};
  (void)unused;
}

template <int D, typename E, typename... VertexTypes>
template <int N, int M, typename AtOType>
void BaseFixedSizedEdge<D, E, VertexTypes...>::
    constructOffDiagonalQuadraticFormM(const AtOType& AtO) {
  constexpr auto fromId = N;
  constexpr auto toId = N + M + 1;
  assert(fromId < toId && "Index mixed up");
  auto to = vertexXn<toId>();
  if (!to->fixed()) {
    const auto& B = std::get<toId>(_jacobianOplus);
    constexpr auto K = internal::pair_to_index(fromId, toId);
    internal::QuadraticFormLock lck(*to);
    (void)lck;
    if (_hessianRowMajor[K]) {  // we have to write to the block as transposed
      auto& hessianTransposed = std::get<K>(_hessianTupleTransposed);
      hessianTransposed.noalias() += B.transpose() * AtO.transpose();
    } else {
      auto& hessian = std::get<K>(_hessianTuple);
      hessian.noalias() += AtO * B;
    }
  }
}

template <int D, typename E, typename... VertexTypes>
template <int N>
void BaseFixedSizedEdge<D, E, VertexTypes...>::constructQuadraticFormN(
    const InformationType& omega, const ErrorVector& weightedError) {
  auto from = vertexXn<N>();
  const auto& A = std::get<N>(_jacobianOplus);

  if (!(from->fixed())) {
    const auto AtO = A.transpose() * omega;
    {
      internal::QuadraticFormLock lck(*from);
      (void)lck;
      from->b().noalias() += A.transpose() * weightedError;
      from->A().noalias() += AtO * A;
    }
    constructOffDiagonalQuadraticFormMs<N>(
        AtO, std::make_index_sequence<_nr_of_vertices - N - 1>());
  }
};

template <int D, typename E, typename... VertexTypes>
void BaseFixedSizedEdge<D, E, VertexTypes...>::linearizeOplus(
    JacobianWorkspace& jacobianWorkspace) {
  linearizeOplus_allocate(jacobianWorkspace,
                          std::make_index_sequence<_nr_of_vertices>());
  linearizeOplus();
}

template <int D, typename E, typename... VertexTypes>
template <std::size_t... Ints>
void BaseFixedSizedEdge<D, E, VertexTypes...>::linearizeOplus_allocate(
    JacobianWorkspace& jacobianWorkspace, std::index_sequence<Ints...>) {
  int unused[] = {
      (new (&std::get<Ints>(_jacobianOplus))
           JacobianType<D, VertexDimension<Ints>()>(
               jacobianWorkspace.workspaceForVertex(Ints),
               D < 0 ? _dimension : D,
               VertexDimension<Ints>() < 0 ? vertexXn<Ints>()->dimension()
                                           : VertexDimension<Ints>()),
       0)...};
  (void)unused;
}

template <int D, typename E, typename... VertexTypes>
template <int N>
void BaseFixedSizedEdge<D, E, VertexTypes...>::linearizeOplusN() {
  auto vertex = vertexXn<N>();

  if (vertex->fixed()) return;

  auto& jacobianOplus = std::get<N>(_jacobianOplus);

  constexpr number_t delta = cst(1e-9);
  constexpr number_t scalar = 1 / (2 * delta);

  internal::QuadraticFormLock lck(*vertex);
  (void)lck;

  typedef typename std::conditional<
      VertexXnType<N>::Dimension == -1, ceres::internal::FixedArray<number_t>,
      ceres::internal::FixedArray<
          number_t, static_cast<size_t>(VertexXnType<N>::Dimension)> >::type
      FixedArray;
  FixedArray add_vertex(vertexDimension<N>());
  add_vertex.fill(0.);

  // estimate the jacobian numerically
  // add small step along the unit vector in each dimension
  for (int d = 0; d < vertexDimension<N>(); ++d) {
    vertex->push();
    add_vertex[d] = delta;
    vertex->oplus(add_vertex.data());
    computeError();
    auto errorBak = this->error();
    vertex->pop();
    vertex->push();
    add_vertex[d] = -delta;
    vertex->oplus(add_vertex.data());
    computeError();
    errorBak -= this->error();
    vertex->pop();
    add_vertex[d] = 0.0;

    jacobianOplus.col(d) = scalar * errorBak;
  }  // end dimension
}

template <int D, typename E, typename... VertexTypes>
template <std::size_t... Ints>
void BaseFixedSizedEdge<D, E, VertexTypes...>::linearizeOplusNs(
    std::index_sequence<Ints...>) {
  int unused[] = {(linearizeOplusN<Ints>(), 0)...};
  (void)unused;
}

template <int D, typename E, typename... VertexTypes>
void BaseFixedSizedEdge<D, E, VertexTypes...>::linearizeOplus() {
  if (allVerticesFixed()) return;
  ErrorVector errorBeforeNumeric = _error;
  linearizeOplusNs(std::make_index_sequence<_nr_of_vertices>());
  _error = errorBeforeNumeric;
}

/**
 * Helper functor class to construct the Hessian Eigen::Map object.
 * We have to pass the size at runtime to allow dynamically sized verices.
 */
struct MapHessianMemoryK {
  number_t* d;
  int rows;
  int cols;
  template <typename HessianT>
  void operator()(HessianT& hessian) {
    new (&hessian)
        typename std::remove_reference<decltype(hessian)>::type(d, rows, cols);
  }
};

template <int D, typename E, typename... VertexTypes>
void BaseFixedSizedEdge<D, E, VertexTypes...>::mapHessianMemory(number_t* d,
                                                                int i, int j,
                                                                bool rowMajor) {
  assert(i < j && "index assumption violated");
  // get the size of the vertices
  int vi_dim =
      static_cast<OptimizableGraph::Vertex*>(HyperGraph::Edge::vertex(i))
          ->dimension();
  int vj_dim =
      static_cast<OptimizableGraph::Vertex*>(HyperGraph::Edge::vertex(j))
          ->dimension();
  int k = internal::pair_to_index(i, j);
  _hessianRowMajor[k] = rowMajor;
  if (rowMajor)
    tuple_apply_i(MapHessianMemoryK{d, vj_dim, vi_dim}, _hessianTupleTransposed,
                  k);
  else
    tuple_apply_i(MapHessianMemoryK{d, vi_dim, vj_dim}, _hessianTuple, k);
}
