// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
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

// Use this to allocate the initial dimensions. If the dimensions are
// known at compile time, then INIT_VERTEX_DIM=D. This guarantees the
// dimensions are correct, even if the constructor were fed the wrong
// dimension. If not known at compile time, then
// INIT_VERTEX_DIM=max(0, dimension). This means that a dynamic vertex
// can be constructed with a suitable default value.

#define INIT_VERTEX_DIM ((D < 0) ? ((dimension >= 0) ? dimension : 0) : D)

template <int D, typename T>
BaseVertex<D, T>::BaseVertex() :
  OptimizableGraph::Vertex(),
  _hessian(nullptr, D, D)
{
  _dimension = D;
}

template <int D, typename T>
void BaseVertex<D, T>::resizeDimension(int dimension) {
  if (D > 0)
    {
      assert(dimension == D && "error resizing vertex with fixed compile time dimension where runtime dimension != compile time dimension");
      return;
    }

  assert(dimension >= 0 && "error resizing vertex vertex with unknown compile time dimension where runtime dimension < 0");

  std::cout << __PRETTY_FUNCTION__ << ": dimension=" << dimension << "; _dimension=" << _dimension << std::endl;
  
  if (dimension != _dimension)
    {
      resizeDimensionImpl(dimension);
      _dimension = dimension;
      mapHessianMemory(nullptr);
      updateCache();
    }
}

template <int D, typename T>
number_t BaseVertex<D, T>::solveDirect(number_t lambda) {
  Eigen::Matrix<number_t, D, D, Eigen::ColMajor> tempA=_hessian + Eigen::Matrix<number_t, D, D, Eigen::ColMajor>::Identity(VERTEX_DIM, VERTEX_DIM)*lambda;
  number_t det=tempA.determinant();
  if (g2o_isnan(det) || det < std::numeric_limits<number_t>::epsilon())
    return det;
  Eigen::Matrix<number_t, D, 1, Eigen::ColMajor> dx=tempA.llt().solve(_b);
  oplus(&dx[0]);
  return det;
}

template <int D, typename T>
void BaseVertex<D, T>::clearQuadraticForm() {
  _b.setZero();
}

template <int D, typename T>
void BaseVertex<D, T>::mapHessianMemory(number_t* d)
{
  new (&_hessian) HessianBlockType(d, VERTEX_DIM, VERTEX_DIM);
}
