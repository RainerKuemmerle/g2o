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

// Use this to allocate - this ensures that, in release mode, the
// dimensions are guaranteed to be correct for a fixed sized vertex
// even if the incorrect dimensions were fed in.

#define INIT_VERTEX_DIM ((D < 0) ? dimension : D)

template <int D, typename T>
BaseVertex<D, T>::BaseVertex(int dimension) :
  OptimizableGraph::Vertex(),
  _hessian(0, INIT_VERTEX_DIM, INIT_VERTEX_DIM)
{
  if (D >= 0)
    assert(dimension == D && "error constructing vertex with fixed compile time dimension where runtime dimension != compile time dimension");
  else
    assert(dimension >= 0 && "error constructing vertex with unknown compile time dimension where runtime dimension < 0");
  _dimension = dimension;
}

template <int D, typename T>
void BaseVertex<D, T>::resize(int dimension) {
  if (D > 0)
    {
      assert(dimension == D && "error resizing vertex with fixed compile time dimension where runtime dimension != compile time dimension");
      return;
    }

  assert(dimension >= 0 && "error resizing vertex vertex with unknown compile time dimension where runtime dimension < 0");
  
  if (dimension != _dimension)
    {
      new (&_hessian) HessianBlockType(0, dimension, dimension);
      _dimension = dimension;
    }
}

template <int D, typename T>
number_t BaseVertex<D, T>::solveDirect(number_t lambda) {
  Eigen::Matrix<number_t, D, D, Eigen::ColMajor> tempA=_hessian + Eigen::Matrix<number_t, D, D, Eigen::ColMajor>::Identity()*lambda;
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

#undef INIT_VERTEX_DIM
