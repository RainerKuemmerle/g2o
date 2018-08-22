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

template <int D, typename T>
BaseVertex<D, T>::BaseVertex() :
  OptimizableGraph::Vertex(),
  _hessian(nullptr, D, D)
{
  _dimension = D;
}

template <int D, typename T>
bool BaseVertex<D, T>::resizeDimension(int newDimension) {

  // If the dimension is known at compile time, check the dimension is unchanged and always return
  if (D > 0)
    {
      assert(newDimension == D && "error resizing vertex with fixed compile time dimension where runtime dimension != compile time dimension");
      return(newDimension == D);
    }

  // Check the dimension is non-negative.
  assert(newDimension >= 0 && "error resizing vertex vertex with unknown compile time dimension where runtime dimension < 0");
  if (newDimension < 0)
    return false;

  // Nothing to do if the dimension is unchanged.
  if (newDimension == _dimension)
    return true;

  // Reset the internal state
  if (resizeDimensionImpl(newDimension) == false)
    return false;
  
  setHessianIndex(-1);
  mapHessianMemory(nullptr);
  _b.resize(newDimension);
  updateCache();

  // If the dimension is being increased and this vertex is in a
  // graph, update the size of the Jacobian workspace just in case it
  // needs to grow.
  if ((newDimension > _dimension) && (_graph != nullptr))
    {
      JacobianWorkspace& jacobianWorkspace = _graph->jacobianWorkspace();
      for (auto e : _edges)
        jacobianWorkspace.updateSize(e);
    }

  _dimension = newDimension;
  return true;
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
