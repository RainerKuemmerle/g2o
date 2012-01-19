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

namespace {
  inline int computeUpperTriangleIndex(int i, int j)
  {
    int elemsUpToCol = ((j-1) * j) / 2;
    return elemsUpToCol + i;
  }
}

template <int D, typename E>
void BaseMultiEdge<D, E>::constructQuadraticForm()
{
  const InformationType& omega = _information;
  Matrix<double, D, 1> omega_r = - omega * _error;

  for (size_t i = 0; i < _vertices.size(); ++i) {
    OptimizableGraph::Vertex* from = static_cast<OptimizableGraph::Vertex*>(_vertices[i]);
    bool istatus = !(from->fixed());

    if (istatus) {
      const MatrixXd& A = _jacobianOplus[i];

      MatrixXd AtO = A.transpose() * omega;
      int fromDim = from->dimension();
      Map<MatrixXd> fromMap(from->hessianData(), fromDim, fromDim);
      Map<VectorXd> fromB(from->bData(), fromDim);

      // ii block in the hessian
#ifdef G2O_OPENMP
      from->lockQuadraticForm();
#endif
      fromMap.noalias() += AtO * A;
      fromB.noalias() += A.transpose() * omega_r;

      // compute the off-diagonal blocks ij for all j
      for (size_t j = i+1; j < _vertices.size(); ++j) {
        OptimizableGraph::Vertex* to = static_cast<OptimizableGraph::Vertex*>(_vertices[j]);
#ifdef G2O_OPENMP
        to->lockQuadraticForm();
#endif
        bool jstatus = !(to->fixed());
        if (jstatus) {
          const MatrixXd& B = _jacobianOplus[j];
          int idx = computeUpperTriangleIndex(i, j);
          assert(idx < (int)_hessian.size());
          HessianHelper& hhelper = _hessian[idx];
          if (hhelper.transposed) { // we have to write to the block as transposed
            hhelper.matrix.noalias() += B.transpose() * AtO.transpose();
          } else {
            hhelper.matrix.noalias() += AtO * B;
          }
        }
#ifdef G2O_OPENMP
        to->unlockQuadraticForm();
#endif
      }

#ifdef G2O_OPENMP
      from->unlockQuadraticForm();
#endif
    }

  }

}

template <int D, typename E>
void BaseMultiEdge<D, E>::linearizeOplus()
{
#ifdef G2O_OPENMP
  for (size_t i = 0; i < _vertices.size(); ++i) {
    OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(_vertices[i]);
    v->lockQuadraticForm();
  }
#endif

  const double delta = 1e-9;
  const double scalar = 1.0 / (2*delta);
  ErrorVector errorBak;
  ErrorVector errorBeforeNumeric = _error;

  for (size_t i = 0; i < _vertices.size(); ++i) {
    //Xi - estimate the jacobian numerically
    OptimizableGraph::Vertex* vi = static_cast<OptimizableGraph::Vertex*>(_vertices[i]);

    if (vi->fixed())
      continue;

    const int vi_dim = vi->dimension();
#ifdef _MSC_VER
    double* add_vi = new double[vi_dim];
#else
    double add_vi[vi_dim];
#endif
    std::fill(add_vi, add_vi + vi_dim, 0.0);
    if (_jacobianOplus[i].rows() != _dimension || _jacobianOplus[i].cols() != vi_dim)
      _jacobianOplus[i].resize(_dimension, vi_dim);
    // add small step along the unit vector in each dimension
    for (int d = 0; d < vi_dim; ++d) {
      vi->push();
      add_vi[d] = delta;
      vi->oplus(add_vi);
      computeError();
      errorBak = _error;
      vi->pop();
      vi->push();
      add_vi[d] = -delta;
      vi->oplus(add_vi);
      computeError();
      errorBak -= _error;
      vi->pop();
      add_vi[d] = 0.0;

      _jacobianOplus[i].col(d) = scalar * errorBak;
    } // end dimension
#ifdef _MSC_VER
    delete[] add_vi;
#endif
  }
  _error = errorBeforeNumeric;

#ifdef G2O_OPENMP
  for (int i = (int)(_vertices.size()) - 1; i >= 0; --i) {
    OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(_vertices[i]);
    v->unlockQuadraticForm();
  }
#endif

}

template <int D, typename E>
void BaseMultiEdge<D, E>::mapHessianMemory(double* d, int i, int j, bool rowMajor)
{
  int idx = computeUpperTriangleIndex(i, j);
  assert(idx < (int)_hessian.size());
  OptimizableGraph::Vertex* vi = static_cast<OptimizableGraph::Vertex*>(_vertices[i]);
  OptimizableGraph::Vertex* vj = static_cast<OptimizableGraph::Vertex*>(_vertices[j]);
  HessianHelper& h = _hessian[idx];
  if (rowMajor) {
    if (h.matrix.data() != d || h.transposed != rowMajor)
      new (&h.matrix) HessianBlockType(d, vj->dimension(), vi->dimension());
  } else {
    if (h.matrix.data() != d || h.transposed != rowMajor)
      new (&h.matrix) HessianBlockType(d, vi->dimension(), vj->dimension());
  }
  h.transposed = rowMajor;
}

template <int D, typename E>
void BaseMultiEdge<D, E>::resize(size_t size)
{
  BaseEdge<D,E>::resize(size);
  int n = (int)_vertices.size();
  int maxIdx = (n * (n-1))/2;
  assert(maxIdx >= 0);
  _hessian.resize(maxIdx);
  _jacobianOplus.resize(size);
}
