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

template <int D, typename E, typename VertexXiType, typename VertexXjType>
OptimizableGraph::Vertex* BaseBinaryEdge<D, E, VertexXiType, VertexXjType>::createFrom(){
  return new VertexXiType();
}

template <int D, typename E, typename VertexXiType, typename VertexXjType>
OptimizableGraph::Vertex* BaseBinaryEdge<D, E, VertexXiType, VertexXjType>::createTo(){
  return new VertexXjType();
}


template <int D, typename E, typename VertexXiType, typename VertexXjType>
void BaseBinaryEdge<D, E, VertexXiType, VertexXjType>::resize(size_t size)
{
  if (size != 2) {
    std::cerr << "WARNING, attempting to resize binary edge " << BaseEdge<D, E>::id() << " to " << size << std::endl;
  }
  BaseEdge<D, E>::resize(size);
}

template <int D, typename E, typename VertexXiType, typename VertexXjType>
void BaseBinaryEdge<D, E, VertexXiType, VertexXjType>::constructQuadraticForm()
{
  VertexXiType* from = static_cast<VertexXiType*>(_vertices[0]);
  VertexXjType* to   = static_cast<VertexXjType*>(_vertices[1]);

  // get the Jacobian of the nodes in the manifold domain
  const JacobianXiOplusType& A = jacobianOplusXi();
  const JacobianXjOplusType& B = jacobianOplusXj();

  const InformationType& omega = _information;

  bool fromNotFixed = !(from->fixed());
  bool toNotFixed = !(to->fixed());

  if (fromNotFixed || toNotFixed) {
#ifdef G2O_OPENMP
    from->lockQuadraticForm();
    to->lockQuadraticForm();
#endif
    Matrix<double, D, 1> omega_r = - omega * _error;
    if (fromNotFixed) {
      Matrix<double, VertexXiType::Dimension, D> AtO = A.transpose() * omega;
      from->b().noalias() += A.transpose() * omega_r;
      from->A().noalias() += AtO*A;
      if (toNotFixed ) {
        if (_hessianRowMajor) // we have to write to the block as transposed
          _hessianTransposed.noalias() += B.transpose() * AtO.transpose();
        else
          _hessian.noalias() += AtO * B;
      }
    } 
    if (toNotFixed ) {
      to->b().noalias() += B.transpose() * omega_r;
      to->A().noalias() += B.transpose() * omega * B;
    }
#ifdef G2O_OPENMP
    to->unlockQuadraticForm();
    from->unlockQuadraticForm();
#endif
  }
}

template <int D, typename E, typename VertexXiType, typename VertexXjType>
void BaseBinaryEdge<D, E, VertexXiType, VertexXjType>::linearizeOplus()
{
  VertexXiType* vi = static_cast<VertexXiType*>(_vertices[0]);
  VertexXjType* vj = static_cast<VertexXjType*>(_vertices[1]);

  bool iNotFixed = !(vi->fixed());
  bool jNotFixed = !(vj->fixed());

  if (!iNotFixed && !jNotFixed)
    return;

#ifdef G2O_OPENMP
  vi->lockQuadraticForm();
  vj->lockQuadraticForm();
#endif

  const double delta = 1e-9;
  const double scalar = 1.0 / (2*delta);
  ErrorVector errorBak;
  ErrorVector errorBeforeNumeric = _error;

  if (iNotFixed) {
    //Xi - estimate the jacobian numerically
    double add_vi[VertexXiType::Dimension];
    std::fill(add_vi, add_vi + VertexXiType::Dimension, 0.0);
    // add small step along the unit vector in each dimension
    for (int d = 0; d < VertexXiType::Dimension; ++d) {
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

      _jacobianOplusXi.col(d) = scalar * errorBak;
    } // end dimension
  }

  if (jNotFixed) {
    //Xj - estimate the jacobian numerically
    double add_vj[VertexXjType::Dimension];
    std::fill(add_vj, add_vj + VertexXjType::Dimension, 0.0);
    // add small step along the unit vector in each dimension
    for (int d = 0; d < VertexXjType::Dimension; ++d) {
      vj->push();
      add_vj[d] = delta;
      vj->oplus(add_vj);
      computeError();
      errorBak = _error;
      vj->pop();
      vj->push();
      add_vj[d] = -delta;
      vj->oplus(add_vj);
      computeError();
      errorBak -= _error;
      vj->pop();
      add_vj[d] = 0.0;

      _jacobianOplusXj.col(d) = scalar * errorBak;
    }
  } // end dimension

  _error = errorBeforeNumeric;
#ifdef G2O_OPENMP
  vj->unlockQuadraticForm();
  vi->unlockQuadraticForm();
#endif
}

template <int D, typename E, typename VertexXiType, typename VertexXjType>
void BaseBinaryEdge<D, E, VertexXiType, VertexXjType>::mapHessianMemory(double* d, int i, int j, bool rowMajor)
{
  (void) i; (void) j;
  //assert(i == 0 && j == 1);
  if (rowMajor) {
    new (&_hessianTransposed) HessianBlockTransposedType(d, VertexXjType::Dimension, VertexXiType::Dimension);
  } else {
    new (&_hessian) HessianBlockType(d, VertexXiType::Dimension, VertexXjType::Dimension);
  }
  _hessianRowMajor = rowMajor;
}
