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

template <int D, typename E, typename VertexXiType>
void BaseUnaryEdge<D, E, VertexXiType>::resize(size_t size)
{
  if (size != 1) {
    std::cerr << "WARNING, attempting to resize unary edge " << BaseEdge<D, E>::id() << " to " << size << std::endl;
  }
  BaseEdge<D, E>::resize(size);
}

template <int D, typename E, typename VertexXiType>
void BaseUnaryEdge<D, E, VertexXiType>::constructQuadraticForm()
{
  VertexXiType* from=static_cast<VertexXiType*>(_vertices[0]);

  // chain rule to get the Jacobian of the nodes in the manifold domain
  const JacobianXiOplusType& A = jacobianOplusXi();
  const InformationType& omega = _information;

  bool istatus = !from->fixed();
  if (istatus) {
#ifdef G2O_OPENMP
    from->lockQuadraticForm();
#endif
    from->b().noalias() -= A.transpose() * omega * _error;
    from->A().noalias() += A.transpose() * omega * A;
#ifdef G2O_OPENMP
    from->unlockQuadraticForm();
#endif
  }
}

template <int D, typename E, typename VertexXiType>
void BaseUnaryEdge<D, E, VertexXiType>::linearizeOplus()
{
  //Xi - estimate the jacobian numerically
  VertexXiType* vi = static_cast<VertexXiType*>(_vertices[0]);

  if (vi->fixed())
    return;

#ifdef G2O_OPENMP
  vi->lockQuadraticForm();
#endif

  const double delta = 1e-9;
  const double scalar = 1.0 / (2*delta);
  ErrorVector error1;
  ErrorVector errorBeforeNumeric = _error;

  double add_vi[VertexXiType::Dimension];
  std::fill(add_vi, add_vi + VertexXiType::Dimension, 0.0);
  // add small step along the unit vector in each dimension
  for (int d = 0; d < VertexXiType::Dimension; ++d) {
    vi->push();
    add_vi[d] = delta;
    vi->oplus(add_vi);
    computeError();
    error1 = _error;
    vi->pop();
    vi->push();
    add_vi[d] = -delta;
    vi->oplus(add_vi);
    computeError();
    vi->pop();
    add_vi[d] = 0.0;

    _jacobianOplusXi.col(d) = scalar * (error1 - _error);
  } // end dimension

  _error = errorBeforeNumeric;
#ifdef G2O_OPENMP
  vi->unlockQuadraticForm();
#endif
}

template <int D, typename E, typename VertexXiType>
void BaseUnaryEdge<D, E, VertexXiType>::initialEstimate(const OptimizableGraph::VertexSet&, OptimizableGraph::Vertex*)
{
  std::cerr << __PRETTY_FUNCTION__ << " is not implemented, please give implementation in your derived class" << std::endl;
}
