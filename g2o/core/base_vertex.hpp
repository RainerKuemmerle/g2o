// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
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


template <int D, typename T>
BaseVertex<D, T>::BaseVertex() :
  OptimizableGraph::Vertex(),
  _hessian(0, D, D)
{
  _dimension = D;
}

template <int D, typename T>
double BaseVertex<D, T>::solveDirect(double lambda) {
  Matrix <double, D, D> tempA=_hessian + Matrix <double, D, D>::Identity()*lambda;
  double det=tempA.determinant();
  if (g2o_isnan(det) || det < std::numeric_limits<double>::epsilon())
    return det;
  Matrix <double, D, 1> dx=tempA.llt().solve(_b);
  oplus(&dx[0]);
  return det;
}

template <int D, typename T>
void BaseVertex<D, T>::clearQuadraticForm() {
  _b.setZero();
}

template <int D, typename T>
void BaseVertex<D, T>::mapHessianMemory(double* d)
{
  new (&_hessian) HessianBlockType(d, D, D);
}
