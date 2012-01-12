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

#include "solver.h"

#include <cstring>
#include <algorithm>

namespace g2o {

Solver::Solver() :
  _optimizer(0), _x(0), _b(0), _xSize(0), _maxXSize(0),
  _isLevenberg(false), _additionalVectorSpace(0)
{
}

Solver::~Solver()
{
  delete[] _x;
  delete[] _b;
}

void Solver::resizeVector(size_t sx)
{
  size_t oldSize = _xSize;
  _xSize = sx;
  sx += _additionalVectorSpace; // allocate some additional space if requested
  if (_maxXSize < sx) {
    _maxXSize = 2*sx;
    delete[] _x;
    _x = new double[_maxXSize];
#ifndef NDEBUG
    memset(_x, 0, _maxXSize * sizeof(double));
#endif
    if (_b) { // backup the former b, might still be needed for online processing
      memcpy(_x, _b, oldSize * sizeof(double));
      delete[] _b;
      _b = new double[_maxXSize];
      std::swap(_b, _x);
    } else {
      _b = new double[_maxXSize];
#ifndef NDEBUG
      memset(_b, 0, _maxXSize * sizeof(double));
#endif
    }
  }
}

void Solver::setOptimizer(SparseOptimizer* optimizer)
{
  _optimizer = optimizer;
}

void Solver::setLevenberg(bool levenberg)
{
  _isLevenberg = levenberg;
}

void Solver::setAdditionalVectorSpace(size_t s)
{
  _additionalVectorSpace = s;
}

} // end namespace
