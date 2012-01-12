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

#include "parameter_se2_offset.h"

#include "vertex_se2.h"

namespace g2o {
  namespace tutorial {

    ParameterSE2Offset::ParameterSE2Offset()
    {
    }

    void ParameterSE2Offset::setOffset(const SE2& offset)
    {
      _offset = offset;
      _inverseOffset = offset.inverse();
    }

    bool ParameterSE2Offset::read(std::istream& is)
    {
      double x, y, th;
      is >> x >> y >> th;
      setOffset(SE2(x, y, th));
      return true;
    }

    bool ParameterSE2Offset::write(std::ostream& os) const
    {
      os << _offset.translation().x() << " " << _offset.translation().y() << " " << _offset.rotation().angle();
      return os.good();
    }

    void CacheSE2Offset::updateImpl()
    {
      const VertexSE2* v = static_cast<const VertexSE2*>(vertex());
      _n2w = v->estimate() * _offsetParam->offset();
      _w2n = _n2w.inverse();
    }

    bool CacheSE2Offset::resolveDependancies()
    {
      _offsetParam = dynamic_cast<ParameterSE2Offset*> (_parameters[0]);
      return _offsetParam != 0;
    }

  } // end namespace tutorial
} // end namespace
