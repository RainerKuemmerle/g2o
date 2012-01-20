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

#ifndef G2O_TUTORIAL_PARAMETER_SE2_OFFSET_H
#define G2O_TUTORIAL_PARAMETER_SE2_OFFSET_H

#include "g2o/core/cache.h"
#include "se2.h"
#include "g2o_tutorial_slam2d_api.h"

namespace g2o {
  namespace tutorial {

    class G2O_TUTORIAL_SLAM2D_API ParameterSE2Offset: public Parameter
    {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        ParameterSE2Offset();

        void setOffset(const SE2& offset = SE2());

        const SE2& offset() const { return _offset;}
        const SE2& inverseOffset() const { return _inverseOffset;}

        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;

      protected:
        SE2 _offset;
        SE2 _inverseOffset;
    };

    class G2O_TUTORIAL_SLAM2D_API CacheSE2Offset: public Cache
    {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        const SE2& w2n() const { return _w2n;}
        const SE2& n2w() const { return _n2w;}

      protected:
        virtual void updateImpl();
        virtual bool resolveDependancies();

        ParameterSE2Offset* _offsetParam;
        SE2 _w2n, _n2w; 
    };

  } // end namespace tutorial
} // end namespace

#endif
