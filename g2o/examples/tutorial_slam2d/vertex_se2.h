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

#ifndef G2O_TUTORIAL_VERTEX_SE2_H
#define G2O_TUTORIAL_VERTEX_SE2_H

#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "se2.h"
#include "g2o_tutorial_slam2d_api.h"

namespace g2o {
  namespace tutorial {

    /**
     * \brief 2D pose Vertex, (x,y,theta)
     */
    class G2O_TUTORIAL_SLAM2D_API VertexSE2 : public BaseVertex<3, SE2>
    {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        VertexSE2();

        virtual void setToOriginImpl() {
          _estimate=SE2();
        }

        virtual void oplusImpl(const double* update)
        {
          SE2 up(update[0], update[1], update[2]);
          _estimate *= up;
        }

        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;

    };

  } // end namespace
} // end namespace

#endif
