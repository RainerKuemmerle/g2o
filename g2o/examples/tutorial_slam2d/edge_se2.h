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

#ifndef G2O_TUTORIAL_EDGE_SE2_H
#define G2O_TUTORIAL_EDGE_SE2_H

#include "vertex_se2.h"
#include "g2o_tutorial_slam2d_api.h"
#include "g2o/core/base_binary_edge.h"

namespace g2o {

  namespace tutorial {

    /**
     * \brief 2D edge between two Vertex2, i.e., the odometry
     */
    class G2O_TUTORIAL_SLAM2D_API EdgeSE2 : public BaseBinaryEdge<3, SE2, VertexSE2, VertexSE2>
    {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgeSE2();

        void computeError()
        {
          const VertexSE2* v1 = static_cast<const VertexSE2*>(_vertices[0]);
          const VertexSE2* v2 = static_cast<const VertexSE2*>(_vertices[1]);
          SE2 delta = _inverseMeasurement * (v1->estimate().inverse()*v2->estimate());
          _error = delta.toVector();
        }
  
        void setMeasurement(const SE2& m){
          _measurement = m;
          _inverseMeasurement = m.inverse();
        }

        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;

      protected:
        SE2 _inverseMeasurement;
    };

  }

} // end namespace

#endif
