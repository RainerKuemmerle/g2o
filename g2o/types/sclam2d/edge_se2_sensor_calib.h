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

#ifndef G2O_EDGE_SE2_SENSOR_CALIB_H
#define G2O_EDGE_SE2_SENSOR_CALIB_H

#include "g2o_types_sclam2d_api.h"
#include "g2o/core/base_multi_edge.h"
#include "g2o/types/slam2d/vertex_se2.h"

namespace g2o {

  /**
   * \brief scanmatch measurement that also calibrates an offset for the laser
   */
  class G2O_TYPES_SCLAM2D_API EdgeSE2SensorCalib : public BaseMultiEdge<3, SE2>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      EdgeSE2SensorCalib();

      void computeError()
      {
        const VertexSE2* v1          = static_cast<const VertexSE2*>(_vertices[0]);
        const VertexSE2* v2          = static_cast<const VertexSE2*>(_vertices[1]);
        const VertexSE2* laserOffset = static_cast<const VertexSE2*>(_vertices[2]);
        const SE2& x1 = v1->estimate();
        const SE2& x2 = v2->estimate();
        SE2 delta = _inverseMeasurement * ((x1 * laserOffset->estimate()).inverse() * x2 * laserOffset->estimate());
        _error = delta.toVector();
      }

      void setMeasurement(const SE2& m){
        _measurement = m;
        _inverseMeasurement = m.inverse();
      }

      virtual number_t initialEstimatePossible(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to)
      {
        if (   from.count(_vertices[2]) == 1 // need the laser offset
            && ((from.count(_vertices[0]) == 1 && to == _vertices[1]) || ((from.count(_vertices[1]) == 1 && to == _vertices[0])))) {
          return 1.0;
        }
        return -1.0;
      }
      virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

    protected:
      SE2 _inverseMeasurement;
  };

#ifdef G2O_HAVE_OPENGL
  class EdgeSE2SensorCalibDrawAction: public DrawAction {
  public:
    EdgeSE2SensorCalibDrawAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_);
  };
#endif

} // end namespace

#endif
