#pragma once

#include "detail/registry.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/edge_se2_lotsofxy.h"
#include "g2o/types/slam2d/edge_se2_offset.h"
#include "g2o/types/slam2d/edge_se2_prior.h"
#include "g2o/types/slam2d/edge_se2_twopointsxy.h"
#include "g2o/types/slam2d/edge_se2_xyprior.h"

namespace g2o {

inline void declareEdgeSE2(detail::Registry& registry) {
  registry.registerEdgeFixed<EdgeSE2>("EdgeSE2");

#if 0
  py::class_<EdgeSE2LotsOfXY, BaseVariableSizedEdge<-1, VectorX>,
             PyEdgeTrampoline<EdgeSE2LotsOfXY>,
             std::shared_ptr<EdgeSE2LotsOfXY>>(m, "EdgeSE2LotsOfXY")
      .def(py::init<>())
      .def("set_dimension", &EdgeSE2LotsOfXY::setDimension<-1>)
      .def("resize", &EdgeSE2LotsOfXY::resize)

      .def("compute_error", &EdgeSE2LotsOfXY::computeError)
      .def("set_measurement_from_state",
           &EdgeSE2LotsOfXY::setMeasurementFromState)
      .def("initial_estimate_possible",
           &EdgeSE2LotsOfXY::initialEstimatePossible)
      .def("initial_estimate", &EdgeSE2LotsOfXY::initialEstimate)
      .def("linearize_oplus", &EdgeSE2LotsOfXY::linearizeOplus);
#endif

  registry.registerEdgeFixed<EdgeSE2Offset>("EdgeSE2Offset");

  registry.registerEdgeFixed<EdgeSE2Prior>("EdgeSE2Prior");

  registry.registerEdgeFixed<EdgeSE2TwoPointsXY>("EdgeSE2TwoPointsXY");

  registry.registerEdgeFixed<EdgeSE2XYPrior>("EdgeSE2XYPrior");
}

}  // end namespace g2o
