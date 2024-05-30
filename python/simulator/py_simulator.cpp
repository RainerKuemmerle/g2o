#include "py_simulator.h"

#include <pybind11/detail/common.h>

#include <memory>

#include "g2o/core/optimizable_graph.h"
#include "g2o/simulator/simulator.h"
#include "g2o/simulator/simulator2d_base.h"

namespace g2o {

void declareSimulator(py::module& m) {
  py::class_<World>(m, "World")
      .def("graph",
           static_cast<g2o::OptimizableGraph& (World::*)()>(&World::graph),
           py::return_value_policy::reference);

  py::class_<Simulator>(m, "Simulator")
      .def("seed", &Simulator::seed)
      .def("graph", &Simulator::graph, py::return_value_policy::reference)
      .def("world",
           static_cast<g2o::World& (Simulator::*)()>(&Simulator::world),
           py::return_value_policy::reference);

  // 2D Simulator
  py::class_<Simulator2D::Config>(m, "Simulator2DConfig")
      .def(py::init<>())
      .def_readwrite("world_size", &Simulator2D::Config::worldSize)
      .def_readwrite("nlandmarks", &Simulator2D::Config::nlandmarks)
      .def_readwrite("sim_steps", &Simulator2D::Config::simSteps)
      .def_readwrite("has_odom", &Simulator2D::Config::hasOdom)
      .def_readwrite("has_pose_sensor", &Simulator2D::Config::hasPoseSensor)
      .def_readwrite("has_point_sensor", &Simulator2D::Config::hasPointSensor)
      .def_readwrite("has_compass", &Simulator2D::Config::hasCompass)
      .def_readwrite("has_gps", &Simulator2D::Config::hasGPS)
      .def_readwrite("has_point_bearing_sensor",
                     &Simulator2D::Config::hasPointBearingSensor)
      .def_readwrite("has_segment_sensor",
                     &Simulator2D::Config::hasSegmentSensor)
      .def_readwrite("nsegments", &Simulator2D::Config::nSegments)
      .def_readwrite("segment_grid_size", &Simulator2D::Config::segmentGridSize)
      .def_readwrite("min_segment_length",
                     &Simulator2D::Config::minSegmentLength)
      .def_readwrite("max_segment_length",
                     &Simulator2D::Config::maxSegmentLength);

  py::class_<Simulator2D, Simulator>(m, "Simulator2D")
      .def(py::init<>())
      .def(py::init([](Simulator2D::Config cfg) {
        return std::make_unique<Simulator2D>(std::move(cfg));
      }))
      .def_readwrite("config", &Simulator2D::config)
      .def("setup", &Simulator2D::setup)
      .def("simulate", &Simulator2D::simulate);
}

}  // namespace g2o
