#include "py_batch_stats.h"

#include "g2o/core/batch_stats.h"

namespace g2o {

void declareG2OBatchStatistics(py::module& m) {
  py::class_<G2OBatchStatistics>(m, "G2OBatchStatistics")
      .def(py::init<>())
      .def_static("global_stats", &G2OBatchStatistics::globalStats)
      .def_static("set_global_stats", &G2OBatchStatistics::setGlobalStats)

      .def_readwrite("iteration", &G2OBatchStatistics::iteration)       // int
      .def_readwrite("num_vertices", &G2OBatchStatistics::numVertices)  // int
      .def_readwrite("num_edges", &G2OBatchStatistics::numEdges)        // int
      .def_readwrite("chi2", &G2OBatchStatistics::chi2)  // double

      // nonlinear part
      .def_readwrite("time_residuals",
                     &G2OBatchStatistics::timeResiduals)  // double
      .def_readwrite("time_linearize",
                     &G2OBatchStatistics::timeLinearize)  // double
      .def_readwrite("time_quadratic_form",
                     &G2OBatchStatistics::timeQuadraticForm)  // double
      .def_readwrite("levenberg_iterations",
                     &G2OBatchStatistics::levenbergIterations)  // int

      // block_solver (constructs Ax=b, plus maybe schur)
      .def_readwrite("time_schur_complement",
                     &G2OBatchStatistics::timeSchurComplement)  // double

      // linear solver (computes Ax=b);
      .def_readwrite("time_symbolic_decomposition",
                     &G2OBatchStatistics::timeSymbolicDecomposition)  // double
      .def_readwrite("time_numeric_decomposition",
                     &G2OBatchStatistics::timeNumericDecomposition)  // double
      .def_readwrite("time_linear_solution",
                     &G2OBatchStatistics::timeLinearSolution)  // double
      .def_readwrite("time_linear_solver",
                     &G2OBatchStatistics::timeLinearSolver)  // double
      .def_readwrite("iterations_linear_solver",
                     &G2OBatchStatistics::iterationsLinearSolver)     // int
      .def_readwrite("time_update", &G2OBatchStatistics::timeUpdate)  // double
      .def_readwrite("time_iteration",
                     &G2OBatchStatistics::timeIteration)  // double
      .def_readwrite("time_marginals",
                     &G2OBatchStatistics::timeMarginals)  // double

      // information about the Hessian matrix
      .def_readwrite("hessian_dimension",
                     &G2OBatchStatistics::hessianDimension)  // size_t
      .def_readwrite("hessian_pose_dimension",
                     &G2OBatchStatistics::hessianPoseDimension)  // size_t
      .def_readwrite("hessian_landmark_dimension",
                     &G2OBatchStatistics::hessianLandmarkDimension)    // size_t
      .def_readwrite("choleskyNNZ", &G2OBatchStatistics::choleskyNNZ)  // size_t
      .def("__str__", [](const G2OBatchStatistics& b) {
        std::stringstream stream;
        stream << b;
        return stream.str();
      });
}

}  // namespace g2o
