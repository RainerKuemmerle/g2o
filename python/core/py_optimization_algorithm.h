#include "g2o/core/optimization_algorithm.h"
#include "g2o/core/optimization_algorithm_dogleg.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_property.h"
#include "g2o/core/optimization_algorithm_with_hessian.h"
#include "g2opy.h"
#include "py_block_solver.h"

namespace g2o {

void declareOptimizationAlgorithm(py::module& m) {
  py::class_<OptimizationAlgorithm>(m, "OptimizationAlgorithm");

  py::class_<OptimizationAlgorithmProperty>(m, "OptimizationAlgorithmProperty");

  py::class_<OptimizationAlgorithmWithHessian, OptimizationAlgorithm>(
      m, "OptimizationAlgorithmWithHessian");

  py::class_<OptimizationAlgorithmGaussNewton, OptimizationAlgorithmWithHessian>(
      m, "OptimizationAlgorithmGaussNewton")
      .def(py::init([](PyBlockSolverBase& blockSolver) {
        return new OptimizationAlgorithmGaussNewton(blockSolver.solver());
      }));

  py::class_<OptimizationAlgorithmLevenberg, OptimizationAlgorithmWithHessian>(
      m, "OptimizationAlgorithmLevenberg")
      .def(py::init([](PyBlockSolverBase& blockSolver) {
        return new OptimizationAlgorithmLevenberg(blockSolver.solver());
      }));

  py::class_<OptimizationAlgorithmDogleg, OptimizationAlgorithmWithHessian>(
      m, "OptimizationAlgorithmDogleg")
      .def(py::init([](PyBlockSolverBase& blockSolver) {
        return new OptimizationAlgorithmDogleg(blockSolver.base_solver());
      }));

  py::class_<AbstractOptimizationAlgorithmCreator>(m, "AbstractOptimizationAlgorithmCreator");

  py::class_<RegisterOptimizationAlgorithmProxy>(m, "RegisterOptimizationAlgorithmProxy");
}

}  // end namespace g2o
