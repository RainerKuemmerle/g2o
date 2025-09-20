#include "py_optimization_algorithm.h"

#include "g2o/core/optimization_algorithm.h"
#include "g2o/core/optimization_algorithm_dogleg.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_property.h"
#include "g2o/core/optimization_algorithm_with_hessian.h"
#include "py_block_solver.h"

namespace g2o {

void declareOptimizationAlgorithm(py::module& m) {
  py::classh<OptimizationAlgorithm>(m, "OptimizationAlgorithm");  // NOLINT

  py::classh<OptimizationAlgorithmProperty>(  // NOLINT
      m, "OptimizationAlgorithmProperty");

  py::classh<OptimizationAlgorithmWithHessian, OptimizationAlgorithm>(
      m, "OptimizationAlgorithmWithHessian");

  py::classh<OptimizationAlgorithmGaussNewton,
             OptimizationAlgorithmWithHessian>(
      m, "OptimizationAlgorithmGaussNewton")
      .def(py::init([](PyBlockSolverBase& blockSolver) {
        return new OptimizationAlgorithmGaussNewton(blockSolver.solver());
      }));

  py::classh<OptimizationAlgorithmLevenberg, OptimizationAlgorithmWithHessian>(
      m, "OptimizationAlgorithmLevenberg")
      .def(py::init([](PyBlockSolverBase& blockSolver) {
        return new OptimizationAlgorithmLevenberg(blockSolver.solver());
      }));

  py::classh<OptimizationAlgorithmDogleg, OptimizationAlgorithmWithHessian>(
      m, "OptimizationAlgorithmDogleg")
      .def(py::init([](PyBlockSolverBase& blockSolver) {
        return new OptimizationAlgorithmDogleg(blockSolver.base_solver());
      }));

  py::classh<AbstractOptimizationAlgorithmCreator>(  // NOLINT
      m, "AbstractOptimizationAlgorithmCreator");

  py::classh<RegisterOptimizationAlgorithmProxy>(
      m, "RegisterOptimizationAlgorithmProxy");
}

}  // end namespace g2o
