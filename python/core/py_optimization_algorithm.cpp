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
  py::class_<OptimizationAlgorithm,  // NOLINT
             std::shared_ptr<OptimizationAlgorithm>>(m,
                                                     "OptimizationAlgorithm");

  py::class_<OptimizationAlgorithmProperty>(  // NOLINT
      m, "OptimizationAlgorithmProperty");

  py::class_<OptimizationAlgorithmWithHessian, OptimizationAlgorithm,  // NOLINT
             std::shared_ptr<OptimizationAlgorithmWithHessian>>(
      m, "OptimizationAlgorithmWithHessian");

  py::class_<OptimizationAlgorithmGaussNewton, OptimizationAlgorithmWithHessian,
             std::shared_ptr<OptimizationAlgorithmGaussNewton>>(
      m, "OptimizationAlgorithmGaussNewton")
      .def(py::init([](PyBlockSolverBase& blockSolver) {
        return new OptimizationAlgorithmGaussNewton(blockSolver.solver());
      }));

  py::class_<OptimizationAlgorithmLevenberg, OptimizationAlgorithmWithHessian,
             std::shared_ptr<OptimizationAlgorithmLevenberg>>(
      m, "OptimizationAlgorithmLevenberg")
      .def(py::init([](PyBlockSolverBase& blockSolver) {
        return new OptimizationAlgorithmLevenberg(blockSolver.solver());
      }));

  py::class_<OptimizationAlgorithmDogleg, OptimizationAlgorithmWithHessian,
             std::shared_ptr<OptimizationAlgorithmDogleg>>(
      m, "OptimizationAlgorithmDogleg")
      .def(py::init([](PyBlockSolverBase& blockSolver) {
        return new OptimizationAlgorithmDogleg(blockSolver.base_solver());
      }));

  py::class_<AbstractOptimizationAlgorithmCreator>(  // NOLINT
      m, "AbstractOptimizationAlgorithmCreator");

  py::class_<RegisterOptimizationAlgorithmProxy>(
      m, "RegisterOptimizationAlgorithmProxy");
}

}  // end namespace g2o
